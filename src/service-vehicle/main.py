import asyncio
import json
import logging
import math
import random
import time

import aiomqtt
from uuid import uuid4

from common import math_utils, physics, roundabout
from common.const import (
	CAR_LENGTH,
	ROAD_WIDTH,
	ROUNDABOUT_PROXIMITY_DIST,
	TIMER_NETWORK_TIMEOUT,
	UPDATES_P_S_VEHICLE,
	AREA_RADIUS,
	LANE_WIDTH,
	ROAD_LENGTH,
	ROUNDABOUT_N_ROADS,
	ROUNDABOUT_POS,
	ROUNDABOUT_RADIUS,
	VEHICLE_ANGLE_TOL_RAD,
	VEHICLE_ANGLE_TRAVELED_MIN_RAD,
	VEHICLE_DIST_TOL,
	VEHICLE_SPEED_TOL_PERC
)
from common.get_env import config
from common.models.models import (
	VEHICLE_FAILSAFE_MAX_SPEED_M_S,
	Command, Position, Vehicle, VehicleNavState, VehiclePosition, VehicleState, SystemCommand, SystemCommandValue
)



logger = logging.getLogger(__name__)

TIMER_NETWORK_FAILSAFE_S	= TIMER_NETWORK_TIMEOUT


vehicle_id = str(uuid4())
logger.info("Generated vehicle_id: %s", vehicle_id)

class RuntimeState:
	def __init__(self):
		self.vehicle = Vehicle(
			id			= vehicle_id,
			pos			= Position(x=45.0, y=9.0),
			pos_angle	= 0.0,
			speed		= 10.0
		)

		# global variables for network state
		self.last_command			= Command(target_acceleration=0.0)
		self.last_net_update_time	= time.time()

		self.sysctrl_pause			= False
		self.sysctrl_failsafe		= False
		self.sysctrl_disconnected	= False

		# v_id -> (VehiclePosition, timestamp)
		self.local_vision_vehicles: dict[str, tuple[VehiclePosition, float]] = {}

state = RuntimeState()



def vehicle_navigate(dt: float, v: Vehicle) -> Vehicle:
	if v.speed == 0 and v.acceleration <= 0:
		# stopped, do not move
		return v

	if v.nav_state == VehicleNavState.APPROACHING:
		# drive on the right side of the road (positive offset)
		target_x, target_y	= roundabout.get_point_on_road(
			v.entry_road, distance_from_boundary=0.0, lane_offset=LANE_WIDTH/2
		)
		entry_target		= Position(x=target_x, y=target_y)
		
		v.pos = physics.move_towards(v.pos, entry_target, v.speed, dt)
		
		if math_utils.get_dist(v.pos, entry_target) <= VEHICLE_DIST_TOL:
			v.nav_state			= VehicleNavState.IN_ROUNDABOUT
			v.pos_angle			= roundabout.get_road_angle(v.entry_road)
			v.angle_traveled	= 0.0
			logger.info("Vehicle %s entered the roundabout (IN_ROUNDABOUT) on road %d", v.id, v.entry_road)
			
	elif v.nav_state == VehicleNavState.IN_ROUNDABOUT:
		old_angle	= v.pos_angle

		v.pos_angle, v.pos = physics.move_on_circle(
			ROUNDABOUT_POS, ROUNDABOUT_RADIUS, v.pos_angle, v.speed, dt
		)

		angle_diff_frame = (v.pos_angle - old_angle) % (2 * math.pi)
		v.angle_traveled += angle_diff_frame
		
		# check if we reached the exit road
		exit_angle = roundabout.get_road_angle(v.exit_road)
		angle_diff = abs(v.pos_angle - exit_angle)
		# handle wrap-around at 2*pi
		angle_diff = min(angle_diff, 2*math.pi - angle_diff) 

		# make sure to not exit immediately (if entry==exit)
		if angle_diff < VEHICLE_ANGLE_TOL_RAD and v.angle_traveled > VEHICLE_ANGLE_TRAVELED_MIN_RAD:
			v.nav_state		= VehicleNavState.EXITING
			# snap angle, or won't drive straight
			v.pos_angle		= exit_angle
			# snap position to correct lane
			exit_x, exit_y	= roundabout.get_point_on_road(
				v.exit_road, distance_from_boundary=0.0, lane_offset=-LANE_WIDTH / 2
			)
			v.pos = Position(x=exit_x, y=exit_y)
			logger.info("Vehicle %s is exiting the roundabout (EXITING) on road %d", v.id, v.exit_road)

	elif v.nav_state == VehicleNavState.EXITING:
		v.pos = physics.vehicle_move_on_direction(v, dt)
		
	return v


def vehicle_navigate_spawn(
	v				: Vehicle,
	other_positions	: list[VehiclePosition],
) -> Command | None:
	"""
	Return a Command when overlapping with other vehicles (e.g. on spawn), otherwise None.
	Important at spawn point, where vehicles overlap.
	"""
	if v.nav_state != VehicleNavState.APPROACHING:
		return None
	
	v_pos			= vehicle_to_pos(v)
	v_dist			= math_utils.get_dist(v.pos, ROUNDABOUT_POS)
	collisions_n	= 0

	for other in other_positions:
		if other.nav_state != v_pos.nav_state:
			continue
		if not physics.vehicle_collide(v_pos, other):
			continue
		collisions_n += 1

		other_distance = math_utils.get_dist(other.pos, ROUNDABOUT_POS)
		if other_distance < v_dist:
			logger.debug("Vehicle %s waiting for closer vehicle %s", v.id, other.id)
			return Command(target_acceleration=-v.params.max_brake)
	if collisions_n > 0:
		return Command(target_acceleration=v.params.max_accel)
	return None


def evaluate_failsafe(v1: Vehicle, v_others: list[VehiclePosition]) -> Command:
	""" 
	Failsafe is a degraded mode: no central controller. 
	The car relies strictly on stopping and checking local distances.
	"""
	# entrance: stop and check
	dist_to_entry = math_utils.get_dist(v1.pos, ROUNDABOUT_POS) - ROUNDABOUT_RADIUS - ROAD_WIDTH
	if (
		v1.nav_state == VehicleNavState.APPROACHING and v1.speed > 0
		# if already stopped close to the entrance, continue (otherwise will never enter)
		and physics.get_stop_distance_vehicle(v1) >= dist_to_entry > ROUNDABOUT_PROXIMITY_DIST
	):
		return Command(target_acceleration=-v1.params.max_brake)

	new_acc = v1.params.max_accel

	# slower speed
	if abs(v1.speed * VEHICLE_SPEED_TOL_PERC - VEHICLE_FAILSAFE_MAX_SPEED_M_S) > 0:
		if v1.speed > VEHICLE_FAILSAFE_MAX_SPEED_M_S:
			new_acc = -v1.params.max_brake * 0.5
	else:
		new_acc = 0.0

	for v2 in v_others:
		safe_dist = physics.get_safety_distance(v1, margin=CAR_LENGTH/2.0)
		
		# don't rear-end the car in front
		if v1.nav_state == v2.nav_state:
			# same road (calculating the angle from the road, it must be the same)
			if (
				(v1.nav_state in [VehicleNavState.APPROACHING, VehicleNavState.EXITING])
				and v1.pos_angle == v2.pos_angle
			):
				v1_dist	= math_utils.get_dist(v1.pos, ROUNDABOUT_POS)
				v2_dist = math_utils.get_dist(v2.pos, ROUNDABOUT_POS)
				if (
					(v1_dist - v2_dist) < safe_dist
					and ((	v1_dist > v2_dist and v1.nav_state == VehicleNavState.APPROACHING)
		  				or (v2_dist > v1_dist and v1.nav_state == VehicleNavState.EXITING))
				):
					new_acc = min(new_acc, -v1.params.max_brake)
					
			elif v1.nav_state == VehicleNavState.IN_ROUNDABOUT:
				arc_dist = math_utils.get_dist_on_circle(v1.pos_angle, v2.pos_angle)
				if 0 < arc_dist < safe_dist:
					new_acc = min(new_acc, -v1.params.max_brake)

		# entrance
		elif v1.nav_state == VehicleNavState.APPROACHING and v2.nav_state == VehicleNavState.IN_ROUNDABOUT:
			v1_dist_to_conflict = math_utils.get_dist(v1.pos, ROUNDABOUT_POS) - ROUNDABOUT_RADIUS - ROAD_WIDTH
			
			if v1_dist_to_conflict < ROUNDABOUT_PROXIMITY_DIST:
				conflict_angle		= roundabout.get_road_angle(v1.entry_road)
				v2_dist_to_conflict = math_utils.get_dist_on_circle(v2.pos_angle, conflict_angle)
				
				# most cautios safety distance
				if v2_dist_to_conflict <= physics.get_stop_distance(v2):
					new_acc = min(new_acc, -v1.params.max_brake)

	return Command(target_acceleration=new_acc)



def vehicle_reset(v: Vehicle, n_roads: int=ROUNDABOUT_N_ROADS):
	"""Spawn or respawns the vehicle on a random road"""
	road_entry	= random.randint(0, n_roads - 1)
	road_exit	= random.randint(0, n_roads - 1)
	
	# Random distance between 50 and 80 meters away from the roundabout.
	# Spawn on right lane.
	spawn_dist	= random.uniform(ROAD_LENGTH * 0.9, ROAD_LENGTH) 
	start_x, start_y = roundabout.get_point_on_road(road_entry, spawn_dist, n_roads=n_roads, lane_offset=LANE_WIDTH/2)
	
	v.pos			= Position(x=start_x, y=start_y)
	v.pos_angle		= roundabout.get_road_angle(road_entry, n_roads=n_roads)
	v.speed			= random.uniform(8.0, 12.0) # Random starting speed
	v.state			= VehicleState.NORMAL
	v.entry_road	= road_entry
	v.exit_road		= road_exit
	v.nav_state		= VehicleNavState.APPROACHING
	v.color_hue				= random.randint(70, 290)		# random shade of blue is 200-260
	v.color_lightness_perc	= random.uniform(15.0, 85.0)

	logger.info("Reset vehicle to: %s", v.model_dump_json())


def vehicle_to_pos(v: Vehicle) -> VehiclePosition:
	return VehiclePosition(
		id			= v.id,
		pos			= v.pos,
		pos_angle	= v.pos_angle,
		speed		= v.speed,
		nav_state	= v.nav_state
	)



async def loop_listen_commands(client, s: RuntimeState = state):
	command_topic			= f"{config.TOPIC_VEHICLE_PREFIX}/{vehicle_id}/{config.TOPIC_VEHICLE_COMMAND_SUFFIX}"
	positions_topic			= f"{config.TOPIC_VEHICLE_PREFIX}/+/{config.TOPIC_VEHICLE_POSITION_SUFFIX}"
	sysctrl_topic			= f"{config.TOPIC_SYSCTRL_PREFIX}/{config.TOPIC_SYSCTRL_BROADCAST_SUFFIX}"
	sysctrl_broadcast_topic = f"{config.TOPIC_SYSCTRL_PREFIX}/{vehicle_id}"

	await client.subscribe(command_topic)
	await client.subscribe(positions_topic)
	await client.subscribe(sysctrl_topic)
	await client.subscribe(sysctrl_broadcast_topic)
	
	async for message in client.messages:
		payload = json.loads(message.payload)

		if str(message.topic) in (sysctrl_topic, sysctrl_broadcast_topic):
			command = SystemCommand(**payload)
			if command.command == SystemCommandValue.PAUSE:
				s.sysctrl_pause = True
				logger.info("SysCtrl: Simulation PAUSED")
			elif command.command == SystemCommandValue.RESUME:
				s.sysctrl_pause = False
				s.last_net_update_time = time.time()	# prevent instant failsafe
				logger.info("SysCtrl: Simulation RESUMED")
			elif command.command == SystemCommandValue.ENTER_FAILSAFE:
				s.sysctrl_failsafe = True
				logger.info("SysCtrl: ENTER FAILSAFE")
			elif command.command == SystemCommandValue.EXIT_FAILSAFE:
				s.sysctrl_failsafe = False
				logger.info("SysCtrl: EXIT FAILSAFE")
			elif command.command == SystemCommandValue.ENTER_DISCONNECTED:
				s.sysctrl_disconnected = True
				logger.info("SysCtrl: ENTER DISCONNECTED")
			elif command.command == SystemCommandValue.EXIT_DISCONNECTED:
				s.sysctrl_disconnected = False
				logger.info("SysCtrl: EXIT DISCONNECTED")

		elif message.topic.matches(positions_topic):
			v_pos								= VehiclePosition(**payload)
			if v_pos.id != vehicle_id:
				s.local_vision_vehicles[v_pos.id]	= (v_pos, time.time())

		elif message.topic.matches(command_topic):
			s.last_command			= Command(**payload)
			s.last_net_update_time	= time.time()
			logger.debug("Received command: %s", s.last_command)



async def loop_physics(client, s: RuntimeState = state):
	last_time = time.time()
	
	while True:
		current_time	= time.time()
		dt				= current_time - last_time
		last_time		= current_time

		if s.sysctrl_pause:
			# update the failsafe timer so it doesn't trigger during pause
			s.last_net_update_time = current_time 
			await asyncio.sleep(1.0 / UPDATES_P_S_VEHICLE)
			continue

		recent_positions = [
			v for v, t in s.local_vision_vehicles.values()
			if current_time - t < TIMER_NETWORK_TIMEOUT
		]
		spawn_command = vehicle_navigate_spawn(s.vehicle, recent_positions)
		if spawn_command is not None:
			active_command	= spawn_command

		else:
			# check for command, otherwise failsafe
			if s.sysctrl_failsafe or s.sysctrl_disconnected or current_time - s.last_net_update_time > TIMER_NETWORK_FAILSAFE_S:
				if s.sysctrl_disconnected:
					s.vehicle.state = VehicleState.DISCONNECTED
				else:
					s.vehicle.state = VehicleState.FAILSAFE
				#active_command	= Command(target_acceleration=-s.vehicle.params.max_brake)
				v_others = [
					vehicle for vehicle, received_at in s.local_vision_vehicles.values()
					if current_time - received_at < TIMER_NETWORK_FAILSAFE_S
				]
				active_command	= evaluate_failsafe(s.vehicle, v_others)
				if not s.sysctrl_failsafe:
					logger.warning("Network lost. FAILSAFE triggered.")
			else:
				s.vehicle.state	= VehicleState.NORMAL
				active_command	= s.last_command

		s.vehicle.acceleration = active_command.target_acceleration
		s.vehicle.speed = physics.update_speed(
			speed		= s.vehicle.speed, 
			acc			= s.vehicle.acceleration, 
			dt			= dt, 
			max_speed	= s.vehicle.params.max_speed
		)
		s.vehicle = vehicle_navigate(dt, s.vehicle)

		if s.vehicle.nav_state == VehicleNavState.EXITING and math_utils.get_dist(s.vehicle.pos, ROUNDABOUT_POS) > AREA_RADIUS:
			vehicle_reset(s.vehicle)
		v_pos = vehicle_to_pos(s.vehicle)

		telemetry_topic	= f'{config.TOPIC_VEHICLE_PREFIX}/{vehicle_id}/{config.TOPIC_VEHICLE_TELEMETRY_SUFFIX}'
		await client.publish(telemetry_topic, payload=s.vehicle.model_dump_json())
		logger.debug("Published to topic %s: %s", f'{config.TOPIC_VEHICLE_PREFIX}/{vehicle_id}/{config.TOPIC_VEHICLE_TELEMETRY_SUFFIX}', s.vehicle.model_dump_json())
		pos_topic		= f'{config.TOPIC_VEHICLE_PREFIX}/{vehicle_id}/{config.TOPIC_VEHICLE_POSITION_SUFFIX}'
		await client.publish(pos_topic, payload=v_pos.model_dump_json())
		logger.debug("Published to topic %s: %s", pos_topic, v_pos.model_dump_json())

		await asyncio.sleep(1.0 / UPDATES_P_S_VEHICLE)


async def main(s: RuntimeState = state):
	vehicle_reset(s.vehicle)
	# random boot delay so cars don't overlap on startup
	await asyncio.sleep(random.uniform(0.0, 5.0))
	
	async with aiomqtt.Client(hostname=config.HOST_BROKER, port=config.PORT_BROKER) as client:
		# run both concurrently
		await asyncio.gather(
			loop_listen_commands(client, s),
			loop_physics(client, s)
		)



if __name__ == "__main__":
	asyncio.run(main())