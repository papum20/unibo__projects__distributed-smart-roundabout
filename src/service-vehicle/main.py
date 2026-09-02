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
	UPDATES_P_S_VEHICLE,
	AREA_RADIUS,
	LANE_WIDTH,
	ROAD_LENGTH,
	ROUNDABOUT_N_ROADS,
	ROUNDABOUT_POS,
	ROUNDABOUT_RADIUS,
	VEHICLE_ANGLE_TOL_RAD,
	VEHICLE_ANGLE_TRAVELED_MIN_RAD,
	VEHICLE_DIST_TOL
)
from common.get_env import config
from common.models.models import Command, Position, Vehicle, VehicleNavState, VehicleState, SystemCommand, SystemCommandValue



logger = logging.getLogger(__name__)

TIMER_NETWORK_FAILSAFE_S	= 2.0


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

		self.is_paused = False

state = RuntimeState()



def navigate_vehicle(dt: float, v: Vehicle) -> Vehicle:
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


def reset_vehicle(v: Vehicle, n_roads: int=ROUNDABOUT_N_ROADS):
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




async def listen_commands(client, s: RuntimeState = state):
	command_topic = f"{config.TOPIC_VEHICLE_PREFIX}/{vehicle_id}/{config.TOPIC_VEHICLE_COMMAND_SUFFIX}"
	await client.subscribe(command_topic)
	await client.subscribe(config.TOPIC_SYSTEM_CONTROL)
	
	async for message in client.messages:
		payload = json.loads(message.payload)

		if message.topic.matches("system/control"):
			command = SystemCommand(**payload)
			if command.command == SystemCommandValue.PAUSE:
				s.is_paused = True
				logger.info("Simulation PAUSED")
			elif command.command == SystemCommandValue.RESUME:
				s.is_paused = False
				s.last_net_update_time = time.time()	# prevent instant failsafe
				logger.info("Simulation RESUMED")

		else:
			s.last_command			= Command(**payload)
			s.last_net_update_time	= time.time()
			logger.debug("Received command: %s", s.last_command)



async def physics_loop(client, s: RuntimeState = state):
	last_time = time.time()
	
	while True:
		current_time	= time.time()
		dt				= current_time - last_time
		last_time		= current_time

		if s.is_paused:
			# update the failsafe timer so it doesn't trigger during pause
			s.last_net_update_time = current_time 
			await asyncio.sleep(1.0 / UPDATES_P_S_VEHICLE)
			continue

		# check for command, otherwise failsafe
		if current_time - s.last_net_update_time > TIMER_NETWORK_FAILSAFE_S:
			s.vehicle.state		= VehicleState.FAILSAFE
			active_command		= Command(target_acceleration=-s.vehicle.params.max_brake)
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
		s.vehicle = navigate_vehicle(dt, s.vehicle)

		if s.vehicle.nav_state == VehicleNavState.EXITING and math_utils.get_dist(s.vehicle.pos, ROUNDABOUT_POS) > AREA_RADIUS:
			reset_vehicle(s.vehicle)

		topic = f'{config.TOPIC_VEHICLE_PREFIX}/{vehicle_id}/{config.TOPIC_VEHICLE_TELEMETRY_SUFFIX}'
		await client.publish(topic, payload=s.vehicle.model_dump_json())
		logger.debug("Published to topic %s: %s", f'{config.TOPIC_VEHICLE_PREFIX}/{vehicle_id}/{config.TOPIC_VEHICLE_TELEMETRY_SUFFIX}', s.vehicle.model_dump_json())

		await asyncio.sleep(1.0 / UPDATES_P_S_VEHICLE)


async def main(s: RuntimeState = state):
	reset_vehicle(s.vehicle)
	# random boot delay so cars don't overlap on startup
	await asyncio.sleep(random.uniform(0.0, 5.0))
	
	async with aiomqtt.Client(hostname=config.HOST_BROKER, port=config.PORT_BROKER) as client:
		# run both concurrently
		await asyncio.gather(
			listen_commands(client, s),
			physics_loop(client, s)
		)



if __name__ == "__main__":
	asyncio.run(main())