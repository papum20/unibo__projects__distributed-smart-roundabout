import asyncio
import logging
import math
import random
import time

import aiomqtt
from uuid import uuid4

from common import math_utils, physics, roundabout
from common.const import FRAMERATE, AREA_RADIUS, ROUNDABOUT_N_ROADS, ROUNDABOUT_POS, ROUNDABOUT_RADIUS, VEHICLE_ANGLE_TOL_RAD
from common.get_env import config
from common.models.models import Command, Position, Vehicle, VehicleNavState, VehicleState



logging.basicConfig(level=logging.INFO if not config.DEBUG_MODE else logging.DEBUG)
logger = logging.getLogger(__name__)

TIMER_STATE_S				= 1.0 / FRAMERATE
TIMER_NETWORK_FAILSAFE_S	= 2.0


vehicle_id = str(uuid4())
logger.info("Generated vehicle_id: %s", vehicle_id)

vehicle = Vehicle(
	id			= vehicle_id,
	pos			= Position(x=45.0, y=9.0),
	pos_angle	= 0.0,
	speed		= 10.0
)



def evaluate_failsafe(
	vehicle:			Vehicle,
	current_time:		float,
	last_net_update_t:	float,
	timeout_limit:		float = TIMER_NETWORK_FAILSAFE_S
) -> tuple[Vehicle, Command]:
	"""
	Check if the network connection to the controller is lost and, in case, go into FAILSAFE mode.
	@return an updated vehicle state and a Command to self.
	"""
	if current_time - last_net_update_t > timeout_limit:
		updated_vehicle	= vehicle.model_copy(update={"state": VehicleState.FAILSAFE})
		brake_command	= Command(target_acceleration=-5.0)
		logger.warning("Network lost. FAILSAFE triggered.")
		return updated_vehicle, brake_command
		
	# do nothing (listen to controller commands)
	return vehicle, Command(target_acceleration=0.0)



def navigate_vehicle(dt: float, v: Vehicle = vehicle) -> Vehicle:
	if v.speed == 0:
		# stopped, do not move
		return v

	if v.nav_state == VehicleNavState.APPROACHING:
		target_x, target_y	= roundabout.get_point_on_road(v.entry_road, distance_from_boundary=0.0)
		entry_target		= Position(x=target_x, y=target_y)
		
		v.pos = physics.move_towards(v.pos, entry_target, v.speed, dt)
		
		if math_utils.get_dist(v.pos, entry_target) <= 0.0:
			v.nav_state	= VehicleNavState.IN_ROUNDABOUT
			v.pos_angle = roundabout.get_road_angle(v.entry_road)
			logger.info("Vehicle %s entered the roundabout (IN_ROUNDABOUT) on road %d", v.id, v.entry_road)
			
	elif v.nav_state == VehicleNavState.IN_ROUNDABOUT:
		v.pos_angle, v.pos = physics.move_on_circle(
			ROUNDABOUT_POS, ROUNDABOUT_RADIUS, v.pos_angle, v.speed, dt
		)
		
		# check if we reached the exit road
		exit_angle = roundabout.get_road_angle(v.exit_road)
		angle_diff = abs(v.pos_angle - exit_angle)
		# handle wrap-around at 2*pi
		angle_diff = min(angle_diff, 2*math.pi - angle_diff) 
		
		if angle_diff < VEHICLE_ANGLE_TOL_RAD:
			v.nav_state = VehicleNavState.EXITING
			logger.info("Vehicle %s is exiting the roundabout (EXITING) on road %d", v.id, v.exit_road)

	elif v.nav_state == VehicleNavState.EXITING:
		# target is a point far away on the exit road
		far_x, far_y	= roundabout.get_point_on_road(v.exit_road, distance_from_boundary=9999.)
		v.pos			= physics.move_towards(v.pos, Position(x=far_x, y=far_y), v.speed, dt)
		
	return v


def reset_vehicle(v: Vehicle, n_roads: int=ROUNDABOUT_N_ROADS) -> Vehicle:
	"""Spawn or respawns the vehicle on a random road"""
	road_entry	= random.randint(0, n_roads - 1)
	road_exit	= random.randint(0, n_roads - 1)
	
	# Random distance between 50 and 80 meters away from the roundabout
	spawn_dist = random.uniform(50.0, 80.0) 
	start_x, start_y = roundabout.get_point_on_road(road_entry, spawn_dist, n_roads=n_roads)
	
	v.pos			= Position(x=start_x, y=start_y)
	v.pos_angle		= roundabout.get_road_angle(road_entry, n_roads=n_roads)
	v.speed			= random.uniform(8.0, 12.0) # Random starting speed
	v.state			= VehicleState.NORMAL
	v.entry_road	= road_entry
	v.exit_road		= road_exit
	v.nav_state		= VehicleNavState.APPROACHING

	global vehicle
	vehicle = v
	logger.info("Reset vehicle to: %s", v.model_dump_json())
	return v



async def main():
	global vehicle
	async with aiomqtt.Client(hostname=config.HOST_BROKER, port=config.PORT_BROKER) as client:
		last_time				= time.time()
		last_net_update_time	= time.time()
		
		while True:
			current_time	= time.time()
			dt				= current_time - last_time


			vehicle, command = evaluate_failsafe(vehicle, current_time, last_net_update_time)

			vehicle.speed = physics.update_speed(
				speed		= vehicle.speed, 
				acc			= command.target_acceleration, 
				dt			= dt, 
				max_speed	= vehicle.params.max_speed
			)
			vehicle = navigate_vehicle(dt)

			last_time				= current_time
			last_net_update_time	= current_time

			if vehicle.nav_state == VehicleNavState.EXITING and math_utils.get_dist(vehicle.pos, ROUNDABOUT_POS) > AREA_RADIUS:
				vehicle = reset_vehicle(vehicle)


			await client.publish(f'{config.TOPIC_VEHICLE_PREFIX}/{vehicle_id}/{config.TOPIC_VEHICLE_TELEMETRY_SUFFIX}', payload=vehicle.model_dump_json())
			logger.debug("Published to topic %s: %s", f'{config.TOPIC_VEHICLE_PREFIX}/{vehicle_id}/{config.TOPIC_VEHICLE_TELEMETRY_SUFFIX}', vehicle.model_dump_json())

			await asyncio.sleep(TIMER_STATE_S)



if __name__ == "__main__":
	reset_vehicle(vehicle)
	asyncio.run(main())