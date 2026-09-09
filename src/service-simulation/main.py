import asyncio
import json
import logging
import time
import aiomqtt

from common import math_utils, physics, roundabout
from common.const import AREA_RADIUS, CAR_LENGTH, TIMER_NETWORK_TIMEOUT, UPDATES_P_S_CONTROLLER, ROUNDABOUT_POS, VEHICLE_DIST_TOL
from common.get_env import config
from common.models.vehicle import Vehicle, VehicleCollision, VehicleNavState, VehiclePosition



logger = logging.getLogger(__name__)

# v_id -> (VehiclePosition, timestamp)
vehicles_state:		dict[str, Vehicle] = {}
vehicle_timestamps: dict[str, float] = {}


async def loop_listen_positions(client: aiomqtt.Client):
	"""
	Listen for cars reporting their positions.
	"""
	topic_pattern = f"{config.TOPIC_VEHICLE_PREFIX}/+/{config.TOPIC_VEHICLE_TELEMETRY_SUFFIX}"
	await client.subscribe(topic_pattern)
	logger.info("Controller listening for positions.")
	
	async for message in client.messages:
		logger.debug("Received on topic %s: %s", message.topic, message.payload)
		try:
			payload 				= json.loads(message.payload)
			vehicle_state			= Vehicle(**payload)
			vehicle_timestamps[vehicle_state.id]	= time.time()
			vehicles_state[vehicle_state.id]		= vehicle_state
		except Exception as e:
			logger.error("Failed to parse position: %s", e)


async def loop_publish_collisions(client: aiomqtt.Client):
	"""
	Publish detected collisions between vehicles.
	"""
	collisions_topic = f"{config.TOPIC_VEHICLE_PREFIX}/{config.TOPIC_VEHICLE_COLLISIONS_SUFFIX}"
	while True:
		current_time = time.time()
		collisions = []

		for v1_id, v1 in vehicles_state.items():
			v1_t = vehicle_timestamps[v1_id]
			if current_time - v1_t > TIMER_NETWORK_TIMEOUT:
				continue

			v1_pos				= v1.to_pos()
			v1_dist_to_center	= math_utils.get_dist(v1_pos.pos, ROUNDABOUT_POS)
			if v1_dist_to_center > AREA_RADIUS / 2:
				# Only check for collisions if the vehicle is close enough to the roundabout.
				# Otherwise, there would be collisions for spawns.
				continue
			
			for v2_id, v2 in vehicles_state.items():
				v2_t = vehicle_timestamps[v2_id]
				if v2_t and current_time - v2_t > TIMER_NETWORK_TIMEOUT:
					continue
				if v2_t < v1_t:		# type: ignore
					# only check a pair once
					continue
				if v1_id == v2_id:
					continue
				
				v2_pos				= v2.to_pos()
				v2_dist_to_center	= math_utils.get_dist(v2_pos.pos, ROUNDABOUT_POS)
				if v2_dist_to_center > AREA_RADIUS / 2:
					continue

				# ignore collisions when one is exiting and the other entering,
				# since it's a simulation representation's problem
				v1_exit_angle = roundabout.get_road_angle(v1.exit_road)
				v2_exit_angle = roundabout.get_road_angle(v2.exit_road)
				v1_close_to_exit = math_utils.get_dist_on_circle(v1.pos_angle, v1_exit_angle) <= CAR_LENGTH / 2
				v2_close_to_exit = math_utils.get_dist_on_circle(v2.pos_angle, v2_exit_angle) <= CAR_LENGTH / 2
				v1_just_entered = v1.angle_traveled <= CAR_LENGTH / 2
				v2_just_entered = v2.angle_traveled <= CAR_LENGTH / 2
				if (
					v1.nav_state == VehicleNavState.IN_ROUNDABOUT
					and v2.nav_state == VehicleNavState.IN_ROUNDABOUT and (
						(v1_close_to_exit and v2_just_entered) or
						(v2_close_to_exit and v1_just_entered)
				)):
					continue

				if physics.vehicle_collide(v1_pos, v2_pos):
					collisions.append(VehicleCollision(
						v1_id=v1_id,
						v2_id=v2_id,
						timestamp=current_time
					).model_dump())
		
		if collisions:
			for collision in collisions:
				await client.publish(collisions_topic, payload=json.dumps(collision))
				logger.warning("Published collision: %s", collision)
		
		await asyncio.sleep(1.0 / UPDATES_P_S_CONTROLLER)


async def main():
	async with aiomqtt.Client(hostname=config.HOST_BROKER, port=config.PORT_BROKER) as client:
		await asyncio.gather(
			loop_listen_positions(client),
			loop_publish_collisions(client),
		)



if __name__ == "__main__":
	asyncio.run(main())