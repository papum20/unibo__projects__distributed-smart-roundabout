import asyncio
import json
import logging
import time
import aiomqtt

from common import math_utils, physics
from common.const import AREA_RADIUS, TIMER_NETWORK_TIMEOUT, UPDATES_P_S_CONTROLLER, ROUNDABOUT_POS
from common.get_env import config
from common.models.vehicle import VehicleCollision, VehiclePosition



logger = logging.getLogger(__name__)

# v_id -> (VehiclePosition, timestamp)
vehicles_pos			: dict[str, VehiclePosition] = {}



async def loop_listen_positions(client: aiomqtt.Client):
	"""
	Listen for cars reporting their positions.
	"""
	topic_pattern = f"{config.TOPIC_VEHICLE_PREFIX}/+/{config.TOPIC_VEHICLE_POSITION_SUFFIX}"
	await client.subscribe(topic_pattern)
	logger.info("Controller listening for positions.")
	
	async for message in client.messages:
		logger.debug("Received on topic %s: %s", message.topic, message.payload)
		try:
			payload 				= json.loads(message.payload)
			v_pos					= VehiclePosition(**payload)
			v_pos.timestamp			= time.time()
			vehicles_pos[v_pos.id]	= v_pos
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

		for v1_id, v1_pos in vehicles_pos.items():
			v1_t = v1_pos.timestamp
			if v1_t and current_time - v1_t > TIMER_NETWORK_TIMEOUT:
				continue
			v1_dist_to_center = math_utils.get_dist(v1_pos.pos, ROUNDABOUT_POS)
			if v1_dist_to_center > AREA_RADIUS / 2:
				# Only check for collisions if the vehicle is close enough to the roundabout.
				# Otherwise, there would be collisions for spawns.
				continue
			
			for v2_id, v2_pos in vehicles_pos.items():
				v2_t = v2_pos.timestamp
				if v2_t and current_time - v2_t > TIMER_NETWORK_TIMEOUT:
					continue
				if v2_t < v1_t:		# type: ignore
					# only check a pair once
					continue
				if v1_id == v2_id:
					continue
				v2_dist_to_center = math_utils.get_dist(v2_pos.pos, ROUNDABOUT_POS)
				if v2_dist_to_center > AREA_RADIUS / 2:
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