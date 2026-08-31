import asyncio
import json
import logging
import math
import aiomqtt

from common import math_utils, roundabout
from common.const import FRAMERATE, ROUNDABOUT_POS, ROUNDABOUT_RADIUS, CAR_LENGTH
from common.get_env import config
from common.models.models import Command, Vehicle, VehicleNavState



logger = logging.getLogger(__name__)

active_vehicles: dict[str, Vehicle] = {}



def evaluate_traffic(vehicles: list[Vehicle], conflict_time_margin_s: float = 2.0) -> dict[str, Command]:
	"""
	Take a list of all current vehicles.
	@return a dictionary mapping vehicle_id -> Command.
	"""
	# default: tell everyone to maintain speed
	commands = {v.id: Command(target_acceleration=0.0) for v in vehicles}
	
	for v1 in vehicles:
		for v2 in vehicles:
			if v1.id == v2.id: continue

			# dynamic safety distance based on speed
			safe_dist = (v1.speed * 1.0) + CAR_LENGTH + 3.0
			
			# approaching, tailgating
			if v1.nav_state == VehicleNavState.APPROACHING and v2.nav_state == VehicleNavState.APPROACHING:
				if v1.entry_road == v2.entry_road:
					dist_v1 = math_utils.get_dist(v1.pos, ROUNDABOUT_POS)
					dist_v2 = math_utils.get_dist(v2.pos, ROUNDABOUT_POS)
					
					# v1 too close behind v2
					if dist_v1 > dist_v2 and (dist_v1 - dist_v2) < safe_dist:
						commands[v1.id].target_acceleration = -v1.params.max_brake


			# inside, tailgating
			elif v1.nav_state == VehicleNavState.IN_ROUNDABOUT and v2.nav_state == VehicleNavState.IN_ROUNDABOUT:
				angle_diff	= (v2.pos_angle - v1.pos_angle) % (2 * math.pi)
				arc_dist	= angle_diff * ROUNDABOUT_RADIUS
				
				# v1 too close behind v2
				if 0 < arc_dist < safe_dist:
					commands[v1.id].target_acceleration = -v1.params.max_brake


			# inside-approaching, conflict
			elif v1.nav_state == VehicleNavState.APPROACHING and v2.nav_state == VehicleNavState.IN_ROUNDABOUT:
				# where v1 will enter the roundabout
				conflict_angle = roundabout.get_road_angle(v1.entry_road)
				
				# how long until v2 arrives
				v2_angle_to_conflict	= (conflict_angle - v2.pos_angle) % (2 * math.pi)
				v2_dist_to_conflict		= v2_angle_to_conflict * ROUNDABOUT_RADIUS
				
				# check if v2 exits earlier
				v2_exit_angle		= roundabout.get_road_angle(v2.exit_road)
				v2_angle_to_exit	= (v2_exit_angle - v2.pos_angle) % (2 * math.pi)
				v2_dist_to_exit		= v2_angle_to_exit * ROUNDABOUT_RADIUS
				
				if v2_dist_to_exit < v2_dist_to_conflict:
					continue
				
				# Time-To-Arrival (TTA)
				v2_speed = max(v2.speed, 0.1)	# prevent div by zero
				v1_speed = max(v1.speed, 0.1)
				
				tta_other = v2_dist_to_conflict / v2_speed
				
				v1_dist_to_conflict	= math_utils.get_dist(v1.pos, ROUNDABOUT_POS) - ROUNDABOUT_RADIUS
				v1_tta				= v1_dist_to_conflict / v1_speed
				
				# If v1 will arrive around the same time as v2 passes by,
				# slow down instead of having to stop
				if abs(v1_tta - tta_other) < conflict_time_margin_s:
					# a light brake is often enough
					commands[v1.id].target_acceleration = -2.0
			
	return commands



async def listen_telemetry(client: aiomqtt.Client):
	"""
	Listen for cars reporting their positions.
	"""
	topic_pattern = f"{config.TOPIC_VEHICLE_PREFIX}/+/{config.TOPIC_VEHICLE_TELEMETRY_SUFFIX}"
	await client.subscribe(topic_pattern)
	logger.info("Controller listening for telemetry.")
	
	async for message in client.messages:
		logger.debug("Received on topic %s: %s", message.topic, message.payload)
		try:
			payload = json.loads(message.payload)
			v = Vehicle(**payload)
			active_vehicles[v.id] = v
		except Exception as e:
			logger.error("Failed to parse telemetry: %s", e)


async def control_loop(client: aiomqtt.Client):
	"""
	Continuously evaluate traffic and publish commands.
	"""
	logger.info("Controller Orchestration Loop started.")
	while True:
		vehicles_list	= list(active_vehicles.values())
		commands		= evaluate_traffic(vehicles_list)
		
		for vid, cmd in commands.items():
			if cmd.target_acceleration < 0.0:
				logger.info("Orchestrator commanding [%s] to yield/brake (ACC: %s)", vid[:4], cmd.target_acceleration)
			else:
				logger.debug("Orchestrator commanding [%s] to maintain speed (ACC: %s)", vid[:4], cmd.target_acceleration)
				
			topic = f"{config.TOPIC_VEHICLE_PREFIX}/{vid}/{config.TOPIC_VEHICLE_COMMAND_SUFFIX}"
			await client.publish(topic, payload=cmd.model_dump_json())
			
		await asyncio.sleep(1.0 / FRAMERATE)


async def main():
	async with aiomqtt.Client(hostname=config.HOST_BROKER, port=config.PORT_BROKER) as client:
		await asyncio.gather(
			listen_telemetry(client),
			control_loop(client)
		)



if __name__ == "__main__":
	asyncio.run(main())