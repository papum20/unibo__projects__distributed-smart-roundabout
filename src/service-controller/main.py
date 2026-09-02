import asyncio
import json
import logging
import math
from time import time
import aiomqtt

from common import math_utils, physics, roundabout
from common.const import ROUNDABOUT_N_ROADS, UPDATES_P_S_CONTROLLER, ROAD_WIDTH, ROUNDABOUT_POS, ROUNDABOUT_RADIUS, CAR_LENGTH
from common.get_env import config
from common.models.models import Command, Vehicle, VehicleNavState



logger = logging.getLogger(__name__)

active_vehicles			: dict[str, Vehicle]	= {}
# time of last update received for each
active_vehicles_times	: dict[str, float]		= {}
active_vehicles_done	: dict[str, bool]		= {}

# Vehicle ids, in order of arrival to the roundabout.
# Keep them inside until they exit, because they may still have to yield to others.
precedence_queue	: list[str] = []

UPDATES_BEFORE_EXPIRY		= 3
ROUNDABOUT_RPOXIMITY_DIST	= 2 * CAR_LENGTH



def require_precedence(v1: Vehicle, v2: Vehicle) -> bool:
	"""
	Determine if v1 should yield to v2, based on the precedence queue
	and on giving priority to the right.
	@return True if v1 should yield to v2, False otherwise.
	"""
	if (v2.entry_road - v1.entry_road) % ROUNDABOUT_N_ROADS >= ROUNDABOUT_N_ROADS / 2:
		return False
	for v in precedence_queue:
		if v == v1.id:
			return False
		if v == v2.id:
			return True
	return False

def safety_distance(v: Vehicle, margin: float) -> float:
	"""
	Calculate a dynamic safety distance based on the vehicle's speed.
	@param margin: additional safety margin (e.g. half a car length, if calculating it from the car on front)
	"""
	# dynamic safety distance based on speed (1s reaction time)
	safe_dist = (v.speed * 1.0) + CAR_LENGTH / 2.0 + margin
	return max(safe_dist, 2 * CAR_LENGTH)



def evaluate_traffic(vehicles: list[Vehicle], conflict_time_margin_s: float = 2.0) -> dict[str, Command]:
	"""
	Take a list of all current vehicles.
	@return a dictionary mapping vehicle_id -> Command.
	"""
	# Default: tell everyone to maintain speed.
	# Each update can only reduce it.
	commands = {v.id: Command(target_acceleration=v.params.max_accel) for v in vehicles}

	for v1 in vehicles:
		if v1.nav_state == VehicleNavState.EXITING:
			try:
				precedence_queue.remove(v1.id)
			except ValueError:
				# v1.id is not in the queue
				pass
		if v1.nav_state == VehicleNavState.APPROACHING and v1.id not in precedence_queue:
			dist_to_roundabout = math_utils.get_dist(v1.pos, ROUNDABOUT_POS) - ROUNDABOUT_RADIUS - ROAD_WIDTH
			if dist_to_roundabout <= ROUNDABOUT_RPOXIMITY_DIST:
				precedence_queue.append(v1.id)
		
		
	for v1 in vehicles:
		for v2 in vehicles:
			if v1.id == v2.id: continue

			# approaching, tailgating
			if v1.nav_state == VehicleNavState.APPROACHING and v2.nav_state == VehicleNavState.APPROACHING:
				if v1.entry_road == v2.entry_road:
					dist_v1 = math_utils.get_dist(v1.pos, ROUNDABOUT_POS)
					dist_v2 = math_utils.get_dist(v2.pos, ROUNDABOUT_POS)
					
					# v1 too close behind v2
					if dist_v1 > dist_v2 and (dist_v1 - dist_v2) < safety_distance(v1, margin=CAR_LENGTH/2.0):
						commands[v1.id].target_acceleration = -v1.params.max_brake


			# inside, tailgating
			elif v1.nav_state == VehicleNavState.IN_ROUNDABOUT and v2.nav_state == VehicleNavState.IN_ROUNDABOUT:
				angle_diff	= (v2.pos_angle - v1.pos_angle) % (2 * math.pi)
				arc_dist	= angle_diff * ROUNDABOUT_RADIUS
				
				# v1 too close behind v2
				if 0 < arc_dist < safety_distance(v1, margin=CAR_LENGTH/2.0):
					commands[v1.id].target_acceleration = -v1.params.max_brake


			# inside-approaching conflict: slow down to yield
			elif v1.nav_state == VehicleNavState.IN_ROUNDABOUT and v2.nav_state == VehicleNavState.APPROACHING:
				if require_precedence(v1, v2):
					# slow down until v2 is able to enter, each maintaining its speed
					conflict_angle		= roundabout.get_road_angle(v2.entry_road)
					v1_dist_to_conflict	= math_utils.get_dist_on_circle(v1.pos_angle, conflict_angle)
					v1_exit_angle		= roundabout.get_road_angle(v1.exit_road)
					v1_dist_to_exit		= math_utils.get_dist_on_circle(v1.pos_angle, v1_exit_angle)

					# check if v1 exits earlier
					if v1_dist_to_exit <= v1_dist_to_conflict:
						continue

					# time to arrival (TTA)
					v1_tta				= physics.vehicle_tta(
						v1, v1_dist_to_conflict,
						new_acc=0.0, margin=CAR_LENGTH
					)
					v1_tta_acc			= physics.vehicle_tta(
						v1, v1_dist_to_conflict,
						new_acc=commands[v1.id].target_acceleration, margin=CAR_LENGTH
					)
					v2_dist_to_conflict	= math_utils.get_dist(v2.pos, ROUNDABOUT_POS) - ROUNDABOUT_RADIUS
					v2_tta				= physics.vehicle_tta(v2, v2_dist_to_conflict, margin=CAR_LENGTH)

					if abs(v1_tta_acc - v2_tta) < conflict_time_margin_s or commands[v1.id].target_acceleration <= 0.0:
						if abs(v1_tta - v2_tta) > conflict_time_margin_s and commands[v1.id].target_acceleration >= 0.0:
							commands[v1.id].target_acceleration = 0.0
						else:
							# a light brake is often enough
							commands[v1.id].target_acceleration = min(commands[v1.id].target_acceleration, -2.0)


			# inside-approaching, conflict
			elif v1.nav_state == VehicleNavState.APPROACHING and v2.nav_state == VehicleNavState.IN_ROUNDABOUT:
				conflict_angle		= roundabout.get_road_angle(v1.entry_road)
				v2_dist_to_conflict	= math_utils.get_dist_on_circle(v2.pos_angle, conflict_angle)
				v2_exit_angle		= roundabout.get_road_angle(v2.exit_road)
				v2_dist_to_exit		= math_utils.get_dist_on_circle(v2.pos_angle, v2_exit_angle)
				
				# check if v2 exits earlier
				if v2_dist_to_exit <= v2_dist_to_conflict:
					continue

				# if v2 has to yield, v1 can just accelerate
				# pylint: disable-next=arguments-out-of-order
				if require_precedence(v2, v1):
					continue
				
				# time to arrival (TTA)
				v2_tta				= physics.vehicle_tta(v2, v2_dist_to_conflict, margin=CAR_LENGTH)
				v1_dist_to_conflict	= math_utils.get_dist(v1.pos, ROUNDABOUT_POS) - ROUNDABOUT_RADIUS
				v1_tta				= physics.vehicle_tta(
					v1, v1_dist_to_conflict,
					new_acc=0.0, margin=CAR_LENGTH
				)
				v1_tta_acc			= physics.vehicle_tta(
					v1, v1_dist_to_conflict,
					new_acc=commands[v1.id].target_acceleration, margin=CAR_LENGTH
				)
				
				# If v1 will arrive around the same time as v2 passes by,
				# slow down, trying to avoid to stop.
				# If possible, prefer the fastest option.
				if abs(v1_tta_acc - v2_tta) < conflict_time_margin_s or commands[v1.id].target_acceleration <= 0.0:
					if abs(v1_tta - v2_tta) > conflict_time_margin_s and commands[v1.id].target_acceleration >= 0.0:
						commands[v1.id].target_acceleration = 0.0
					else:
						# a light brake is often enough
						commands[v1.id].target_acceleration = min(commands[v1.id].target_acceleration, -2.0)
			
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
			active_vehicles[v.id]		= v
			active_vehicles_times[v.id] = time()
			active_vehicles_done[v.id]	= False
		except Exception as e:
			logger.error("Failed to parse telemetry: %s", e)


async def publish_state(client: aiomqtt.Client):
	"""
	Publish the current state of the controller, including the precedence queue.
	"""
	state_topic = config.TOPIC_CONTROLLER_STATUS
	while True:
		state_payload = {
			"precedence_queue": precedence_queue
		}
		await client.publish(state_topic, payload=json.dumps(state_payload))
		await asyncio.sleep(1.0 / UPDATES_P_S_CONTROLLER)


async def control_loop(client: aiomqtt.Client):
	"""
	Continuously evaluate traffic and publish commands.
	"""
	logger.info("Controller Orchestration Loop started.")
	while True:
		vehicles_list	= list(active_vehicles.values())
		commands		= evaluate_traffic(vehicles_list)
		current_time	= time()
		
		for v_n, (vid, cmd) in enumerate(commands.items(), start=1):
			if active_vehicles_done.get(vid, False) and active_vehicles_times.get(vid, 0) < current_time - UPDATES_BEFORE_EXPIRY / UPDATES_P_S_CONTROLLER:
				logger.debug("Vehicle #%d [%s]: no recent telemetry. Skipping command.", v_n, vid[:4])
				continue

			if cmd.target_acceleration < 0.0:
				logger.info("Vehicle #%d [%s]: brake (ACC: %s)", v_n, vid[:4], cmd.target_acceleration)
			else:
				logger.debug("Vehicle #%d [%s]: maintaining speed (ACC: %s)", v_n, vid[:4], cmd.target_acceleration)
				
			topic = f"{config.TOPIC_VEHICLE_PREFIX}/{vid}/{config.TOPIC_VEHICLE_COMMAND_SUFFIX}"
			await client.publish(topic, payload=cmd.model_dump_json())
			
		await asyncio.sleep(1.0 / UPDATES_P_S_CONTROLLER)


async def main():
	async with aiomqtt.Client(hostname=config.HOST_BROKER, port=config.PORT_BROKER) as client:
		await asyncio.gather(
			listen_telemetry(client),
			publish_state(client),
			control_loop(client)
		)



if __name__ == "__main__":
	asyncio.run(main())