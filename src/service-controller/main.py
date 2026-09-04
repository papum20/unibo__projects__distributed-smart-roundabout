import asyncio
import json
import logging
import time
import aiomqtt

from common import math_utils, physics, roundabout, vehicle
from common.const import (
	ROUNDABOUT_N_ROADS, TIMER_NETWORK_TIMEOUT, UPDATES_P_S_CONTROLLER, ROAD_WIDTH, ROUNDABOUT_POS, ROUNDABOUT_PROXIMITY_DIST, ROUNDABOUT_RADIUS, CAR_LENGTH, VEHICLE_SAFETY_MARGIN_M
)
from common.get_env import config
from common.models.models import Command
from common.models.vehicle import (
	Vehicle, VehicleNavState, VehiclePosition, VehicleState
)



logger = logging.getLogger(__name__)

active_vehicles			: dict[str, Vehicle]	= {}
# time of last update received for each
active_vehicles_times	: dict[str, float]		= {}
active_vehicles_done	: dict[str, bool]		= {}

# crowdsensed vehicles, thorugh each one's local vision
# v_id -> (VehiclePosition, timestamp)
vehicles_pos			: list[VehiclePosition] = []

# Vehicle ids, in order of arrival to the roundabout.
# Keep them inside until they exit, because they may still have to yield to others.
precedence_queue		: list[str] = []

UPDATES_BEFORE_EXPIRY		= 3
VISION_MATCH_TOLERANCE_M	= 5.0




def should_yield_to(v1: Vehicle, v2: Vehicle) -> bool:
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



def evaluate(vehicles: list[Vehicle], conflict_time_margin_s: float = 2.0) -> dict[str, Command]:
	"""
	@param vehicles: a list of all current vehicles.
	@return a dictionary mapping vehicle_id -> Command.
	"""
	vehicles_pos	= [v.to_pos() for v in vehicles]
	# Default: tell everyone to maintain speed.
	# Each update can only reduce it.
	commands 		= {v.id: Command(target_acceleration=v.params.max_accel) for v in vehicles}

	for v1 in vehicles:
		if v1.nav_state == VehicleNavState.EXITING:
			try:
				precedence_queue.remove(v1.id)
			except ValueError:
				# v1.id is not in the queue
				pass
		if v1.nav_state == VehicleNavState.APPROACHING and v1.id not in precedence_queue:
			dist_to_roundabout = math_utils.get_dist(v1.pos, ROUNDABOUT_POS) - ROUNDABOUT_RADIUS - ROAD_WIDTH
			if dist_to_roundabout <= ROUNDABOUT_PROXIMITY_DIST:
				precedence_queue.append(v1.id)
		
		
	for v1 in vehicles:

		# step 1: safety checks

		commands[v1.id].target_acceleration = vehicle.evaluate_safely(v1, vehicles_pos).target_acceleration
		
		for v2 in vehicles:
			if v1.id == v2.id: continue

			# inside-approaching conflict: slow down to yield
			elif v1.nav_state == VehicleNavState.IN_ROUNDABOUT and v2.nav_state == VehicleNavState.APPROACHING:
				
				if should_yield_to(v1, v2) or v2.state == VehicleState.FAILSAFE:
					conflict_angle		= roundabout.get_road_angle(v2.entry_road)
					v1_dist_to_conflict	= math_utils.get_dist_on_circle(v1.pos_angle, conflict_angle)
					v1_exit_angle		= roundabout.get_road_angle(v1.exit_road)
					v1_dist_to_exit		= math_utils.get_dist_on_circle(v1.pos_angle, v1_exit_angle)

					# check if v1 exits earlier
					if v1_dist_to_exit <= v1_dist_to_conflict:
						continue
					if v2.state == VehicleState.FAILSAFE:
						# v2 is out of control, so yield.
						# However, we consider its exit data reliable.
						commands[v1.id].target_acceleration = min(commands[v1.id].target_acceleration, -2.0)
						continue

					# slow down until v2 is able to enter, each maintaining its speed.
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
							commands[v1.id].target_acceleration = min(commands[v1.id].target_acceleration, 0.0)
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
				if v2.state == VehicleState.FAILSAFE:
					# v2 is out of control, so yield.
					# However, we consider its exit data reliable.
					commands[v1.id].target_acceleration = min(commands[v1.id].target_acceleration, -2.0)
					continue

				# if v2 has to yield, v1 can just accelerate
				# pylint: disable-next=arguments-out-of-order
				if should_yield_to(v2, v1):
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
						commands[v1.id].target_acceleration = min(commands[v1.id].target_acceleration, 0.0)
					else:
						# a light brake is often enough
						commands[v1.id].target_acceleration = min(commands[v1.id].target_acceleration, -2.0)

				if commands[v1.id].target_acceleration == -v1.get_acc_brake():
					break


		# step 2: optimizing, with max acc from prev step

		for v2 in vehicles:
			if v1.id == v2.id: continue

			# inside-approaching conflict: slow down to yield
			if v1.nav_state == VehicleNavState.IN_ROUNDABOUT and v2.nav_state == VehicleNavState.APPROACHING:
				
				if should_yield_to(v1, v2) or v2.state != VehicleState.NORMAL:
					new_acc				= commands[v1.id].target_acceleration

					conflict_angle		= roundabout.get_road_angle(v2.entry_road)
					v1_dist_to_conflict	= math_utils.get_dist_on_circle(v1.pos_angle, conflict_angle)
					v1_exit_angle		= roundabout.get_road_angle(v1.exit_road)
					v1_dist_to_exit		= math_utils.get_dist_on_circle(v1.pos_angle, v1_exit_angle)

					# check if v1 exits earlier
					if v1_dist_to_exit <= v1_dist_to_conflict:
						continue
					if v2.state != VehicleState.NORMAL:
						# v2 is out of control: in failsafe mode, it will behave cautiously.
						# However, to avoid starvation for v2 and who's behind it, yield if possible.
						# If it's in failsafe, its exit data is reliable, otherwise
						# it's just estimated as a worst-case scenario, so that it won't influence our decisions.
						v1_stop_dist = v1.get_stop_dist(acc_brake=v1.get_acc_brake())
						if v1_stop_dist >= v1_dist_to_conflict + VEHICLE_SAFETY_MARGIN_M:
							commands[v1.id].target_acceleration = min(new_acc, v1.get_acc_brake())
						continue

					# coordinate with v2 (which has priority, in case), looking for the fastest option

					if physics.vehicle_enters_later(		v2, v1, v2_acc=new_acc, v1_acc=v2.acceleration):
						continue
					if physics.vehicle_can_enter_safely(	v2, v1, v2_acc=new_acc, v1_acc=v2.acceleration):
						continue
					if new_acc > 0						and physics.vehicle_can_enter_safely( v2, v1, v2_acc=0.0,					v1_acc=v2.acceleration):
						commands[v1.id].target_acceleration = 0.0
						continue
					if new_acc > v1.get_acc_brake()		and physics.vehicle_can_enter_safely( v2, v1, v2_acc=v1.get_acc_brake(),	v1_acc=v2.acceleration):
						commands[v1.id].target_acceleration = v1.get_acc_brake()
						continue
					if new_acc > v1.params.max_brake	and physics.vehicle_can_enter_safely( v2, v1, v2_acc=v1.params.max_brake, v1_acc=v2.acceleration):
						commands[v1.id].target_acceleration = v1.params.max_brake
						continue

			# inside-approaching, conflict
			elif v1.nav_state == VehicleNavState.APPROACHING and v2.nav_state == VehicleNavState.IN_ROUNDABOUT:
				new_acc				= commands[v1.id].target_acceleration

				conflict_angle		= roundabout.get_road_angle(v1.entry_road)
				v2_dist_to_conflict	= math_utils.get_dist_on_circle(v2.pos_angle, conflict_angle)
				v2_exit_angle		= roundabout.get_road_angle(v2.exit_road)
				v2_dist_to_exit		= math_utils.get_dist_on_circle(v2.pos_angle, v2_exit_angle)
				
				# check if v2 exits earlier
				if v2_dist_to_exit <= v2_dist_to_conflict:
					continue

				# if v2 has to yield, v1 can just accelerate.
				# if v2 in failsafe, it will have priority, but maybe v1 can pass without interfering
				# pylint: disable-next=arguments-out-of-order
				if v2.state == VehicleState.NORMAL and should_yield_to(v2, v1):
					continue

				# coordinate with v2 (which has priority, in case)

				if physics.vehicle_enters_later(		v1, v2, v1_acc=new_acc, v2_acc=v2.acceleration):
					continue
				if physics.vehicle_can_enter_safely(	v1, v2, v1_acc=new_acc, v2_acc=v2.acceleration):
					continue
				if new_acc > 0						and physics.vehicle_can_enter_safely( v2, v1, v1_acc=0.0,					v2_acc=v2.acceleration):
					commands[v1.id].target_acceleration = 0.0
					continue
				if new_acc > v1.get_acc_brake()		and physics.vehicle_can_enter_safely( v2, v1, v1_acc=v1.get_acc_brake(),	v2_acc=v2.acceleration):
					commands[v1.id].target_acceleration = v1.get_acc_brake()
					continue
				if new_acc > v1.params.max_brake	and physics.vehicle_can_enter_safely( v2, v1, v1_acc=v1.params.max_brake,	v2_acc=v2.acceleration):
					commands[v1.id].target_acceleration = v1.params.max_brake
					continue
			
	return commands



async def loop_listen_telemetry(client: aiomqtt.Client):
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
			active_vehicles_times[v.id] = time.time()
			active_vehicles_done[v.id]	= False
		except Exception as e:
			logger.error("Failed to parse telemetry: %s", e)


async def loop_listen_positions(client: aiomqtt.Client):
	"""
	Listen for cars' crowdsensing of nearby vehicles.
	"""
	topic_pattern = f"{config.TOPIC_VEHICLE_PREFIX}/+/{config.TOPIC_VEHICLE_VISION_SUFFIX}"
	await client.subscribe(topic_pattern)
	logger.info("Controller listening for positions.")
	
	async for message in client.messages:
		logger.debug("Received on topic %s: %s", message.topic, message.payload)
		try:
			payload 			= json.loads(message.payload)
			v_pos				= VehiclePosition(**payload)
			if v_pos.timestamp is None:
				v_pos.timestamp	= time.time()
			vehicles_pos.append(v_pos)
		except Exception as e:
			logger.error("Failed to parse position: %s", e)


async def loop_publish_state(client: aiomqtt.Client):
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


async def loop_control(client: aiomqtt.Client):
	"""
	Continuously evaluate traffic and publish commands.
	"""
	logger.info("Controller Orchestration Loop started.")
	ghost_counter	= 0

	while True:
		current_t	= time.time()

		# clear old vision data
		while True:
			if len(vehicles_pos) == 0:
				break
			v_t = vehicles_pos[0].timestamp
			if v_t and current_t - v_t > TIMER_NETWORK_TIMEOUT:
				vehicles_pos.pop(0)
			else:
				break

		# Predict current positions.
		# can't receive messages from disconnected vehicles (they're only sent for debugging).
		v_list			= []
		for v in active_vehicles.values():
			if v.state	!= VehicleState.DISCONNECTED:
				v_t		= active_vehicles_times.get(v.id, current_t)
				v_list.append(
					vehicle.vehicle_navigate(dt=current_t - v_t, v=v.model_copy(deep=True))
				)
		# try to merge vision data
		for v_pos in vehicles_pos:
			is_known = False
			
			# compare with known ones
			for known_v in v_list:
				dist = math_utils.get_dist(v_pos.pos, known_v.pos)
				if dist < VISION_MATCH_TOLERANCE_M:
					is_known = True
					break
					
			if not is_known:
				ghost_counter += 1
				ghost_id = f"GHOST-{ghost_counter}"
				
				ghost_v = Vehicle(
					id			= ghost_id,
					pos			= v_pos.pos,
					pos_angle	= v_pos.pos_angle,
					speed		= v_pos.speed,
					state		= VehicleState.DISCONNECTED,
					nav_state	= v_pos.nav_state,
					entry_road	= vehicle.get_predicted_entry(v_pos),
					exit_road	= vehicle.get_predicted_exit(v_pos)
				)

				v_pos_t = v_pos.timestamp if v_pos.timestamp is not None else current_t
				ghost_v = vehicle.vehicle_navigate(dt=current_t - v_pos_t, v=ghost_v)
				
				# if a ghost in a similar position is found, it won't be added because it will match with this
				v_list.append(ghost_v)
				logger.debug("Detected DISCONNECTED vehicle %s: %s", ghost_id, ghost_v)

		commands	= evaluate(v_list)
		current_t	= time.time()
		
		for v_n, (vid, cmd) in enumerate(commands.items(), start=1):
			if vid.startswith("GHOST"):
				continue
	
			if active_vehicles_done.get(vid, False) and active_vehicles_times.get(vid, 0) < current_t - UPDATES_BEFORE_EXPIRY / UPDATES_P_S_CONTROLLER:
				logger.debug("Vehicle #%d [%s]: no recent telemetry. Skipping command.", v_n, vid[:4])
				continue

			if cmd.target_acceleration < 0.0:
				logger.debug("Vehicle #%d [%s]: brake (ACC: %s)", v_n, vid[:4], cmd.target_acceleration)
			else:
				logger.debug("Vehicle #%d [%s]: maintaining speed (ACC: %s)", v_n, vid[:4], cmd.target_acceleration)
				
			topic = f"{config.TOPIC_VEHICLE_PREFIX}/{vid}/{config.TOPIC_VEHICLE_COMMAND_SUFFIX}"
			await client.publish(topic, payload=cmd.model_dump_json())
			
		await asyncio.sleep(1.0 / UPDATES_P_S_CONTROLLER)


async def main():
	async with aiomqtt.Client(hostname=config.HOST_BROKER, port=config.PORT_BROKER) as client:
		await asyncio.gather(
			loop_listen_telemetry(client),
			loop_listen_positions(client),
			loop_publish_state(client),
			loop_control(client)
		)



if __name__ == "__main__":
	asyncio.run(main())