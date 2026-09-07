import asyncio
import json
import logging
import time
import aiomqtt

from common import math_utils, physics, roundabout, vehicle
from common.const import (
	CAR_LENGTH, ROUNDABOUT_N_ROADS, TIMER_NETWORK_TIMEOUT, UPDATES_P_S_CONTROLLER, ROAD_WIDTH, ROUNDABOUT_POS, ROUNDABOUT_PROXIMITY_DIST, ROUNDABOUT_RADIUS, VEHICLE_SAFETY_MARGIN_M
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

# kept to share it with the webviewer
ghosts					: dict[str, Vehicle] = {}

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
		# don't add disconnected vehicles: will give them priority anyway (if possible) - otherwise they will fill the queue
		if v1.state == VehicleState.NORMAL and v1.nav_state == VehicleNavState.APPROACHING and v1.id not in precedence_queue:
			dist_to_roundabout = math_utils.get_dist(v1.pos, ROUNDABOUT_POS) - ROUNDABOUT_RADIUS - ROAD_WIDTH
			if dist_to_roundabout <= ROUNDABOUT_PROXIMITY_DIST:
				precedence_queue.append(v1.id)
		
		
	for v1 in vehicles:

		# step 1: safety checks

		commands[v1.id].target_acceleration = vehicle.evaluate_safely(v1, vehicles_pos).target_acceleration
		
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
							commands[v1.id].target_acceleration = min(new_acc, -v1.get_acc_brake())
						continue

					# coordinate with v2 (which has priority, in case), looking for the fastest option

					if physics.vehicle_enters_later( v2, v1, v1_acc=v2.acceleration, v2_acc=new_acc ):
						continue
					v1_acc_choices = (new_acc, 0.0, -v1.get_acc_brake(), -v1.params.max_brake)
					for v1_acc in v1_acc_choices:
						if new_acc >= v1_acc and physics.vehicle_can_enter_safely( v2, v1, v1_acc=v2.acceleration, v2_acc=v1_acc ):
							commands[v1.id].target_acceleration = v1_acc
							break
					# if can't stop safely, just pass: the approaching v2,
					# either guided by controller or failsafe mode, will brake

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

				if physics.vehicle_enters_later( v1, v2, v1_acc=new_acc, v2_acc=v2.acceleration ):
					continue
				if physics.vehicle_can_enter_safely(v1, v2, v1_acc=new_acc, v2_acc=v2.acceleration):
					# if possible, go faster
					continue
				# if still far from roundabout, no need to brake
				v1_dist_to_entry = math_utils.get_dist(v1.pos, ROUNDABOUT_POS) - ROUNDABOUT_RADIUS - ROAD_WIDTH - CAR_LENGTH / 2
				if v1.get_stop_dist(acc_brake=v1.get_acc_brake()) + v1.get_reaction_time_dist() - v1_dist_to_entry < ROUNDABOUT_PROXIMITY_DIST:
					# as long as you can start braking later, no need to already do it now
					continue
				v1_acc_choices = (0.0, -v1.get_acc_brake(), -v1.params.max_brake)
				for v1_acc in v1_acc_choices:
					if new_acc > v1_acc and physics.vehicle_can_enter_safely( v1, v2, v1_acc=v1_acc, v2_acc=v2.acceleration ):
						commands[v1.id].target_acceleration = v1_acc
						break
				else:
					# no safe option: wait for v2 to pass
					commands[v1.id].target_acceleration = -v1.params.max_brake
			
	return commands



async def loop_listen(client):
	telemetry_topic = (f"{config.TOPIC_VEHICLE_PREFIX}/+/{config.TOPIC_VEHICLE_TELEMETRY_SUFFIX}")
	vision_topic	= (f"{config.TOPIC_VEHICLE_PREFIX}/+/{config.TOPIC_VEHICLE_VISION_SUFFIX}")

	await client.subscribe(telemetry_topic)
	await client.subscribe(vision_topic)

	async for message in client.messages:
		try:
			payload = json.loads(message.payload)

			if message.topic.matches(telemetry_topic):
				vehicle_state = Vehicle(**payload)
				active_vehicles[vehicle_state.id]		= vehicle_state
				active_vehicles_times[vehicle_state.id]	= time.time()
				active_vehicles_done[vehicle_state.id]	= False

			elif message.topic.matches(vision_topic):
				for item in payload:
					v_pos = VehiclePosition(**item)
					if v_pos.timestamp is None:
						v_pos.timestamp = time.time()
					vehicles_pos.append(v_pos)

		except Exception as error:
			logger.error("Failed to parse vehicle message: %s", error)


async def loop_publish_state(client: aiomqtt.Client):
	"""
	Publish the current state of the controller, including the precedence queue.
	"""
	state_topic = config.TOPIC_CONTROLLER_STATUS
	while True:
		state_payload = {
			"precedence_queue"	: precedence_queue,
			"ghosts"			: [ghost.model_dump(mode="json")for ghost in ghosts.values()],
		}
		await client.publish(state_topic, payload=json.dumps(state_payload))
		await asyncio.sleep(1.0 / UPDATES_P_S_CONTROLLER)


async def loop_control(client: aiomqtt.Client):
	"""
	Continuously evaluate traffic and publish commands.
	"""
	global ghosts
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
		ghosts			= {}
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
				
				ghosts[ghost_id] = ghost_v
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
			loop_listen(client),
			loop_publish_state(client),
			loop_control(client)
		)



if __name__ == "__main__":
	asyncio.run(main())