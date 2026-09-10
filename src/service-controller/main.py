import asyncio
import json
import logging
import math
import time
import aiomqtt

from common import math_utils, physics, roundabout, vehicle
from common.const import (
	ROAD_WIDTH,
	ROUNDABOUT_N_ROADS, ROUNDABOUT_PERIMETER,
	ROUNDABOUT_POS, ROUNDABOUT_RADIUS,
	TIMER_NETWORK_TIMEOUT, UPDATES_P_S_CONTROLLER,
	CAR_LENGTH,
	VEHICLE_SAFETY_MARGIN_M,
)
from common.get_env import config
from common.models.models import Command, SystemCommand, SystemCommandValue
from common.models.vehicle import (
	Vehicle, VehicleNavState, VehiclePosition, VehicleState
)



logger = logging.getLogger(__name__)

is_paused	: bool	= False

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

# min speed to keep inside roundabout: only go below for emergency or safety braking, not for e.g. yielding
VEHICLE_INSIDE_SPEED_MIN_PERC	= 0.5



def sort_vehicles(vehicles: list[Vehicle]) -> list[Vehicle]:
	"""
	Sort vehicles, so that we have updated acc values where it's most important:
	- those in front first (inside the roundabout, sort clockwise)
	- inside has priority on approaching; approaching vehicles should play it safer
	- exiting vehicles don't matter much
	"""
	def sort_key(v: Vehicle) -> tuple[int, float]:
		distance_to_center = math_utils.get_dist(v.pos, ROUNDABOUT_POS)

		if v.nav_state == VehicleNavState.EXITING:
			# furthest from center first
			return 0, -distance_to_center
		if v.nav_state == VehicleNavState.IN_ROUNDABOUT:
			# Numeric angle order: 0 -> 2*pi
			return 1, v.pos_angle % (2 * math.pi)
		if v.nav_state == VehicleNavState.APPROACHING:
			# closest to center first
			return 2, distance_to_center

		return 3, 0.0
	return sorted(vehicles, key=sort_key)


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


def vehicle_enters_later_safely(
	v_app		: Vehicle,
	v_in		: Vehicle,
	v_app_acc	: float,
	v_in_acc	: float|None = None
) -> bool:
	"""
	@return : True if v_app enters later and does it safely
	"""
	predicted = physics.vehicle_enters_later( v1=v_app, v2=v_in, v1_acc=v_app_acc, v2_acc=v_in_acc )
	if predicted is not None:
		pred_v1, pred_v2	= predicted
		cmd					= vehicle.evaluate_safely(pred_v1, [pred_v2.to_pos()])
		return cmd is not None and cmd.target_acceleration >= v_app_acc
	return False
		



def evaluate(vehicles: list[Vehicle]) -> dict[str, Command]:
	"""
	@param vehicles: a list of all current vehicles.
	@return a dictionary mapping vehicle_id -> Command.
	"""
	vehicles		= sort_vehicles(vehicles)
	vehicles_pos	= [v.to_pos() for v in vehicles]
	# default: None, tell everyone to maintain speed, each update can only reduce it.
	commands 		= {}

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
			if dist_to_roundabout <= 6 * CAR_LENGTH:
				precedence_queue.append(v1.id)
		
		
	for v1 in vehicles:

		#if v1.nav_state == VehicleNavState.APPROACHING:
		#	dist_to_roundabout = math_utils.get_dist(v1.pos, ROUNDABOUT_POS) - ROUNDABOUT_RADIUS
		#	if dist_to_roundabout < CAR_LENGTH / 2 + CAR_WIDTH:
		#		# roundabout already occupied, no need to stop there
		#		commands[v1.id] = Command(target_acceleration=v1.params.max_accel)
		#		continue

		# step 1: safety checks

		safe_cmd = vehicle.evaluate_safely(v1, vehicles_pos)
		if safe_cmd is not None:
			new_acc = safe_cmd.target_acceleration
		else:
			new_acc = -v1.params.max_brake
		commands[v1.id] = Command(target_acceleration=new_acc)

		# step 2: optimizing, with max acc from prev step

		for v2 in vehicles:
			if v1.id == v2.id: continue

			new_acc		= commands[v1.id].target_acceleration
			# use updated acc if possible, for better prediction
			v2_curr_acc	= commands[v2.id].target_acceleration if commands.get(v2.id) is not None else v2.acceleration
			if new_acc == -v1.params.max_brake:
				break

			# inside-approaching conflict: slow down to yield
			if v1.nav_state == VehicleNavState.IN_ROUNDABOUT and v2.nav_state == VehicleNavState.APPROACHING:

				conflict_angle		= roundabout.get_road_angle(v2.entry_road)
				v2_dist_to_conflict	= math_utils.get_dist(v2.pos, ROUNDABOUT_POS) - ROUNDABOUT_RADIUS

				# - if v2 is out of control: in failsafe mode, it will behave cautiously.
				# However, to avoid starvation for v2 and who's behind it, yield if possible.
				# If it's in failsafe, its exit data is reliable, otherwise
				# it's just estimated as a worst-case scenario, so that it won't influence our decisions.
				# - also try to stop if v2 has already occupied the conflict point
				# - to avoid deadlocks, never really stop but just slow down (except for emergencies)
				if should_yield_to(v1, v2) or v2.state != VehicleState.NORMAL or v2_dist_to_conflict <= CAR_LENGTH:

					v1_dist_to_conflict	= math_utils.get_dist_on_circle(v1.pos_angle, conflict_angle)
					v1_exit_angle		= roundabout.get_road_angle(v1.exit_road)
					v1_dist_to_exit		= math_utils.get_dist_on_circle(v1.pos_angle, v1_exit_angle)
					v1_in_min_speed		= VEHICLE_INSIDE_SPEED_MIN_PERC * v1.params.max_speed

					# check if v1 exits earlier, or if too far
					if v1_dist_to_exit <= v1_dist_to_conflict <= ROUNDABOUT_PERIMETER / 2 or v1_dist_to_conflict >= ROUNDABOUT_PERIMETER / 2:
						continue

					# coordinate with v2 (which has priority, in case), looking for the fastest option.
					# since v2 has priority, try to make it pass first (without hard braking, since it's not an emergency);
					# if it can't pass first with any of the possible accelerations for v1, just go at max acc (max or 0).

					# dist required to stop safely
					stop_dist = v1.get_stop_dist(v1.get_acc_brake()) + CAR_LENGTH + VEHICLE_SAFETY_MARGIN_M
					# if could stop safely
					if (
						v1.speed > v1_in_min_speed and
						stop_dist <= v1_dist_to_conflict <= stop_dist + max(v1.get_reaction_time_dist(), VEHICLE_SAFETY_MARGIN_M)
					):
						commands[v1.id].target_acceleration = min(new_acc, -v1.get_acc_brake())
						continue

					# in emergency cases, also try with hard brake
					stop_dist = v1.get_stop_dist(v1.params.max_brake) + CAR_LENGTH
					if (
						v2_dist_to_conflict <= CAR_LENGTH and
						stop_dist <= v1_dist_to_conflict <= stop_dist + max(v1.get_reaction_time_dist(), VEHICLE_SAFETY_MARGIN_M)
					):
						commands[v1.id].target_acceleration = min(new_acc, -v1.params.max_brake)
						break

					if v1.speed <= v1_in_min_speed:
						continue

					v1_acc_choices = [acc for acc in (new_acc, 0.0, -v1.get_acc_brake()) if acc <= new_acc]
					for v1_acc in v1_acc_choices:
						predicted = physics.vehicle_enters_first( v2, v1, v1_acc=v2_curr_acc, v2_acc=v1_acc )
						if predicted is not None:
							pred_v2, pred_v1	= predicted
							# check safety of v1 behind v2
							pred_cmd			= vehicle.evaluate_safely(pred_v1, [pred_v2.to_pos()])
							# if possible, look for a choice which avoids hard braking;
							# as a fallback, look for any safe option
							if pred_cmd is not None and pred_cmd.target_acceleration >= -v1.get_acc_brake():
								commands[v1.id].target_acceleration = v1_acc
								break
					# if can't stop safely, just pass: the approaching v2,
					# either guided by controller or failsafe mode, will brake

			# inside-approaching, conflict
			elif v1.nav_state == VehicleNavState.APPROACHING and v2.nav_state == VehicleNavState.IN_ROUNDABOUT:

				conflict_angle		= roundabout.get_road_angle(v1.entry_road)
				v1_dist_to_conflict	= math_utils.get_dist(v1.pos, ROUNDABOUT_POS) - ROUNDABOUT_RADIUS
				v2_dist_to_conflict	= math_utils.get_dist_on_circle(v2.pos_angle, conflict_angle)

				# avoid deadlocks if v2 has stopped right before, to yield
				if v2.speed == 0.0 and v2_curr_acc <= 0 and CAR_LENGTH <= v2_dist_to_conflict <= ROUNDABOUT_PERIMETER / 2:
					continue

				# check if v2 has just passed the entrance and is physically blocking the way
				#v2_dist_from_conflict = math_utils.get_dist_on_circle(conflict_angle, v2.pos_angle)
				#if v2_dist_from_conflict < CAR_LENGTH + VEHICLE_SAFETY_MARGIN_M:
				#	v1_dist_to_entry = math_utils.get_dist(v1.pos, ROUNDABOUT_POS) - ROUNDABOUT_RADIUS - CAR_LENGTH / 2
				#	if v1_dist_to_entry <= v1.get_stop_dist(acc_brake=v1.get_acc_brake()) + max(v1.get_reaction_time_dist(), VEHICLE_SAFETY_MARGIN_M):
				#		commands[v1.id].target_acceleration = min(new_acc, -v1.get_acc_brake())
				#	if v1_dist_to_entry <= v1.get_stop_dist(acc_brake=v1.params.max_brake):
				#		commands[v1.id].target_acceleration = min(new_acc, -v1.params.max_brake)
				#		continue
				
				v2_exit_angle		= roundabout.get_road_angle(v2.exit_road)
				v2_dist_to_exit		= math_utils.get_dist_on_circle(v2.pos_angle, v2_exit_angle)
				
				# check if v2 exits earlier
				if v2_dist_to_exit <= v2_dist_to_conflict <= ROUNDABOUT_PERIMETER / 2:
					continue

				# even if v2 has to yield, v1 still has to check:
				# v2 may be unable to slow down safely, and v1 should anyway check for those ahead.
				# if v2 in failsafe, it will have priority, but maybe v1 can pass without interfering

				# coordinate with v2

				if vehicle_enters_later_safely(v1, v2, new_acc, v_in_acc=v2_curr_acc):
					continue

				# if possible, go faster
				predicted = physics.vehicle_enters_first(v1, v2, v1_acc=new_acc, v2_acc=v2_curr_acc)
				if predicted is not None:
					pred_v1, pred_v2	= predicted
					# check safety of v2 behind v1
					pred_cmd			= vehicle.evaluate_safely(pred_v2, [pred_v1.to_pos()])
					# for simplicity, we allow hard braking here
					if pred_cmd is not None:
						continue
				# if v1 can't enter first safely at max acc, neither can it do at a lower one

				# if still far from roundabout, no need to brake
				v1_dist_to_entry = math_utils.get_dist(v1.pos, ROUNDABOUT_POS) - ROUNDABOUT_RADIUS - ROAD_WIDTH - CAR_LENGTH / 2
				if v1_dist_to_entry - v1.get_stop_dist(acc_brake=v1.get_acc_brake()) - v1.get_reaction_time_dist() > 0: # ROUNDABOUT_PROXIMITY_DIST:
					# as long as you can start braking later, no need to already do it now
					continue

				v1_acc_choices		= [acc for acc in (0.0, -v1.get_acc_brake(), -v1.params.max_brake) if acc < new_acc]
				for v1_acc in v1_acc_choices:
					if vehicle_enters_later_safely(v1, v2, v1_acc, v_in_acc=v2_curr_acc):
						commands[v1.id].target_acceleration = v1_acc
						break
				else:
					# no safe option: wait for v2 to pass
					commands[v1.id].target_acceleration = -v1.params.max_brake

	return commands



async def loop_listen(client):
	global is_paused
	telemetry_topic = (f"{config.TOPIC_VEHICLE_PREFIX}/+/{config.TOPIC_VEHICLE_TELEMETRY_SUFFIX}")
	vision_topic	= (f"{config.TOPIC_VEHICLE_PREFIX}/+/{config.TOPIC_VEHICLE_VISION_SUFFIX}")
	sysctrl_topic			= f"{config.TOPIC_SYSCTRL_PREFIX}/{config.TOPIC_SYSCTRL_BROADCAST_SUFFIX}"
	sysctrl_broadcast_topic = f"{config.TOPIC_SYSCTRL_PREFIX}/+"

	await client.subscribe(telemetry_topic)
	await client.subscribe(vision_topic)
	await client.subscribe(sysctrl_topic)
	await client.subscribe(sysctrl_broadcast_topic)

	async for message in client.messages:
		try:
			payload = json.loads(message.payload)

			if message.topic.matches(telemetry_topic):
				vehicle_state = Vehicle(**payload)
				# can't receive messages from disconnected vehicles (they're only sent for debugging).
				if vehicle_state.state != VehicleState.DISCONNECTED:
					active_vehicles[vehicle_state.id]		= vehicle_state
					active_vehicles_times[vehicle_state.id]	= time.time()
					active_vehicles_done[vehicle_state.id]	= False

			elif message.topic.matches(vision_topic):
				for item in payload:
					v_pos = VehiclePosition(**item)
					if v_pos.timestamp is None:
						v_pos.timestamp = time.time()
					vehicles_pos.append(v_pos)

			elif str(message.topic) in (sysctrl_topic, sysctrl_broadcast_topic):
				command = SystemCommand(**payload)
				if command.command == SystemCommandValue.PAUSE:
					is_paused = True
					logger.info("SysCtrl: Simulation PAUSED")
				elif command.command == SystemCommandValue.RESUME:
					is_paused = False
					logger.info("SysCtrl: Simulation RESUMED")

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
	last_update_t	= time.time()

	while True:
		current_t	= time.time()
		
		if is_paused:
			# prevent problems with times
			for vehicle_id in active_vehicles:
				active_vehicles_times[vehicle_id] += current_t - last_update_t
			last_update_t = current_t

			await asyncio.sleep(1.0 / UPDATES_P_S_CONTROLLER)
			continue

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

		# if a car is exiting, its ghost won't exit but continue inside the roundabout,
		# so also save the predictions of known cars without exiting
		v_exiting_next = []
		for v in active_vehicles.values():
			if v.is_exiting_next():
				v_t	 = active_vehicles_times.get(v.id, current_t)
				v_exiting_next.append(
					vehicle.vehicle_navigate(v=v.model_copy(deep=True), dt=current_t - v_t)
				)

		v_list	= []
		ghosts	= {}
		for v in active_vehicles.values():
			v_t = active_vehicles_times.get(v.id, current_t)
			v_list.append(
				vehicle.vehicle_navigate(v=v.model_copy(deep=True), dt=current_t - v_t)
			)
		# try to merge vision data
		for v_pos in vehicles_pos:
			
			ghost_v = vehicle.vehicle_from_pos(v_pos, state=VehicleState.DISCONNECTED)
			v_pos_t = v_pos.timestamp if v_pos.timestamp is not None else current_t
			ghost_v = vehicle.vehicle_navigate(v=ghost_v, dt=current_t - v_pos_t)
		
			# compare with known ones
			is_known = any(
				#math_utils.get_dist(v_pos.pos, known_v.pos) < VISION_MATCH_TOLERANCE_M
				physics.vehicle_collide(v1=ghost_v.to_pos(), v2=known_v)
				for known_v in [*v_list, *v_exiting_next]
			)
			if is_known:
				continue
					
			ghost_counter += 1
			ghost_id = f"GHOST-{ghost_counter}"
			ghost_v.id = ghost_id				
			
			ghosts[ghost_id] = ghost_v
			# if a ghost in a similar position is found, it won't be added because it will match with this
			v_list.append(ghost_v)
			logger.debug("Detected DISCONNECTED vehicle %s: %s", ghost_id, ghost_v)

		commands	= evaluate(v_list)
		
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
			
		last_update_t = current_t
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