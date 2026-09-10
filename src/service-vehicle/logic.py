import logging
import random

from common import math_utils, physics, roundabout, vehicle
from common.const import (
	CAR_LENGTH,
	ROAD_WIDTH,
	ROUNDABOUT_PERIMETER,
	ROUNDABOUT_PROXIMITY_DIST,
	LANE_WIDTH,
	ROAD_LENGTH,
	ROUNDABOUT_N_ROADS,
	ROUNDABOUT_POS,
	ROUNDABOUT_RADIUS,
	VEHICLE_SAFETY_MARGIN_M,
	VEHICLE_SPEED_TOL_PERC
)
from common.models.models import (
	Command, Position
)
from common.models.vehicle import (
	VEHICLE_FAILSAFE_MAX_SPEED_M_S,
	Vehicle, VehicleNavState, VehiclePosition, VehicleState, 
)



logger = logging.getLogger(__name__)


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
	
	v_pos			= v.to_pos()
	v_dist			= math_utils.get_dist(v.pos, ROUNDABOUT_POS)
	front_gap		= float("inf")
	front_v2_pos	= None
	collisions_n	= 0

	for v2_pos in other_positions:
		if v2_pos.nav_state != v_pos.nav_state or not v.is_on_same_road(v2_pos):
			continue

		v2_dist = math_utils.get_dist(v2_pos.pos, ROUNDABOUT_POS)
		if not physics.vehicle_collide(v_pos, v2_pos):
			if v2_dist < v_dist:
				front_v2_pos	= v2_pos
				front_gap		= min(front_gap, v_dist - v2_dist)
			continue
		collisions_n += 1

		if v2_dist < v_dist:
			logger.debug("Vehicle %s waiting for overlapping closer vehicle %s", v.id, v2_pos.id)
			return Command(target_acceleration=-v.params.max_brake)

	if collisions_n > 0:
		if front_v2_pos is not None:
			return vehicle.evaluate_safely(v, [front_v2_pos], front_v2_pos.get_reaction_time_dist())
		return Command(target_acceleration=v.params.max_accel)
	return None


def vehicle_navigate_emergency(
	v1			: Vehicle,
	v_others	: list[VehiclePosition],
) -> Command | None:
	"""
	Do basic safety checks:
	- with hard brakes (so they can pass over controller's instructions).
	- at spawn point

	@return : a Command if necessary for emergency, otherwise None
	"""
	spawn_cmd = vehicle_navigate_spawn(v1, v_others)
	if spawn_cmd is not None:
		return spawn_cmd
	
	# hard check for tail-gating
	safe_cmd = vehicle.evaluate_safely(v1, v_others)
	if safe_cmd is None or safe_cmd.target_acceleration == -v1.params.max_brake:
		return Command(target_acceleration=-v1.params.max_brake)

	for v2_pos in v_others:
		# entrance
		if v1.nav_state == VehicleNavState.APPROACHING and v2_pos.nav_state == VehicleNavState.IN_ROUNDABOUT:

			conflict_angle		= roundabout.get_road_angle(v1.entry_road)
			v1_dist_to_conflict	= math_utils.get_dist_on_circle(v1.pos_angle, conflict_angle)
			v2_dist_to_conflict	= math_utils.get_dist(v2_pos.pos, ROUNDABOUT_POS) - ROUNDABOUT_RADIUS

			# stop if v2 has already occupied the conflict point
			stop_dist = v1.get_stop_dist(v1.params.max_brake) + CAR_LENGTH
			if (
				v2_dist_to_conflict <= CAR_LENGTH and
				stop_dist <= v1_dist_to_conflict <= stop_dist + max(v1.get_reaction_time_dist(), VEHICLE_SAFETY_MARGIN_M)
			):
				return Command(target_acceleration=-v1.params.max_brake)

	return None



def evaluate_failsafe(v1: Vehicle, v_others: list[VehiclePosition]) -> Command:
	""" 
	Failsafe is a degraded mode: no central controller. 
	The car relies strictly on stopping and checking local distances.
	"""
	# close to entrance: stop and check
	dist_to_entry = math_utils.get_dist(v1.pos, ROUNDABOUT_POS) - ROUNDABOUT_RADIUS - ROAD_WIDTH
	if (
		v1.nav_state == VehicleNavState.APPROACHING and v1.speed > 0
		# if already stopped close to the entrance, continue (otherwise will never enter).
		# add VEHICLE_SAFETY_MARGIN_M so that it won't stop after the entrance.
		and v1.get_stop_dist() >= dist_to_entry + VEHICLE_SAFETY_MARGIN_M > ROUNDABOUT_PROXIMITY_DIST
	):
		return Command(target_acceleration=-v1.params.max_brake)

	safe_cmd = vehicle.evaluate_safely(v1, v_others)
	if safe_cmd is not None:
		new_acc = safe_cmd.target_acceleration
	else:
		return Command(target_acceleration=-v1.params.max_brake)

	# slower speed
	if abs(v1.speed * VEHICLE_SPEED_TOL_PERC - VEHICLE_FAILSAFE_MAX_SPEED_M_S) > 0:
		if v1.speed > VEHICLE_FAILSAFE_MAX_SPEED_M_S:
			new_acc = min(new_acc, -v1.get_acc_brake())
	else:
		new_acc = min(new_acc, 0.0)

	for v2_pos in v_others:
		# entrance
		if v1.nav_state == VehicleNavState.APPROACHING and v2_pos.nav_state == VehicleNavState.IN_ROUNDABOUT:

			conflict_angle		= roundabout.get_road_angle(v1.entry_road)
			v2_dist_to_conflict = math_utils.get_dist_on_circle(v2_pos.pos_angle, conflict_angle)

			# avoid deadlocks if v2 has stopped
			if v2_pos.speed == 0.0 and CAR_LENGTH <= v2_dist_to_conflict < ROUNDABOUT_PERIMETER / 2:
				continue
			
			v1_dist_to_conflict = math_utils.get_dist(v1.pos, ROUNDABOUT_POS) - ROUNDABOUT_RADIUS
			if v1_dist_to_conflict < ROUNDABOUT_PROXIMITY_DIST + ROAD_WIDTH:
				
				# most cautios safety distance
				#if v2_dist_to_conflict <= (
				#	v2_pos.get_stop_dist(acc_brake=v1.get_acc_brake()) + v2_pos.get_reaction_time_dist() + VEHICLE_SAFETY_MARGIN_M
				#):
				#	new_acc = -v1.params.max_brake
				#	break

				v2 = vehicle.vehicle_from_pos(v2_pos)
				# use acc max, to be safe 
				predicted = physics.vehicle_enters_first(v1, v2, v1_acc=new_acc, v2_acc=v2.params.max_accel)
				if predicted is not None:
					pred_v1, pred_v2	= predicted
					# check safety of v2 behind v1.
					# double reaction time, for double latency of communicating with controller forth and back
					pred_cmd			= vehicle.evaluate_safely(pred_v2, [pred_v1.to_pos()], v2.get_reaction_time_dist())
					if pred_cmd is not None:
						continue
				else:
					new_acc = -v1.params.max_brake
					break

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
