import logging
import random

from common import math_utils, physics, roundabout, vehicle
from common.const import (
	CAR_LENGTH,
	ROAD_WIDTH,
	ROUNDABOUT_PROXIMITY_DIST,
	LANE_WIDTH,
	ROAD_LENGTH,
	ROUNDABOUT_N_ROADS,
	ROUNDABOUT_POS,
	ROUNDABOUT_RADIUS,
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
	collisions_n	= 0

	for other in other_positions:
		if other.nav_state != v_pos.nav_state:
			continue
		if not physics.vehicle_collide(v_pos, other):
			continue
		collisions_n += 1

		other_distance = math_utils.get_dist(other.pos, ROUNDABOUT_POS)
		if other_distance < v_dist:
			logger.debug("Vehicle %s waiting for closer vehicle %s", v.id, other.id)
			return Command(target_acceleration=-v.params.max_brake)
	if collisions_n > 0:
		return Command(target_acceleration=v.params.max_accel)
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
		# if already stopped close to the entrance, continue (otherwise will never enter)
		and v1.get_stop_dist() >= dist_to_entry > ROUNDABOUT_PROXIMITY_DIST
	):
		return Command(target_acceleration=-v1.params.max_brake)

	new_acc = vehicle.evaluate_safely(v1, v_others).target_acceleration

	# slower speed
	if abs(v1.speed * VEHICLE_SPEED_TOL_PERC - VEHICLE_FAILSAFE_MAX_SPEED_M_S) > 0:
		if v1.speed > VEHICLE_FAILSAFE_MAX_SPEED_M_S:
			new_acc = -v1.params.max_brake * 0.5
	else:
		new_acc = 0.0

	for v2 in v_others:
		safe_dist = v1.get_safety_dist(margin=CAR_LENGTH/2.0)

		# entrance
		if v1.nav_state == VehicleNavState.APPROACHING and v2.nav_state == VehicleNavState.IN_ROUNDABOUT:
			v1_dist_to_conflict = math_utils.get_dist(v1.pos, ROUNDABOUT_POS) - ROUNDABOUT_RADIUS - ROAD_WIDTH
			
			if v1_dist_to_conflict < ROUNDABOUT_PROXIMITY_DIST:
				conflict_angle		= roundabout.get_road_angle(v1.entry_road)
				v2_dist_to_conflict = math_utils.get_dist_on_circle(v2.pos_angle, conflict_angle)
				
				# most cautios safety distance
				if v2_dist_to_conflict <= v2.get_stop_dist():
					new_acc = min(new_acc, -v1.params.max_brake)
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
