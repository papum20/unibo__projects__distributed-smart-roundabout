import logging
import math

from common import math_utils, physics, roundabout
from common.const import (
	LANE_WIDTH,
	ROAD_WIDTH,
	ROUNDABOUT_N_ROADS,
	ROUNDABOUT_POS,
	ROUNDABOUT_RADIUS,
	VEHICLE_ANGLE_TOL_RAD,
	VEHICLE_ANGLE_TRAVELED_MIN_RAD,
	VEHICLE_DIST_TOL,
	VEHICLE_SAFETY_MARGIN_M,
)
from common.models.models import (
	Command,
	Position
)
from common.models.vehicle import (
	Vehicle, VehicleNavState, VehiclePosition
)



def vehicle_navigate(dt: float, v: Vehicle, logger: logging.Logger|None = None) -> Vehicle:
	if v.speed == 0 and v.acceleration <= 0:
		# stopped, do not move
		return v

	if v.nav_state == VehicleNavState.APPROACHING:
		# drive on the right side of the road (positive offset)
		target_x, target_y	= roundabout.get_point_on_road(
			v.entry_road, distance_from_boundary=-ROAD_WIDTH, lane_offset=LANE_WIDTH/2
		)
		entry_target		= Position(x=target_x, y=target_y)
		
		v.pos = physics.move_towards(v.pos, entry_target, v.speed, dt)
		
		if math_utils.get_dist(v.pos, entry_target) <= VEHICLE_DIST_TOL:
			v.nav_state			= VehicleNavState.IN_ROUNDABOUT
			v.pos_angle			= roundabout.get_road_angle(v.entry_road)
			v.angle_traveled	= 0.0
			if logger is not None:
				logger.info("Vehicle %s entered the roundabout (IN_ROUNDABOUT) on road %d", v.id, v.entry_road)
			
	elif v.nav_state == VehicleNavState.IN_ROUNDABOUT:
		old_angle	= v.pos_angle

		v.pos_angle, v.pos = physics.move_on_circle(
			ROUNDABOUT_POS, ROUNDABOUT_RADIUS, v.pos_angle, v.speed, dt
		)

		angle_diff_frame = (v.pos_angle - old_angle) % (2 * math.pi)
		v.angle_traveled += angle_diff_frame
		
		# check if we reached the exit road
		exit_angle = roundabout.get_road_angle(v.exit_road)
		angle_diff = abs(v.pos_angle - exit_angle)
		# handle wrap-around at 2*pi
		angle_diff = min(angle_diff, 2*math.pi - angle_diff) 

		# make sure to not exit immediately (if entry==exit)
		if angle_diff < VEHICLE_ANGLE_TOL_RAD and v.angle_traveled > VEHICLE_ANGLE_TRAVELED_MIN_RAD:
			v.nav_state		= VehicleNavState.EXITING
			# snap angle, or won't drive straight
			v.pos_angle		= exit_angle
			# snap position to correct lane
			exit_x, exit_y	= roundabout.get_point_on_road(
				v.exit_road, distance_from_boundary=-ROAD_WIDTH, lane_offset=-LANE_WIDTH / 2
			)
			v.pos = Position(x=exit_x, y=exit_y)
			if logger is not None:
				logger.info("Vehicle %s is exiting the roundabout (EXITING) on road %d", v.id, v.exit_road)

	elif v.nav_state == VehicleNavState.EXITING:
		v.pos = physics.vehicle_move_on_direction(v, dt)
		
	return v



def get_predicted_entry(v_pos: VehiclePosition) -> int:
	"""
	Predict the entry road in the safest way possible (worst case scenario).
	@return: the entry road index
	"""
	angle_per_road	= (2 * math.pi) / ROUNDABOUT_N_ROADS

	if v_pos.nav_state == VehicleNavState.APPROACHING:
		# closest road based on current angle
		return round(v_pos.pos_angle / angle_per_road) % ROUNDABOUT_N_ROADS
	else:
		return 0	# doesn't matter


def get_predicted_exit(v_pos: VehiclePosition) -> int:
	"""
	Predict the exit road in the safest way possible (worst case scenario).
	@return: the exit road index
	"""
	angle_per_road	= (2 * math.pi) / ROUNDABOUT_N_ROADS

	if v_pos.nav_state == VehicleNavState.IN_ROUNDABOUT:
		# worst-case scenario: it exits at the furthest possible road (the one just passed)
		# int() acts as floor()
		return int((v_pos.pos_angle - 1) / angle_per_road) % ROUNDABOUT_N_ROADS
	elif v_pos.nav_state == VehicleNavState.EXITING:
		# closest road based on current angle
		return round(v_pos.pos_angle / angle_per_road) % ROUNDABOUT_N_ROADS
	else:
		return 0	# doesn't matter


def is_same_approach_road(v1: Vehicle, v2: VehiclePosition) -> bool:
	road_angle	= roundabout.get_road_angle(v1.entry_road)
	angle_diff	= (v2.pos_angle - road_angle) % (2 * math.pi)

	return min(angle_diff, 2 * math.pi - angle_diff) < VEHICLE_ANGLE_TOL_RAD



def evaluate_safely(v1: Vehicle, v_others: list[VehiclePosition]) -> Command:
	""" 
	Check if the vehicle should slow down to avoid crashing in the one in front.
	This is an additional layer of safety to the controller.
	@return : the max acceleration that can be kept safely
	"""
	new_acc		= v1.params.max_accel
	closest_v2	= None
	# closest non-negative distance to another v
	closest_gap = float("inf")

	for v2 in v_others:
		if v1.id == v2.id or v1.nav_state != v2.nav_state:
			continue
		if v1.nav_state == VehicleNavState.IN_ROUNDABOUT:
			gap = math_utils.get_dist_on_circle(
				v1.pos_angle,
				v2.pos_angle,
			)
		else:
			if not is_same_approach_road(v1, v2):
				continue
			v1_dist = math_utils.get_dist(v1.pos, ROUNDABOUT_POS)
			v2_dist = math_utils.get_dist(v2.pos, ROUNDABOUT_POS)
			if v2_dist >= v1_dist:
				continue
			gap = v1_dist - v2_dist

		if 0 < gap < closest_gap:
			closest_gap	= gap
			closest_v2	= v2

	if closest_v2 is not None:
		safety_margin_soft	= v1.get_stop_behind_margin(closest_v2, v1_acc_brake=v1.get_acc_brake())
		if safety_margin_soft < VEHICLE_SAFETY_MARGIN_M + v1.get_reaction_time_dist():
			if closest_gap < VEHICLE_SAFETY_MARGIN_M:
				# start braking hard immediately
				# (if safety_margin_hard < VEHICLE_SAFETY_MARGIN_M, it may already be too late)
				new_acc = min(new_acc, -v1.params.max_brake)
			else:
				new_acc = min(new_acc, -v1.get_acc_brake())
		elif closest_gap < VEHICLE_SAFETY_MARGIN_M:
				# if gap too close: increase it
				new_acc = min(new_acc, -v1.get_acc_brake())

	return Command(target_acceleration=new_acc)
