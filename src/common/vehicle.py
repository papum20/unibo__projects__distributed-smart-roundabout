import logging
import math

from common import math_utils, physics, roundabout
from common.const import (
	CAR_LENGTH,
	CAR_WIDTH,
	LANE_WIDTH,
	ROUNDABOUT_N_ROADS,
	ROUNDABOUT_PERIMETER,
	ROUNDABOUT_POS,
	VEHICLE_ANGLE_TRAVELED_MIN_RAD,
	VEHICLE_SAFETY_MARGIN_M,
)
from common.models.models import (
	Command
)
from common.models.vehicle import (
	Vehicle, VehicleNavState, VehiclePosition, VehicleState
)



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


def vehicle_from_pos(
	v_pos	: VehiclePosition,
	new_id	: str | None	= None,
	state	: VehicleState	= VehicleState.DISCONNECTED
) -> Vehicle:
	"""
	Create a Vehicle object from a VehiclePosition object.
	"""
	return Vehicle(
		id			= new_id if new_id is not None else v_pos.id,
		pos			= v_pos.pos,
		pos_angle	= v_pos.pos_angle,
		speed		= v_pos.speed,
		nav_state	= v_pos.nav_state,

		state		= state,
		entry_road	= get_predicted_entry(v_pos),
		exit_road	= get_predicted_exit(v_pos)
	)



#
# NAVIGATION
#

def v_navigate(v: Vehicle, dt: float, logger: logging.Logger|None = None) -> Vehicle:
	"""
	Update vehicle position and speed, including angle, pos_angle, angle_traveled, nav_state.
	
	@return : the updated vehicle (also modified)
	"""
	if dt == 0 or (v.speed == 0 and v.acceleration <= 0):
		# stopped, do not move
		return v

	if v.nav_state == VehicleNavState.APPROACHING:
		# drive on the right side of the road (positive offset)
		entry_target	= roundabout.get_point_on_road(v.entry_road, lane_offset=LANE_WIDTH/2)
		tta				= physics.v_ride_t( v, math_utils.get_dist(v.pos, entry_target) )
		t_diff	= dt - tta
		ride_t	= min(tta, dt)
		v.pos	= physics.v_move_towards(v, entry_target, ride_t)
		v.speed	= physics.v_update_speed(v, ride_t)

		# reached entry
		if t_diff >= 0:
			v.nav_state			= VehicleNavState.IN_ROUNDABOUT
			v.pos_angle			= roundabout.get_road_angle(v.entry_road, lane_offset=LANE_WIDTH/2)
			v.angle_traveled	= 0.0
			if logger is not None:
				logger.info("Vehicle %s entered the roundabout (IN_ROUNDABOUT) on road %d", v.id, v.entry_road)
			return v_navigate(v, t_diff, logger=logger)
			
	elif v.nav_state == VehicleNavState.IN_ROUNDABOUT:
		exit_angle		= roundabout.get_road_angle(v.exit_road, lane_offset=-LANE_WIDTH/2)
		tta				= physics.v_ride_t( v, math_utils.get_dist_on_circle(v.pos_angle, exit_angle) )
		t_diff			= dt - tta
		ride_t			= min(tta, dt)
		old_angle			= v.pos_angle
		v.pos_angle, v.pos	= physics.v_move_on_circle(v, ride_t)
		v.angle_traveled	+= (v.pos_angle - old_angle) % (2 * math.pi)
		v.speed				= physics.v_update_speed(v, ride_t)
		
		# reached exit - make sure to not exit immediately (if entry==exit)
		if t_diff >= 0 and v.angle_traveled > VEHICLE_ANGLE_TRAVELED_MIN_RAD:
			v.nav_state		= VehicleNavState.EXITING
			# snap angle, or won't drive straight
			v.pos_angle		= roundabout.get_road_angle(v.exit_road)
			# snap position to correct lane
			v.pos	= roundabout.get_point_on_road(v.exit_road, lane_offset=-LANE_WIDTH/2)
			if logger is not None:
				logger.info("Vehicle %s is exiting the roundabout (EXITING) on road %d", v.id, v.exit_road)
			return v_navigate(v, t_diff, logger=logger)

	else:
		v.pos	= physics.v_move_on_direction(v, dt)
		v.speed = physics.v_update_speed(v, dt)
		
	return v



def evaluate_safely(
	v1: Vehicle, v_others: list[VehiclePosition], additional_safety_margin: float = 0.0
) -> Command|None:
	""" 
	Check if the vehicle should slow down to avoid crashing in the one in front.
	This is an additional layer of safety to the controller.

	@param additional_safety_margin : safety margin to add to VEHICLE_SAFETY_MARGIN_M
	@return : the max acceleration that can be kept safely, or None if none is safe
	"""
	new_acc			= v1.params.max_accel
	safety_margin	= VEHICLE_SAFETY_MARGIN_M + additional_safety_margin
	closest_v2		= None
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
			if not v1.is_on_same_road(v2):
				continue
			v1_dist = math_utils.get_dist(v1.pos, ROUNDABOUT_POS)
			v2_dist = math_utils.get_dist(v2.pos, ROUNDABOUT_POS)
			if (
				(v1.nav_state == VehicleNavState.APPROACHING	and v2_dist >= v1_dist) or
				(v1.nav_state == VehicleNavState.EXITING		and v2_dist <= v1_dist)
			):
				continue
			gap = abs(v1_dist - v2_dist)

		if 0 < gap < closest_gap:
			closest_gap	= gap
			closest_v2	= v2

	if closest_v2 is not None:
		braking_dist_soft	= v1.get_stop_behind_margin(closest_v2, v1_acc_brake=v1.get_acc_brake())
		braking_dist_hard	= v1.get_stop_behind_margin(closest_v2, v1_acc_brake=v1.params.max_brake)
		if braking_dist_soft < safety_margin + v1.get_reaction_time_dist():
			if braking_dist_soft > safety_margin:
				new_acc = min(new_acc, -v1.get_acc_brake())
			elif braking_dist_hard > safety_margin:
				# start braking hard immediately
				new_acc = min(new_acc, -v1.params.max_brake)
			else:
				# it may already be too late
				new_acc = None
		elif closest_gap < safety_margin:
				# if gap too close (even at speed 0): increase it
				new_acc = min(new_acc, -v1.get_acc_brake())

	if new_acc is None:
		return None
	return Command(target_acceleration=new_acc)



#
# COLLISIONS
#

def _v_predict_conflict(
	v_app		: Vehicle,
	v_in		: Vehicle,
	v_app_acc	: float|None	= None,
	v_in_acc	: float|None	= None,
	dt			: float|None	= None
) -> tuple[Vehicle, Vehicle]|None:
	"""
	@param dt : time delay for prediction, defaults to the time it takes for v_app to reach the conflict point
	@return : the predicted positions of v_app approaching and v_in in roundabout, after dt seconds;
	None in case of error
	"""
	if v_app.nav_state != VehicleNavState.APPROACHING or v_in.nav_state != VehicleNavState.IN_ROUNDABOUT:
		return None
	# tta to the exact point, for following calculations
	v1_tta = physics.v_ride_t(
		v_app, math_utils.get_dist_to_roundabout(v_app.pos),
		new_acc=v_app_acc, margin=0 )
	if dt is None:
		dt = v1_tta

	new_v1				= v_app.model_copy(deep=True)
	new_v1.acceleration = v_app_acc if v_app_acc is not None else v_app.acceleration
	v_navigate(new_v1, dt)
	new_v2				= v_in.model_copy(deep=True)
	new_v2.acceleration = v_in_acc if v_in_acc is not None else v_in.acceleration
	v_navigate(new_v2, dt)

	return (new_v1, new_v2)


def v_entry_conflict(
	v1			: Vehicle,
	v2			: Vehicle,
	v1_acc		: float|None	= None,
	v2_acc		: float|None	= None,
	safety_dist	: float			= 0.0
) -> tuple[float, tuple[Vehicle, Vehicle]] | None:
	"""
	If v1 is approaching and v2 is already in the the roundabout,
	determine when they will reach the conflict point.

	@param v1_acc : optional new acceleration for v1, otherwise use its current one.
	@param safety_dist : additional safety distance to consider (careful, untested, may cause deadlocks or other problems)
	@return :
		(time_diff, (v1_predicted, v2_predicted)), with the time difference between
		v1 and v2 at the conflict point (v1_tta - v2_tta). 0 if they arrive at the same time,
		within a tolerance margin. None if none of them will arrive (both will stop before).
	"""
	if v1.nav_state != VehicleNavState.APPROACHING or v2.nav_state != VehicleNavState.IN_ROUNDABOUT:
		return None

	# times to get to conflict point are more conservative:
	# they also take into account some margins, e.g. for the car size
	conflict_angle		= roundabout.get_road_angle(v1.entry_road)
	v1_dist_to_conflict	= math_utils.get_dist(v1.pos, roundabout.get_point_on_road(v1.entry_road, lane_offset=LANE_WIDTH/2))
	v2_dist_to_conflict	= math_utils.get_dist_on_circle(v2.pos_angle, conflict_angle)
	# if v1 wants to enter, it must free the margin's space and not remain stationary there
	v1_tta_conflict		= physics.v_ride_t(v1, v1_dist_to_conflict, new_acc=v1_acc, margin=-CAR_LENGTH / 2 - CAR_WIDTH)
	if v2_dist_to_conflict >= ROUNDABOUT_PERIMETER / 2:
		# v2 has already passed the point, so it has arrived before v1
		v2_tta_conflict = -float('inf')
	else:
		# v2 has to free the conflict point too
		v2_tta_conflict		= physics.v_ride_t(v2, v2_dist_to_conflict, new_acc=v2_acc, margin=-CAR_LENGTH)
	
	if not math.isfinite(v2_tta_conflict) and not math.isfinite(v1_tta_conflict):
		return None
	pred	= _v_predict_conflict(v1, v2, v1_acc, v2_acc)
	t_diff	= v1_tta_conflict - v2_tta_conflict
	if pred is None:
		return None
	pred_v1, pred_v2 = pred
	if min(
		math_utils.get_dist_on_circle(pred_v1.pos_angle, pred_v2.pos_angle),
		math_utils.get_dist_on_circle(pred_v2.pos_angle, pred_v1.pos_angle)
	) < CAR_LENGTH + safety_dist:
		# if they are too close, consider them as arriving at the same time
		return 0, (pred_v1, pred_v2)

	if t_diff > 0:
		# if v1 arrives after v2, check for conflict while v1 is still approaching,
		# but has occupied the conflict point with the front part of the car
		pred_conflict = _v_predict_conflict(v1, v2, v1_acc, v2_acc, v1_tta_conflict)
		if pred_conflict is None:
			# if they can get to the conflict point, this should never be None as well
			return None
		conflict_v1, conflict_v2 = pred_conflict
		#if v1_dist_to_conflict < CAR_LENGTH + safety_dist:
		#	return 0.0, (pred_v1, pred_v2)
		# then, check that v2 has already freed the conflict point
		v2_dist_from_conflict = math_utils.get_dist_on_circle(conflict_angle, conflict_v2.pos_angle)
		if v2_dist_from_conflict < CAR_LENGTH + safety_dist:
			return 0.0, (pred_v1, pred_v2)
	elif math.isfinite(v2_tta_conflict) and t_diff < 0:
		# if v1 arrives before v2,
		# check that v1 frees the conflict point before v2 occupies it with the front part of the car
		pred_conflict = _v_predict_conflict(v1, v2, v1_acc, v2_acc, v2_tta_conflict)
		if pred_conflict is None:
			# if they can get to the conflict point, this should never be None as well
			return None
		conflict_v1, conflict_v2 = pred_conflict
		# then, check that v1 has already freed the conflict point
		v1_dist_from_conflict = math_utils.get_dist_on_circle(conflict_angle, conflict_v1.pos_angle)
		if v1_dist_from_conflict < CAR_LENGTH + safety_dist:
			return 0.0, (pred_v1, pred_v2)
	return t_diff, (pred_v1, pred_v2)



def _vehicle_heading(v: VehiclePosition) -> float:
	"""Return the direction in which the vehicle is pointing."""
	if v.nav_state == VehicleNavState.APPROACHING:
		return v.pos_angle + math.pi
	if v.nav_state == VehicleNavState.IN_ROUNDABOUT:
		# Vehicles travel counter-clockwise around the circle.
		return v.pos_angle + math.pi / 2.0
	return v.pos_angle

def _vehicle_corners(
	v: VehiclePosition,
	length	: float = CAR_LENGTH,
	width	: float = CAR_WIDTH,
) -> list[tuple[float, float]]:
	heading = _vehicle_heading(v)

	forward_x = math.cos(heading)
	forward_y = math.sin(heading)

	side_x = -forward_y
	side_y = forward_x

	half_length = length / 2.0
	half_width	= width / 2.0

	center_x = v.pos.x
	center_y = v.pos.y

	return [
		(
			center_x + forward_x * half_length + side_x * half_width,
			center_y + forward_y * half_length + side_y * half_width,
		),
		(
			center_x + forward_x * half_length - side_x * half_width,
			center_y + forward_y * half_length - side_y * half_width,
		),
		(
			center_x - forward_x * half_length + side_x * half_width,
			center_y - forward_y * half_length + side_y * half_width,
		),
		(
			center_x - forward_x * half_length - side_x * half_width,
			center_y - forward_y * half_length - side_y * half_width,
		),
	]

def _project_polygon(
	polygon	: list[tuple[float, float]],
	axis	: tuple[float, float],
) -> tuple[float, float]:
	projections = [
		point_x * axis[0] + point_y * axis[1]
		for point_x, point_y in polygon
	]

	return min(projections), max(projections)


def v_collide(
	v1		: VehiclePosition,
	v2		: VehiclePosition,
	length	: float = CAR_LENGTH,
	width	: float = CAR_WIDTH,
) -> bool:
	"""
	@return : True when two oriented vehicle rectangles overlap.
	"""
	if abs(v1.pos.x - v2.pos.x) > 3 * length or abs(v1.pos.y - v2.pos.y) > 3 * length:
		# quick check for long distance
		return False
	
	polygon1 = _vehicle_corners(v1, length, width)
	polygon2 = _vehicle_corners(v2, length, width)

	axes = []

	# Separating Axis Theorem:
	# Two rectangles do not collide if we can find one direction (axis)
	# where their projections are separate.
	for polygon in (polygon1, polygon2):
		# for each edge of the rectangle, we take the perpendicular axis
		for index in range(4):
			point1 = polygon[index]
			point2 = polygon[(index + 1) % 4]

			edge_x = point2[0] - point1[0]
			edge_y = point2[1] - point1[1]

			edge_length = math.hypot(edge_x, edge_y)
			# Perpendicular axis to the edge.
			# Axis represented as a direction, a unit vector.
			axis = (-edge_y / edge_length, edge_x / edge_length)
			axes.append(axis)

	for axis in axes:
		min1, max1 = _project_polygon(polygon1, axis)
		min2, max2 = _project_polygon(polygon2, axis)

		if max1 < min2 or max2 < min1:
			# separating axis found: rectangles do not overlap.
			return False

	return True
