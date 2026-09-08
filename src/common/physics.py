import math

from common import roundabout
from common import math_utils
from common.const import CAR_LENGTH, CAR_WIDTH, ROAD_WIDTH, ROUNDABOUT_POS, ROUNDABOUT_RADIUS, VEHICLE_SAFETY_MARGIN_M
from common.models.models import Position
from common.models.vehicle import (
	Vehicle, VehicleNavState, VehiclePosition
)
from common.math_utils import get_dist



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



def update_speed(speed: float, acc: float, dt: float, max_speed: float) -> float:
	"""Update speed, preventing it from going below 0 (reversing) or above max_speed."""
	new_speed = speed + (acc * dt)
	return max(0.0, min(new_speed, max_speed))

def vehicle_update_speed(v: Vehicle, dt: float, new_acc: float|None = None) -> float:
	"""Update speed, preventing it from going below 0 (reversing) or above max_speed."""
	return update_speed(v.speed, v.acceleration if new_acc is None else new_acc, dt, v.params.max_speed)



#
# MOVEMENT
#

def move_towards(current: Position, target: Position, speed: float, dt: float) -> Position:
	"""Move straight towards a specific target point."""
	dist = get_dist(current, target)
	if dist == 0:
		return current
		
	move_dist = speed * dt
	if move_dist >= dist:
		# snap to target to avoid overshooting or moving forth and back
		return target
		
	dir_x = target.x - current.x
	dir_y = target.y - current.y
	
	new_x = current.x + (dir_x / dist) * move_dist
	new_y = current.y + (dir_y / dist) * move_dist
	return Position(x=new_x, y=new_y)


def move_on_direction(current: Position, angle_rad: float, speed: float, dt: float) -> Position:
	"""Move in a specific direction (vector, made of angle in radians and speed)."""
	move_dist = speed * dt
	new_x = current.x + move_dist * math.cos(angle_rad)
	new_y = current.y + move_dist * math.sin(angle_rad)
	return Position(x=new_x, y=new_y)

def vehicle_move_on_direction(v: Vehicle, dt: float) -> Position:
	"""Move in a specific direction (vector, made of angle in radians and speed)."""
	return move_on_direction(v.pos, v.pos_angle, v.speed, dt)


def move_on_circle(center: Position, radius: float, current_angle: float, speed: float, dt: float) -> tuple[float, Position]:
	"""Move along the perimeter of a circle counter-clockwise."""
	angular_speed	= speed / radius	# v = w*r
	new_angle		= current_angle + (angular_speed * dt)
	
	# keep angle normalized between 0 and 2pi
	new_angle = new_angle % (2 * math.pi)
	
	new_x = center.x + radius * math.cos(new_angle)
	new_y = center.y + radius * math.sin(new_angle)
	
	return new_angle, Position(x=new_x, y=new_y)


def vehicle_move_dist_on_circle(
	v: Vehicle, dist: float, radius: float=ROUNDABOUT_RADIUS
) -> tuple[float, Position]:
	
	if dist <= 0.0:
		return v.pos_angle, v.pos

	new_angle = (v.pos_angle + dist / radius) % (2 * math.pi)
	new_pos = Position(
		x=ROUNDABOUT_POS.x + radius * math.cos(new_angle),
		y=ROUNDABOUT_POS.y + radius * math.sin(new_angle),
	)

	return new_angle, new_pos


def vehicle_ride(
	v: Vehicle, dt: float, new_acc: float|None = None
) -> float:
	"""
	@param new_acc : optional new acceleration to use instead of v's current one
	@return : the distance traveled
	"""
	if dt <= 0.0:
		return 0.0

	acc			= float(v.acceleration if new_acc is None else new_acc)
	speed		= max(float(v.speed), 0.0)
	max_speed	= max(float(v.params.max_speed), 0.0)

	if acc > 0.0 and speed < max_speed:
		time_to_max = (max_speed - speed) / acc

		if dt <= time_to_max:
			return speed * dt + 0.5 * acc * dt ** 2
		else:
			dist_to_max =	speed * time_to_max + 0.5 * acc * time_to_max ** 2
			return			dist_to_max + max_speed * (dt - time_to_max)
	elif acc < 0.0:
		t_to_stop = speed / -acc

		if dt <= t_to_stop:
			return speed * dt + 0.5 * acc * dt ** 2
		else:
			return speed * t_to_stop + 0.5 * acc * t_to_stop ** 2
	else:
		return speed * dt



def vehicle_tta(v, dist: float, new_acc: float|None = None, margin: float = 0.0) -> float:
	"""
	Estimate time-to-arrival for a vehicle with constant acceleration
	and a max-speed cap.

	@param v: vehicle, with current speed, acceleration, max_speed
	@param dist: remaining distance
	@param new_acc: optional new acceleration to use instead of the current one
	@param margin: optional margin, to add to the distance (e.g., vehicle length)
	@return: time in seconds
	"""
	if dist <= 0.0:
		return 0.0

	d		= max(0.0, dist + margin)
	v0		= max(float(v.speed), 0.0)
	vmax	= v.params.max_speed

	acc = float(v.acceleration if new_acc is None else new_acc)
	if abs(acc) < 1e-9:
		if v0 == 0.0:
			return math.inf
		return d / v0

	# distance to accelerate from v0 to vmax
	d_acc = (vmax ** 2 - v0 ** 2) / (2.0 * acc)

	if d <= d_acc:
		return (math.sqrt(v0 ** 2 + 2 * acc * d) - v0) / acc

	t_acc		= 		(vmax - v0)	/ acc
	t_cruise	= max(	(d - d_acc)	/ vmax, 0.0)
	return t_acc + t_cruise



#
# COLLISIONS
#

def _vehicle_predict_conflict(
	v_app		: Vehicle,
	v_in		: Vehicle,
	v_app_acc	: float|None	= None,
	v_in_acc	: float|None	= None
) -> tuple[Vehicle, Vehicle]|None:
	"""
	@param dt : time for v_app to reach the roundabout entry
	@return : the predicted positions of v_app approaching and v_in in roundabout, after dt seconds;
	None in case of error
	"""
	if v_app.nav_state != VehicleNavState.APPROACHING or v_in.nav_state != VehicleNavState.IN_ROUNDABOUT:
		return None

	# tta to the exact point, for following calculations
	v1_tta	= vehicle_tta(
		v_app, math_utils.get_dist(v_app.pos, ROUNDABOUT_POS) - ROUNDABOUT_RADIUS,
		new_acc=v_app_acc, margin=0 )
	
	conflict_angle	= roundabout.get_road_angle(v_app.entry_road)
	# v1 has just reached the roundabout entry
	new_v1				= v_app.model_copy(deep=True)
	new_v1.nav_state	= VehicleNavState.IN_ROUNDABOUT
	new_v1.pos_angle	= conflict_angle
	new_v1.pos			= roundabout.get_entry(v_app.entry_road)
	new_v1.speed		= vehicle_update_speed(v_app, v1_tta, new_acc=v_app_acc)
	new_v1.acceleration = v_app.acceleration if v_app_acc is None else v_app_acc

	# advance v2 for the same amount of time
	new_v2				= v_in.model_copy(deep=True)
	v2_distance			= vehicle_ride(v_in, v1_tta, new_acc=v_in_acc)
	new_v2.pos_angle	= (v_in.pos_angle + v2_distance / ROUNDABOUT_RADIUS) % (2 * math.pi)
	new_v2.pos			= math_utils.get_point_on_circle(new_v2.pos_angle)
	new_v2.speed		= vehicle_update_speed(v_in, v1_tta, new_acc=v_in_acc)
	new_v2.acceleration	= v_in.acceleration if v_in_acc is None else v_in_acc

	return (new_v1, new_v2)


def vehicle_enters_first(
	v1			: Vehicle,
	v2			: Vehicle,
	v1_acc		: float|None	= None,
	v2_acc		: float|None	= None,
	safety_dist	: float			= VEHICLE_SAFETY_MARGIN_M
) -> tuple[Vehicle, Vehicle] | None:
	"""
	If v1 is approachingthe roundabout and v2 is already inside,
	determine if v1 enters before v2 has passed, and predict their future positions.  

	@param v1_acc : optional new acceleration for v1, otherwise use its current one.
	@return :
		(v1_predicted, v2_predicted) if v1 enters first.
		None otherwise.
	"""
	if v1.nav_state != VehicleNavState.APPROACHING or v2.nav_state != VehicleNavState.IN_ROUNDABOUT:
		return None
	
	# times to get to conflict point are more conservative:
	# they also take into account some margins, e.g. for the car size
	conflict_angle		= roundabout.get_road_angle(v1.entry_road)
	v1_dist_to_conflict	= max(0.0, math_utils.get_dist(v1.pos, ROUNDABOUT_POS) - ROUNDABOUT_RADIUS)
	v2_dist_to_conflict	= math_utils.get_dist_on_circle(v2.pos_angle, conflict_angle)
	v1_tta_conflict		= vehicle_tta(v1, v1_dist_to_conflict, new_acc=v1_acc, margin=0)
	v2_tta_conflict		= vehicle_tta(v2, v2_dist_to_conflict, new_acc=v2_acc, margin=-CAR_LENGTH - safety_dist)

	if not math.isfinite(v1_tta_conflict) or not math.isfinite(v2_tta_conflict) or v1_tta_conflict > v2_tta_conflict:
		return None
	
	return _vehicle_predict_conflict(v1, v2, v1_acc, v2_acc)


def vehicle_enters_later(
	v1			: Vehicle,
	v2			: Vehicle,
	v1_acc		: float|None	= None,
	v2_acc		: float|None	= None,
	safety_dist	: float			= VEHICLE_SAFETY_MARGIN_M
) -> tuple[Vehicle, Vehicle]|None:
	"""
	If v1 is approaching and v2 is already in the the roundabout,
	determine if v2 will pass before v1 enters, and predict their future positions.

	@param v1_acc : optional new acceleration for v1, otherwise use its current one.
	@return :
		(v1_predicted, v2_predicted) if v1 arrives later than v2.
		None if v1 does not arrive later.
	"""
	if v1.nav_state != VehicleNavState.APPROACHING or v2.nav_state != VehicleNavState.IN_ROUNDABOUT:
		return None

	# times to get to conflict point are more conservative:
	# they also take into account some margins, e.g. for the car size
	conflict_angle		= roundabout.get_road_angle(v1.entry_road)
	v1_dist_to_conflict	= max(0.0, math_utils.get_dist(v1.pos, ROUNDABOUT_POS) - ROUNDABOUT_RADIUS)
	v2_dist_to_conflict	= math_utils.get_dist_on_circle(v2.pos_angle, conflict_angle)
	v1_tta_conflict		= vehicle_tta(v1, v1_dist_to_conflict, new_acc=v1_acc, margin=0)
	v2_tta_conflict		= vehicle_tta(v2, v2_dist_to_conflict, new_acc=v2_acc, margin=CAR_LENGTH + safety_dist)

	if not math.isfinite(v2_tta_conflict) or math.isfinite(v1_tta_conflict) or v2_tta_conflict > v1_tta_conflict:
		return None
	
	return _vehicle_predict_conflict(v1, v2, v1_acc, v2_acc)



def vehicle_collide(
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
