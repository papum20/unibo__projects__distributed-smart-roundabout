import math

from common import roundabout
from common import math_utils
from common.const import CAR_LENGTH, CAR_WIDTH, ROUNDABOUT_POS, ROUNDABOUT_RADIUS, VEHICLE_SAFETY_MARGIN_M
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



def vehicle_tta(v, dist: float, new_acc: float|None = None, margin: float = 0.0) -> float:
	"""
	Estimate time-to-arrival for a vehicle with constant acceleration
	and a max-speed cap.

	@param v: vehicle, with current speed, acceleration, max_speed
	@param dist: remaining distance
	@param new_acc: optional new acceleration to use instead of the current one
	@param margin: optional margin, to subtract from the distance (e.g., vehicle length)
	@return: time in seconds
	"""
	if dist <= 0.0:
		return 0.0

	d		= max(0.0, dist - margin)
	v0		= max(float(v.speed), 0.0)
	vmax	= v.params.max_speed

	if new_acc is not None:
		acc = float(new_acc)
	else:
		acc = float(v.acceleration)
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



def vehicle_can_enter_safely(
	v1			: Vehicle,
	v2			: Vehicle,
	v1_acc		: float|None	= None,
	v2_acc		: float|None	= None,
	safety_dist	: float			= VEHICLE_SAFETY_MARGIN_M
) -> bool:
	"""
	If v1 is about to enter the roundabout and v2 is already inside,
	determine if v1 can enter safely without crashing into v2 (either immediately or later,
	because v2 may reach v1 later).  
	`s2 = v t / R + 0.5 a t**2 / R`  
	`S1 = V t / R + 0.5 A t**2 / R`  

	`v t / R + 0.5 a t**2 / R + d = V t / R + 0.5 A t**2 / R`  
	`(0.5 * (a - A)) * t**2 + (v - V) * t - d = 0`  
	`t = (- (v - V) + sqrt((v - V)**2 + 2 * (a - A) * d)) / (a - A)`  
	@param v1_acc : optional new acceleration for v1, otherwise use its current one.
	`v1_acc == v2_acc` is illegal, so return False 
	@return : True if v1 can enter safely
	"""
	if v1.nav_state != VehicleNavState.APPROACHING or v2.nav_state != VehicleNavState.IN_ROUNDABOUT:
		return True
	if v1.speed >= v2.speed:
		return True
	
	conflict_angle		= roundabout.get_road_angle(v1.entry_road)
	v2_dist_to_conflict	= math_utils.get_dist_on_circle(v2.pos_angle, conflict_angle)

	v1_acc	= v1_acc if v1_acc is not None else v1.acceleration
	v2_acc	= v2_acc if v2_acc is not None else v2.acceleration
	d		= v2_dist_to_conflict - safety_dist
	if v1_acc == v2_acc:
		return False

	# solve quadratic equation for time t
	a				= 0.5 * (v2_acc - v1_acc)
	b				= v2.speed - v1.speed
	discriminant	= b ** 2 + 4 * a * d
	if discriminant < 0:
		# no solution, v1 will never catch up to v2
		return True
	
	t1 = (-b + math.sqrt(discriminant)) / (2 * a)
	t2 = (-b - math.sqrt(discriminant)) / (2 * a)
	return t1 < 0 and t2 < 0


def vehicle_enters_later(
	v1			: Vehicle,
	v2			: Vehicle,
	v1_acc		: float|None	= None,
	v2_acc		: float|None	= None,
	margin		: float			= VEHICLE_SAFETY_MARGIN_M
) -> bool:
	"""
	If v1 is approaching and v2 is already in the the roundabout,
	determine if v2 will pass before v1 enters.  
	@param v1_acc : optional new acceleration for v1, otherwise use its current one.
	@return : True if v1 enters later.
	"""
	conflict_angle		= roundabout.get_road_angle(v1.entry_road)
	v1_dist_to_conflict	= math_utils.get_dist(v2.pos, ROUNDABOUT_POS) - ROUNDABOUT_RADIUS
	v2_dist_to_conflict	= math_utils.get_dist_on_circle(v2.pos_angle, conflict_angle)
		
	# time to arrival (TTA), considering the straight road part for v1
	v1_tta	= vehicle_tta(
		v1, v1_dist_to_conflict,
		new_acc=v1_acc, margin=CAR_LENGTH
	)
	v2_tta	= vehicle_tta(
		v2, v2_dist_to_conflict,
		new_acc=v2_acc, margin=CAR_LENGTH + margin
	)

	if v2_tta < v1_tta:
		# v2 will pass before v1 arrives
		return True
	return False



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
