import math

from common.const import CAR_LENGTH, CAR_WIDTH
from common.models.models import VEHICLE_DFLT_ACC_BRAKE_M_S2, Position, Vehicle, VehicleNavState, VehiclePosition
from common.math_utils import get_dist



def get_safety_distance(v: Vehicle, margin: float) -> float:
	"""
	Calculate a dynamic safety distance based on the vehicle's speed.
	@param margin: additional safety margin (e.g. half a car length, if calculating it from the car on front)
	"""
	# dynamic safety distance based on speed (1s reaction time)
	safe_dist = (v.speed * 1.0) + CAR_LENGTH / 2.0 + margin
	return max(safe_dist, 2 * CAR_LENGTH)

def get_stop_distance(v: VehiclePosition, max_brake: float = VEHICLE_DFLT_ACC_BRAKE_M_S2) -> float:
	"""
	Calculate the distance required to stop the vehicle, based on its current speed and max braking.
	@return: distance in meters
	"""
	if v.speed <= 0.0:
		return 0.0
	return (v.speed ** 2) / (2 * max_brake)

def get_stop_distance_vehicle(v: Vehicle) -> float:
	"""
	Calculate the distance required to stop the vehicle, based on its current speed and max braking.
	@return: distance in meters
	"""
	return get_stop_distance(
		VehiclePosition(
			id			= v.id,
			pos			= v.pos,
			pos_angle	= v.pos_angle,
			speed		= v.speed,
			nav_state	= v.nav_state
		),
		max_brake = v.params.max_brake
	)

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

	d = max(0.0, dist - margin)

	if new_acc is not None:
		acc = max(float(new_acc), 1e-6)
	else:
		acc = max(float(v.acceleration), 1e-6)

	v0		= max(float(v.speed), 0.0)
	vmax	= v.params.max_speed

	# distance to accelerate from v0 to vmax
	d_acc = (vmax ** 2 - v0 ** 2) / (2.0 * acc)

	if d <= d_acc:
		return (math.sqrt(v0 ** 2 + 2 * acc * d) - v0) / acc

	t_acc		= 		(vmax - v0)	/ acc
	t_cruise	= max(	(d - d_acc)	/ vmax, 0.0)
	return t_acc + t_cruise



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
