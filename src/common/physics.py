import math

from common.models.models import Position, Vehicle
from common.math_utils import get_dist



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


def vehicle_tta(v, dist: float) -> float:
	"""
	Estimate time-to-arrival for a vehicle with constant acceleration
	and a max-speed cap.

	@param v: vehicle, with current speed, acceleration, max_speed
	@param dist: remaining distance
	@return: time in seconds
	"""
	if dist <= 0.0:
		return 0.0

	v0		= max(float(v.speed), 0.0)
	acc		= max(float(v.acceleration), 1e-6)
	vmax	= v.params.max_speed

	# distance to accelerate from v0 to vmax
	d_acc = (vmax ** 2 - v0 ** 2) / (2.0 * acc)

	if dist <= d_acc:
		return (math.sqrt(v0 ** 2 + 2 * acc * dist) - v0) / acc

	t_acc		= (vmax - v0)		/ acc
	t_cruise	= (dist - d_acc)	/ vmax
	return t_acc + t_cruise
