import math

from common.const import (
	ROUNDABOUT_POS, ROUNDABOUT_RADIUS
)
from common.models.models import Position
from common.models.vehicle import (
	Vehicle
)
from common.math_utils import get_dist



def v_update_speed(v: Vehicle, dt: float) -> float:
	"""Update speed, preventing it from going below 0 (reversing) or above max_speed."""
	new_speed	= v.speed + (v.acceleration * dt)
	return max(0.0, min(new_speed, v.params.max_speed))



#
# MOVEMENT
#


def v_move_on_circle(
	v: Vehicle, dt: float, radius: float=ROUNDABOUT_RADIUS
) -> tuple[float, Position]:

	dist		= v_ride_dist(v, dt)
	new_angle	= (v.pos_angle + dist / radius) % (2 * math.pi)
	new_pos		= Position(
		x=ROUNDABOUT_POS.x + radius * math.cos(new_angle),
		y=ROUNDABOUT_POS.y + radius * math.sin(new_angle),
	)

	return new_angle, new_pos


def v_move_on_direction(v: Vehicle, dt: float) -> Position:
	"""
	Move in a specific direction (vector, made of angle in radians and speed).
	"""
	dist = v_ride_dist(v, dt)
	return Position(
		x=v.pos.x + dist * math.cos(v.pos_angle),
		y=v.pos.y + dist * math.sin(v.pos_angle),
	)


def v_move_towards(
	v: Vehicle, target: Position, dt: float
) -> Position:
	"""
	Move straight towards a specific target point.

	@return : new pos
	"""
	dist = get_dist(v.pos, target)
	if dist == 0:
		return v.pos

	move_dist = v_ride_dist(v, dt)
	if move_dist >= dist:
		# snap to target to avoid overshooting or moving forth and back
		return target
		
	ratio = move_dist / dist
	return Position(
		x=v.pos.x + (target.x - v.pos.x) * ratio,
		y=v.pos.y + (target.y - v.pos.y) * ratio,
	)


def v_ride_dist(
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
			# avoid backwards movement
			return max(0.0, speed * dt + 0.5 * acc * dt**2)
		return max(0.0, speed * t_to_stop + 0.5 * acc * t_to_stop ** 2)
	else:
		return speed * dt



def v_ride_t(v, dist: float, new_acc: float|None = None, margin: float = 0.0) -> float:
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

	if acc > 0:
		# distance to accelerate from v0 to vmax
		d_acc = (vmax ** 2 - v0 ** 2) / (2.0 * acc)

		if d <= d_acc:
			return (math.sqrt(v0 ** 2 + 2 * acc * d) - v0) / acc

		t_acc		= 		(vmax - v0)	/ acc
		t_cruise	= max(	(d - d_acc)	/ vmax, 0.0)
		return t_acc + t_cruise
	else:
		# acc < 0 (braking)
		d_stop = (v0 ** 2) / (2.0 * -acc)
		if d >= d_stop:
			return math.inf
		# It will reach distance d before stopping completely
		return (v0 - math.sqrt(max(0.0, v0 ** 2 + 2 * acc * d))) / -acc
