#
# The x,y coordinate system starts from the bottom left corner.
#

import math

from common.const import (
	ROUNDABOUT_POS,
	ROUNDABOUT_RADIUS
)
from common.models.models import Position



def get_dist(pos1: Position, pos2: Position) -> float:
	return math.sqrt((pos1.x - pos2.x)**2 + (pos1.y - pos2.y)**2)


def get_dist_on_circle(angle1: float, angle2: float, radius: float = ROUNDABOUT_RADIUS) -> float:
	"""
	Note that the angle is calculated counter-clockwise, so the result is always the positive distance
	from angle1 to angle2 (in modulo).
	@param angle1: angle of the first point, in radians
	@param angle2: angle of the second point, in radians
	@param radius: radius of the circle, in meters
	@return: distance between the two points on the perimeter of a circle with the given radius, in meters
	"""
	angle_diff = (angle2 - angle1) % (2 * math.pi)
	return angle_diff * radius



#
# MOVEMENT (VEHICLE)
#

def move(pos_start: Position, speed: float, dt: float) -> Position:
	"""
	@param pos_start: starting position
	@param speed: speed in m/s
	@param dt: time delta in seconds
	@return: new position after moving at the given speed, for the given time delta,
	towards the center of the roundabout (negative speed means moving away)
	"""
	move_dist		= speed * dt
	dist_to_center	= get_dist(pos_start, ROUNDABOUT_POS)

	if dist_to_center == 0:
		# already at the center, cannot move towards it
		return pos_start
	
	# vector(center-car)
	dir_x = ROUNDABOUT_POS.x - pos_start.x
	dir_y = ROUNDABOUT_POS.y - pos_start.y

	# 2. Normalize the vector (make its length 1) and multiply by move_dist
	move_x = (dir_x / dist_to_center) * move_dist
	move_y = (dir_y / dist_to_center) * move_dist
		
	return Position(x=pos_start.x + move_x, y=pos_start.y + move_y)



#
# ROUNDABOUT
#

def get_dist_to_roundabout(pos: Position) -> float:
	"""
	@return: distance from the vehicle to enter the roundabout (furthest point from center), in meters
	"""
	dist_to_center		= get_dist(pos, ROUNDABOUT_POS)
	dist_to_boundary	= dist_to_center - ROUNDABOUT_RADIUS

	print(dist_to_center)
	print(dist_to_boundary)

	if dist_to_boundary <= 0:
		# already inside
		return 0.0
		
	return dist_to_boundary


def is_in_roundabout(pos: Position) -> bool:
	return get_dist_to_roundabout(pos) <= 0.0
