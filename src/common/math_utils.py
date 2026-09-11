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


def get_point_on_circle(angle: float, radius: float = ROUNDABOUT_RADIUS) -> Position:
	"""
	@return: the coordinates of a point on a circle
	"""
	return Position(
		x=ROUNDABOUT_POS.x + radius * math.cos(angle),
		y=ROUNDABOUT_POS.y + radius * math.sin(angle),
	)


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
# ROUNDABOUT
#

def get_dist_to_roundabout(pos: Position) -> float:
	"""
	@return: distance (>=0) from the vehicle to enter the roundabout (furthest point from center), in meters
	"""
	dist = get_dist(pos, ROUNDABOUT_POS) - ROUNDABOUT_RADIUS
	if dist <= 0:
		# already inside
		return 0.0
	return dist


def is_in_roundabout(pos: Position) -> bool:
	return get_dist_to_roundabout(pos) <= 0.0
