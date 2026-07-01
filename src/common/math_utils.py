#
# The x,y coordinate system goes starts from the bottom left corner.
#

import math

from common.const import (
	ROUNDABOUT_POS,
	ROUNDABOUT_RADIUS
)
from common.models.models import Position



def get_dist(pos1: Position, pos2: Position) -> float:
    return math.sqrt((pos1.x - pos2.x)**2 + (pos1.y - pos2.y)**2)



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
	
	if pos_start.x == ROUNDABOUT_POS.x:
		# moving vertically
		move_x = 0.0
		move_y = - move_dist if pos_start.y > ROUNDABOUT_POS.y else move_dist

	elif pos_start.y == ROUNDABOUT_POS.y:
		# moving horizontally
		move_y = 0.0
		move_x = - move_dist if pos_start.x > ROUNDABOUT_POS.x else move_dist

	else:
		delta_x	= pos_start.x - ROUNDABOUT_POS.x
		delta_y	= pos_start.y - ROUNDABOUT_POS.y

		# Triangle formed by the vector going from the vehicle to the center of the roundabout.
		# Considering the angle at the center.
		center_angle_cos	= dist_to_center / math.fabs(delta_x)
		center_angle_sin	= dist_to_center / math.fabs(delta_y)

		move_x	= - move_dist * center_angle_cos if pos_start.x > ROUNDABOUT_POS.x else move_dist * center_angle_cos
		move_y	= - move_dist * center_angle_sin if pos_start.y > ROUNDABOUT_POS.y else move_dist * center_angle_sin
		
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
