import math

from common.models.models import Position
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


def move_on_circle(center: Position, radius: float, current_angle: float, speed: float, dt: float) -> tuple[float, Position]:
    """Move along the perimeter of a circle counter-clockwise."""
    angular_speed	= speed / radius	# v = w*r
    new_angle		= current_angle + (angular_speed * dt)
    
    # keep angle normalized between 0 and 2pi
    new_angle = new_angle % (2 * math.pi)
    
    new_x = center.x + radius * math.cos(new_angle)
    new_y = center.y + radius * math.sin(new_angle)
    
    return new_angle, Position(x=new_x, y=new_y)