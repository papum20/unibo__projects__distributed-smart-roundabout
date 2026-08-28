import math

from common.const import ROUNDABOUT_N_ROADS, ROUNDABOUT_POS, ROUNDABOUT_RADIUS



def get_road_angle(road_index: int, n_roads: int = ROUNDABOUT_N_ROADS) -> float:
	"""
	@return the angle of the road in radians (0 is East, pi/2 is North)
	"""
	return (2 * math.pi / n_roads) * road_index
	

def get_point_on_road(road_index: int, distance_from_boundary: float, n_roads: int = ROUNDABOUT_N_ROADS) -> tuple[float, float]:
	"""
	@return the (x, y) coordinates on a specific road at a given distance from the roundabout line.
	"""
	angle = get_road_angle(road_index, n_roads)
	# total distance from the absolute center (0,0)
	total_dist = ROUNDABOUT_RADIUS + distance_from_boundary
	
	x = ROUNDABOUT_POS.x + total_dist * math.cos(angle)
	y = ROUNDABOUT_POS.y + total_dist * math.sin(angle)
	return x, y
