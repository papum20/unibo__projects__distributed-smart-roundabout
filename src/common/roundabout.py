import math

from common.const import ROUNDABOUT_N_ROADS, ROUNDABOUT_POS, ROUNDABOUT_RADIUS
from common.models.models import Position



def get_road_angle(
    road_index	: int,
    n_roads		: int	= ROUNDABOUT_N_ROADS,
    lane_offset	: float = 0.0,
) -> float:
	"""
	@return the angle of the road in radians (0 is East, pi/2 is North)
	"""
	road_angle = (2 * math.pi / n_roads) * road_index
	# arc tangent (i.e. angle corresponding to tangent)
	return road_angle + math.atan2(lane_offset, ROUNDABOUT_RADIUS)


def get_point_on_road(
	road_index				: int,
	dist_from_roundabout	: float	= 0.0,
	n_roads					: int	= ROUNDABOUT_N_ROADS,
	lane_offset				: float	= 0.0
) -> Position:
	"""
	@param dist_from_roundabout: distance from the roundabout, i.e. minus its radius. Remember to add ROAD_WIDTH, in case.
	@return the (x, y) coordinates on a specific road at a given distance from the roundabout line.
	"""
	# no lane offset, the angle of the road is the same for both lanes
	angle = get_road_angle(road_index, n_roads)
	# total distance from the absolute center (0,0)
	total_dist = ROUNDABOUT_RADIUS + dist_from_roundabout

	# center of the road
	cx = ROUNDABOUT_POS.x + total_dist * math.cos(angle)
	cy = ROUNDABOUT_POS.y + total_dist * math.sin(angle)
	
	# offset perpendicularly to create lanes
	px = cx - lane_offset * math.sin(angle)
	py = cy + lane_offset * math.cos(angle)
	return Position(x=px, y=py)