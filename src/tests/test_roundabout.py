import math

from common.const import ROUNDABOUT_RADIUS
from common.roundabout import get_road_angle, get_point_on_road



def test_get_road_angle():
	assert get_road_angle(road_index=0, n_roads=4) == 0.0
	assert get_road_angle(road_index=1, n_roads=4) == math.pi / 2
	assert get_road_angle(road_index=2, n_roads=4) == math.pi
	assert get_road_angle(road_index=3, n_roads=4) == 3 * math.pi / 2


def test_get_point_on_road():
	pos = get_point_on_road(road_index=0, dist_from_roundabout=10.0, n_roads=4)
	assert pos.x == 10.0 + ROUNDABOUT_RADIUS
	assert pos.y == 0.0

	# road 1 (North)
	pos = get_point_on_road(road_index=1, dist_from_roundabout=10.0, n_roads=4)
	assert math.isclose(pos.x, 0.0,							abs_tol=1e-9)
	assert math.isclose(pos.y, 10.0 + ROUNDABOUT_RADIUS,	abs_tol=1e-9)