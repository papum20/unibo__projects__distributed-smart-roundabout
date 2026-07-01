# tests/test_math_utils.py
import math

from  common.const import (
	ROUNDABOUT_POS,
	ROUNDABOUT_RADIUS
)
from common.math_utils import (
	get_dist,
	get_dist_to_roundabout,
	is_in_roundabout,
	move
)
from common.models.models import Position



def test_is_in_roundabout():

	# inside
	pi1 = Position(x=ROUNDABOUT_POS.x + ROUNDABOUT_RADIUS / 2, y=ROUNDABOUT_POS.y + ROUNDABOUT_RADIUS / 2)
	# outside
	po1 = Position(x=ROUNDABOUT_POS.x + ROUNDABOUT_RADIUS * 2, y=ROUNDABOUT_POS.y + ROUNDABOUT_RADIUS * 3)
	po2 = Position(x=ROUNDABOUT_POS.x + ROUNDABOUT_RADIUS + 1, y=ROUNDABOUT_POS.y + ROUNDABOUT_RADIUS + 1)
	po3 = Position(x=ROUNDABOUT_POS.x + ROUNDABOUT_RADIUS - 1, y=ROUNDABOUT_POS.y + ROUNDABOUT_RADIUS - 1)
	# edge
	pe4 = Position(x=ROUNDABOUT_POS.x + ROUNDABOUT_RADIUS, y=ROUNDABOUT_POS.y)
	
	assert is_in_roundabout(pi1) is True
	assert is_in_roundabout(po1) is False
	assert is_in_roundabout(po2) is False
	assert is_in_roundabout(po3) is False
	assert is_in_roundabout(pe4) is True



def test_dist_to_roundabout():

	# horizontal
	ph1 = Position(x=ROUNDABOUT_POS.x + ROUNDABOUT_RADIUS + 5, y=ROUNDABOUT_POS.y)
	ph2 = Position(x=ROUNDABOUT_POS.x - ROUNDABOUT_RADIUS - 3, y=ROUNDABOUT_POS.y)
	# vertical
	pv1 = Position(x=ROUNDABOUT_POS.x, y=ROUNDABOUT_POS.y + ROUNDABOUT_RADIUS + 2)
	pv2 = Position(x=ROUNDABOUT_POS.x, y=ROUNDABOUT_POS.y - ROUNDABOUT_RADIUS - 4)
	# diagonal
	pd1 = Position(x=ROUNDABOUT_POS.x + ROUNDABOUT_RADIUS + 3, y=ROUNDABOUT_POS.y + ROUNDABOUT_RADIUS + 4)
	pd2 = Position(x=ROUNDABOUT_POS.x - ROUNDABOUT_RADIUS - 5, y=ROUNDABOUT_POS.y - ROUNDABOUT_RADIUS - 6)
	pd3 = Position(x=ROUNDABOUT_POS.x + ROUNDABOUT_RADIUS + 7, y=ROUNDABOUT_POS.y - ROUNDABOUT_RADIUS - 8)
	pd4 = Position(x=ROUNDABOUT_POS.x - ROUNDABOUT_RADIUS - 9, y=ROUNDABOUT_POS.y + ROUNDABOUT_RADIUS + 10)
	# boundary
	pb1 = Position(x=ROUNDABOUT_POS.x + ROUNDABOUT_RADIUS, y=ROUNDABOUT_POS.y)
	# inside
	pi1 = Position(x=ROUNDABOUT_POS.x, y=ROUNDABOUT_POS.y)

	dist_h1 = get_dist_to_roundabout(ph1)
	assert dist_h1 == ph1.x - (ROUNDABOUT_POS.x + ROUNDABOUT_RADIUS)
	dist_h2 = get_dist_to_roundabout(ph2)
	assert dist_h2 == (ROUNDABOUT_POS.x - ROUNDABOUT_RADIUS) - ph2.x

	dist_v1 = get_dist_to_roundabout(pv1)
	assert dist_v1 == pv1.y - (ROUNDABOUT_POS.y + ROUNDABOUT_RADIUS)
	dist_v2 = get_dist_to_roundabout(pv2)
	assert dist_v2 == (ROUNDABOUT_POS.y - ROUNDABOUT_RADIUS) - pv2.y

	dist_d1 = get_dist_to_roundabout(pd1)
	assert dist_d1 + ROUNDABOUT_RADIUS == get_dist(pd1, ROUNDABOUT_POS)
	dist_d2 = get_dist_to_roundabout(pd2)
	assert dist_d2 + ROUNDABOUT_RADIUS == get_dist(pd2, ROUNDABOUT_POS)
	dist_d3 = get_dist_to_roundabout(pd3)
	assert dist_d3 + ROUNDABOUT_RADIUS == get_dist(pd3, ROUNDABOUT_POS)
	dist_d4 = get_dist_to_roundabout(pd4)
	assert dist_d4 + ROUNDABOUT_RADIUS == get_dist(pd4, ROUNDABOUT_POS)

	dist_b1 = get_dist_to_roundabout(pb1)
	assert dist_b1 == 0.0

	dist_i1 = get_dist_to_roundabout(pi1)
	assert dist_i1 == 0.0

	