import math

from common.models.models import Position
from common.physics import update_speed, move_towards, move_on_circle



def test_update_speed():
    # accelerating
    assert update_speed(speed=5.0, acc=2.0, dt=1.0, max_speed=10.0) == 7.0
    # clamping at max speed
    assert update_speed(speed=9.0, acc=2.0, dt=1.0, max_speed=10.0) == 10.0
    # braking (no reversing allowed)
    assert update_speed(speed=1.0, acc=-3.0, dt=1.0, max_speed=10.0) == 0.0


def test_move_towards():
    pos		= Position(x=0.0, y=0.0)
    target	= Position(x=10.0, y=0.0)
    new_pos = move_towards(pos, target, speed=2.0, dt=1.0)
    assert new_pos.x == 2.0
    assert new_pos.y == 0.0


def test_move_on_circle():
    new_angle, new_pos = move_on_circle(
        center			= Position(x=0.0, y=0.0),
        radius			= 10.0,
        current_angle	= 0.0, 
        speed			= 5.0, 
        dt				= 1.0
    )
    # angular velocity = speed / radius = 5 / 10 = 0.5 rad/s
    assert new_angle == 0.5
    # x = r * cos(theta), y = r * sin(theta)
    assert math.isclose(new_pos.x, 10.0 * math.cos(0.5))
    assert math.isclose(new_pos.y, 10.0 * math.sin(0.5))