import math

from common.models.models import Position
from common.models.vehicle import Vehicle, VehicleState, VehicleNavState, VehicleParams
from common.physics import v_update_speed, v_ride_dist, v_ride_t, v_move_towards


def create_dummy_vehicle(speed=10.0, acc=0.0):
	return Vehicle(
		id="test",
		pos=Position(x=0, y=0),
		pos_angle=0.0,
		speed=speed,
		acceleration=acc,
		state=VehicleState.NORMAL,
		nav_state=VehicleNavState.APPROACHING,
		# Set a high max speed so tests don't unexpectedly cap the speed
		params=VehicleParams(max_speed=100.0, max_accel=10.0, max_brake=10.0)
	)

def test_v_update_speed():
	v = create_dummy_vehicle(speed=10.0, acc=2.0)
	new_speed = v_update_speed(v, dt=1.0)
	assert new_speed == 12.0
	
	# Test braking
	v.acceleration = -5.0
	assert v_update_speed(v, dt=1.0) == 5.0

	# Test complete stop (no reverse)
	assert v_update_speed(v, dt=3.0) == 0.0


def test_v_ride_dist():
	# Constant speed
	v = create_dummy_vehicle(speed=10.0, acc=0.0)
	assert v_ride_dist(v, dt=2.0) == 20.0

	# Accelerating: d = vt + 0.5 * a * t^2 -> 10*2 + 0.5*2*4 = 24
	v.acceleration = 2.0
	assert v_ride_dist(v, dt=2.0) == 24.0

	# Braking to a halt: v=10, a=-5. Takes 2s to stop. Distance = 10
	v.acceleration = -5.0
	assert v_ride_dist(v, dt=3.0) == 10.0 


def test_v_ride_t():
	v = create_dummy_vehicle(speed=10.0, acc=0.0)
	# At 10m/s, takes 2 seconds to travel 20 meters
	assert v_ride_t(v, dist=20.0) == 2.0

	# Braking vehicle: v=10, a=-2. TTA for 16 meters.
	# 16 = 10t - t^2 -> t=2.
	v.acceleration = -2.0
	assert math.isclose(v_ride_t(v, dist=16.0), 2.0)

	# Braking vehicle that will never reach the target
	assert v_ride_t(v, dist=30.0) == math.inf


def test_v_move_towards():
	v		= create_dummy_vehicle(speed=5.0, acc=0.0)
	v.pos	= Position(x=0, y=0)
	target	= Position(x=10, y=0)
	
	new_pos = v_move_towards(v, target, dt=1.0)
	assert new_pos.x == 5.0
	assert new_pos.y == 0.0
	
	# Overshoot prevention: moving 15 meters when target is only 10m away
	new_pos2 = v_move_towards(v, target, dt=3.0) 
	assert new_pos2.x == 10.0	# snapped to target