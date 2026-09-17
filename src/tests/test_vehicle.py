import importlib

from common.models.models import Position
from common.models.vehicle import Vehicle, VehicleNavState, VehicleState, VehiclePosition
from common.vehicle import evaluate_safely, v_collide

pkg_service_vehicle = importlib.import_module("service-vehicle.logic")
evaluate_failsafe	= pkg_service_vehicle.evaluate_failsafe
vehicle_reset		= pkg_service_vehicle.vehicle_reset


def test_reset_vehicle_clears_state():
	v = Vehicle(
		id="123", 
		pos=Position(x=999, y=999), 
		pos_angle=0.0, speed=0.0,
		state=VehicleState.FAILSAFE, nav_state=VehicleNavState.EXITING,
	)
	
	# Mutates v in place
	vehicle_reset(v, n_roads=4)
	
	assert v.id == "123"
	assert v.state == VehicleState.NORMAL
	assert v.nav_state == VehicleNavState.APPROACHING
	assert v.speed > 0.0
	assert 0 <= v.entry_road < 4


def test_v_collide_rectangles():
	# Two vehicles in the exact same spot should collide
	v1 = VehiclePosition(id="1", pos=Position(x=0, y=0), pos_angle=0.0, speed=0.0, nav_state=VehicleNavState.APPROACHING)
	v2 = VehiclePosition(id="2", pos=Position(x=0, y=0), pos_angle=0.0, speed=0.0, nav_state=VehicleNavState.APPROACHING)
	assert v_collide(v1, v2) is True

	# Move v2 completely out of the way
	v2.pos = Position(x=10, y=10)
	assert v_collide(v1, v2) is False


def test_failsafe_brakes_for_tailgating():
	# Failsafe vehicle should brake if someone is in front of it on the same road
	v1 = Vehicle(id="V1", pos=Position(x=100, y=0), pos_angle=0.0, speed=10.0, nav_state=VehicleNavState.APPROACHING)
	v2_pos = VehiclePosition(id="V2", pos=Position(x=90, y=0), pos_angle=0.0, speed=10.0, nav_state=VehicleNavState.APPROACHING)
	
	cmd = evaluate_failsafe(v1, [v2_pos])
	# Must issue a braking command
	assert cmd.target_acceleration < 0.0


def test_evaluate_safely_clear_road():
	v1 = Vehicle(id="V1", pos=Position(x=100, y=0), pos_angle=0.0, speed=10.0, nav_state=VehicleNavState.APPROACHING)
	# V2 is far away on another road
	v2_pos = VehiclePosition(id="V2", pos=Position(x=0, y=100), pos_angle=1.57, speed=10.0, nav_state=VehicleNavState.APPROACHING)

	cmd = evaluate_safely(v1, [v2_pos])
	# No cars in front, it should return the maximum allowed acceleration
	assert cmd is not None
	assert cmd.target_acceleration == v1.params.max_accel