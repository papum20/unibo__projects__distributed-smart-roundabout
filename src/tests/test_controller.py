import importlib

from common.models.models import Position
from common.models.vehicle import Vehicle, VehicleNavState, VehicleState
pkg_service_controller = importlib.import_module("service-controller.main")
evaluate = pkg_service_controller.evaluate


def test_controller_slows_down_tailgating_car():
	# Car A is at distance 10. Car B is at distance 15 (5 meters behind A).
	# Both are going 10 m/s. B must brake.
	car_a = Vehicle(id="A", pos=Position(x=10, y=0), pos_angle=0.0, speed=10.0, nav_state=VehicleNavState.APPROACHING)
	car_b = Vehicle(id="B", pos=Position(x=15, y=0), pos_angle=0.0, speed=10.0, nav_state=VehicleNavState.APPROACHING)
	
	commands = evaluate([car_a, car_b], is_deadlock=0)
	
	# Car A has nobody in front of it, it accelerates or maintains
	assert commands["A"].target_acceleration >= 0.0
	# Car B is tailgating, it must brake
	assert commands["B"].target_acceleration < 0.0


def test_controller_ignores_disconnected_ghosts_for_commands():
	car_a = Vehicle(id="A", pos=Position(x=10, y=0), pos_angle=0.0, speed=10.0, state=VehicleState.DISCONNECTED)
	
	commands = evaluate([car_a], is_deadlock=0)
	
	# Controller should not issue commands to disconnected vehicles
	assert "A" not in commands