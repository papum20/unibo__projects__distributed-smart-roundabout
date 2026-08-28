import importlib

from common.models.models import Position, Vehicle
pkg_service_controller	= importlib.import_module("service-controller.main")
evaluate_traffic		= pkg_service_controller.evaluate_traffic



def test_controller_slows_down_tailgating_car():
	# Arrange: Both cars are approaching the center.
	# Car A is at distance 10. Car B is at distance 15 (5 meters behind A).
	car_a = Vehicle(id="A", pos=Position(x=10, y=0), pos_angle=0.0, speed=10.0)
	car_b = Vehicle(id="B", pos=Position(x=15, y=0), pos_angle=0.0, speed=10.0)
	
	# Act: Safe distance is set to 10 meters
	commands = evaluate_traffic(vehicles=[car_a, car_b], safe_distance=10.0)
	
	# Assert
	# Car A has nobody in front of it, it should not brake
	assert commands["A"].target_acceleration == 0.0
	
	# Car B is only 5 meters behind Car A, it MUST brake
	assert commands["B"].target_acceleration < 0.0