import importlib

from common.models.models import Position, Vehicle, VehicleNavState, VehicleState
pkg_service_vehicle = importlib.import_module("service-vehicle.main")
evaluate_failsafe	= pkg_service_vehicle.evaluate_failsafe
reset_vehicle		= pkg_service_vehicle.reset_vehicle



def test_failsafe_triggers_on_timeout():
	vehicle			= Vehicle(id="1", pos=Position(x=10, y=10), pos_angle=0.0, speed=10.0, state=VehicleState.NORMAL)
	current_time	= 100.0
	last_time		= 97.0
	timeout_limit	= 2.0
	
	updated_vehicle, command = evaluate_failsafe(vehicle, current_time, last_time, timeout_limit)
	
	assert updated_vehicle.state == VehicleState.FAILSAFE
	assert command.target_acceleration == -5.0	# max braking



def test_normal_operation_on_healthy_heartbeat():
	vehicle			= Vehicle(id="1", pos=Position(x=10, y=10), pos_angle=0.0, speed=10.0, state=VehicleState.NORMAL)
	current_time	= 100.0
	last_time		= 99.5
	timeout_limit	= 2.0
	
	updated_vehicle, command = evaluate_failsafe(vehicle, current_time, last_time, timeout_limit)
	
	assert updated_vehicle.state == VehicleState.NORMAL
	assert command.target_acceleration == 0.0	# maintain speed



def test_reset_vehicle_clears_state():
	old_vehicle = Vehicle(
		id			= "123", 
		pos			= Position(x=999, y=999), 
		pos_angle	= 0.0,
		speed		= 0.0,
		state		= VehicleState.FAILSAFE,
		nav_state	= VehicleNavState.EXITING,
	)
	
	# respawn it on a 4-road roundabout
	new_vehicle = reset_vehicle(old_vehicle, n_roads=4)
	
	# assert expected values (id unchanged, others reassigned)
	assert new_vehicle.id == "123"
	assert new_vehicle.state == VehicleState.NORMAL
	assert new_vehicle.nav_state == VehicleNavState.APPROACHING
	assert new_vehicle.speed > 0.0
	
	# ensure it picked valid entry/exit roads
	assert 0 <= new_vehicle.entry_road	< 4
	assert 0 <= new_vehicle.exit_road	< 4