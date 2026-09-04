import logging
import math

from common import math_utils, physics, roundabout
from common.const import (
	LANE_WIDTH,
	ROUNDABOUT_N_ROADS,
	ROUNDABOUT_POS,
	ROUNDABOUT_RADIUS,
	VEHICLE_ANGLE_TOL_RAD,
	VEHICLE_ANGLE_TRAVELED_MIN_RAD,
	VEHICLE_DIST_TOL,
	VEHICLE_SAFETY_MARGIN_M,
)
from common.models.models import (
	Command,
	Position
)
from common.models.vehicle import (
	Vehicle, VehicleNavState, VehiclePosition
)



def vehicle_navigate(dt: float, v: Vehicle, logger: logging.Logger|None = None) -> Vehicle:
	if v.speed == 0 and v.acceleration <= 0:
		# stopped, do not move
		return v

	if v.nav_state == VehicleNavState.APPROACHING:
		# drive on the right side of the road (positive offset)
		target_x, target_y	= roundabout.get_point_on_road(
			v.entry_road, distance_from_boundary=0.0, lane_offset=LANE_WIDTH/2
		)
		entry_target		= Position(x=target_x, y=target_y)
		
		v.pos = physics.move_towards(v.pos, entry_target, v.speed, dt)
		
		if math_utils.get_dist(v.pos, entry_target) <= VEHICLE_DIST_TOL:
			v.nav_state			= VehicleNavState.IN_ROUNDABOUT
			v.pos_angle			= roundabout.get_road_angle(v.entry_road)
			v.angle_traveled	= 0.0
			if logger is not None:
				logger.info("Vehicle %s entered the roundabout (IN_ROUNDABOUT) on road %d", v.id, v.entry_road)
			
	elif v.nav_state == VehicleNavState.IN_ROUNDABOUT:
		old_angle	= v.pos_angle

		v.pos_angle, v.pos = physics.move_on_circle(
			ROUNDABOUT_POS, ROUNDABOUT_RADIUS, v.pos_angle, v.speed, dt
		)

		angle_diff_frame = (v.pos_angle - old_angle) % (2 * math.pi)
		v.angle_traveled += angle_diff_frame
		
		# check if we reached the exit road
		exit_angle = roundabout.get_road_angle(v.exit_road)
		angle_diff = abs(v.pos_angle - exit_angle)
		# handle wrap-around at 2*pi
		angle_diff = min(angle_diff, 2*math.pi - angle_diff) 

		# make sure to not exit immediately (if entry==exit)
		if angle_diff < VEHICLE_ANGLE_TOL_RAD and v.angle_traveled > VEHICLE_ANGLE_TRAVELED_MIN_RAD:
			v.nav_state		= VehicleNavState.EXITING
			# snap angle, or won't drive straight
			v.pos_angle		= exit_angle
			# snap position to correct lane
			exit_x, exit_y	= roundabout.get_point_on_road(
				v.exit_road, distance_from_boundary=0.0, lane_offset=-LANE_WIDTH / 2
			)
			v.pos = Position(x=exit_x, y=exit_y)
			if logger is not None:
				logger.info("Vehicle %s is exiting the roundabout (EXITING) on road %d", v.id, v.exit_road)

	elif v.nav_state == VehicleNavState.EXITING:
		v.pos = physics.vehicle_move_on_direction(v, dt)
		
	return v



def get_predicted_entry(v_pos: VehiclePosition) -> int:
	"""
	Predict the entry road in the safest way possible (worst case scenario).
	@return: the entry road index
	"""
	angle_per_road	= (2 * math.pi) / ROUNDABOUT_N_ROADS

	if v_pos.nav_state == VehicleNavState.APPROACHING:
		# closest road based on current angle
		return round(v_pos.pos_angle / angle_per_road) % ROUNDABOUT_N_ROADS
	else:
		return 0	# doesn't matter


def get_predicted_exit(v_pos: VehiclePosition) -> int:
	"""
	Predict the exit road in the safest way possible (worst case scenario).
	@return: the exit road index
	"""
	angle_per_road	= (2 * math.pi) / ROUNDABOUT_N_ROADS

	if v_pos.nav_state == VehicleNavState.IN_ROUNDABOUT:
		# worst-case scenario: it exits at the furthest possible road (the one just passed)
		# int() acts as floor()
		return int((v_pos.pos_angle - 1) / angle_per_road) % ROUNDABOUT_N_ROADS
	elif v_pos.nav_state == VehicleNavState.EXITING:
		# closest road based on current angle
		return round(v_pos.pos_angle / angle_per_road) % ROUNDABOUT_N_ROADS
	else:
		return 0	# doesn't matter



def evaluate_safely(v1: Vehicle, v_others: list[VehiclePosition]) -> Command:
	""" 
	Check if the vehicle should slow down to avoid crashing in the one in front.
	This is an additional layer of ssafety to the controller.
	@return : the max acceleration that can be kept safely
	"""
	new_acc = v1.params.max_accel

	for v2 in v_others:
		safety_margin_soft	= v1.get_stop_behind_margin(v2, v1_acc_brake=v1.get_acc_brake())
		safety_margin_hard	= v1.get_stop_behind_margin(v2, v1_acc_brake=v1.params.max_brake)
		if safety_margin_soft >= VEHICLE_SAFETY_MARGIN_M:
			# plenty of margin
			continue
		elif safety_margin_hard >= VEHICLE_SAFETY_MARGIN_M:
			# start slowing down, but we can still brake hard in case of emergency
			new_acc = min(new_acc, -v1.get_acc_brake())
		else:
			# start braking hard immediately, even if it could be too late
			new_acc = min(new_acc, -v1.params.max_brake)
			break

	return Command(target_acceleration=new_acc)
