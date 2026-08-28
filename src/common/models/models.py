from enum import Enum

from pydantic import BaseModel



VEHICLE_DFLT_MAX_SPEED_M_S	= 10.0
VEHICLE_DFLT_ACC_M_S2		= 3.0
VEHICLE_DFLT_ACC_BRAKE_M_S2	= 5.0



class Position(BaseModel):
	x	: float		# m
	y	: float		# m


class VehicleState(Enum):
	NORMAL		= "NORMAL"
	FAILSAFE	= "FAILSAFE"

class VehicleNavState(str, Enum):
	APPROACHING		= "APPROACHING"
	IN_ROUNDABOUT	= "IN_ROUNDABOUT"
	EXITING			= "EXITING"			# moving outwards

class VehicleParams(BaseModel):
	max_speed: float = VEHICLE_DFLT_MAX_SPEED_M_S
	max_accel: float = VEHICLE_DFLT_ACC_M_S2
	max_brake: float = VEHICLE_DFLT_ACC_BRAKE_M_S2

class Vehicle(BaseModel):
	id				: str
	pos				: Position
	pos_angle		: float						# rad
	speed			: float						# m/s
	acceleration	: float				= 0.0	# m/s^2
	state			: VehicleState		= VehicleState.NORMAL

	entry_road		: int				= 0
	exit_road		: int				= 0
	nav_state		: VehicleNavState	= VehicleNavState.APPROACHING

	params			: VehicleParams = VehicleParams()


class Command(BaseModel):
	target_acceleration: float	# m/s^2
	