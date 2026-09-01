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
	angle_traveled	: float				= 0.0	# traveled distance, to track if we've done a full lap
	nav_state		: VehicleNavState	= VehicleNavState.APPROACHING

	color_hue				: int			= 220	# base car color
	color_lightness_perc	: float			= 50.0	# lightness (for all colors, including failsafe mode)
	params					: VehicleParams = VehicleParams()


class Command(BaseModel):
	target_acceleration: float	# m/s^2

class SystemCommandValue(Enum):
	PAUSE	= "PAUSE"
	RESUME	= "RESUME"

class SystemCommand(BaseModel):
	command: SystemCommandValue