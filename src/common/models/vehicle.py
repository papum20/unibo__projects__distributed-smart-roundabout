from enum import Enum

from pydantic import BaseModel

from common.const import CAR_LENGTH
from common.models.models import Position
import math_utils



VEHICLE_DFLT_MAX_SPEED_M_S		= 50 / 3.6
VEHICLE_DFLT_ACC_M_S2			= 4.5
VEHICLE_DFLT_ACC_BRAKE_MAX_M_S2	= 5.0
VEHICLE_FAILSAFE_MAX_SPEED_M_S	= 30 / 3.6



class VehicleState(Enum):
	NORMAL			= "NORMAL"
	# can't receive commands, but still tries to share its data
	FAILSAFE		= "FAILSAFE"
	# totally unreachable, can't receive nor send
	DISCONNECTED	= "DISCONNECTED"

class VehicleNavState(str, Enum):
	APPROACHING		= "APPROACHING"
	IN_ROUNDABOUT	= "IN_ROUNDABOUT"
	EXITING			= "EXITING"			# moving outwards

class VehicleParams(BaseModel):
	max_speed: float = VEHICLE_DFLT_MAX_SPEED_M_S
	max_accel: float = VEHICLE_DFLT_ACC_M_S2
	max_brake: float = VEHICLE_DFLT_ACC_BRAKE_MAX_M_S2


class VehicleCollision(BaseModel):
	v1_id		: str
	v2_id		: str
	timestamp	: float



class VehiclePosition(BaseModel):
	id			: str
	pos			: Position
	pos_angle	: float
	speed		: float
	nav_state	: VehicleNavState
	timestamp	: float | None = None

	def get_stop_dist(self, acc_brake: float = VEHICLE_DFLT_ACC_BRAKE_MAX_M_S2) -> float:
		"""
		Calculate the distance required to stop the vehicle, based on its current speed and max braking.
		@return: distance in meters
		"""
		if self.speed <= 0.0:
			return 0.0
		return (self.speed ** 2) / (2 * acc_brake)



class Vehicle(BaseModel):
	id				: str
	pos				: Position
	# angle from the center's point of view
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

	def to_pos(self) -> VehiclePosition:
		return VehiclePosition(
			id			= self.id,
			pos			= self.pos,
			pos_angle	= self.pos_angle,
			speed		= self.speed,
			nav_state	= self.nav_state
		)

	def get_acc_brake(self) -> float:
		"""
		@return : the braking acceleration used by a vehicle for a common or soft brake
		"""
		return self.params.max_brake * 0.5


	def get_safety_dist(self, margin: float) -> float:
		"""
		Calculate a dynamic safety distance based on the vehicle's speed.
		@param margin: additional safety margin (e.g. half a car length, if calculating it from the car on front)
		"""
		# dynamic safety distance based on speed (1s reaction time)
		safe_dist = (self.speed * 1.0) + CAR_LENGTH / 2.0 + margin
		return max(safe_dist, 2 * CAR_LENGTH)

	def get_stop_dist(self, acc_brake: float|None = None) -> float:
		"""
		Calculate the distance required to stop the vehicle, based on its current speed and max braking.
		@param acc_brake: optional braking acceleration to use, otherwise use the vehicle's default max braking.
		@return: distance in meters
		"""
		acc_brake = acc_brake if acc_brake is not None else self.params.max_brake
		return self.to_pos().get_stop_dist(acc_brake = acc_brake)

	def get_stop_behind_margin(
			self, v2: VehiclePosition, v1_acc_brake: float|None = None, v2_acc_brake: float = VEHICLE_DFLT_ACC_BRAKE_MAX_M_S2
	) -> float:
		"""
		Check if this vehicle can stop behind another vehicle, i.e. if it could stop without crashing into it
		if they were to both start to brake (both for straight line and circle).  
		@param v2: the other vehicle's position
		@return: the available margin before it's to late to be able to stop behind the other vehicle
		"""
		v1_acc_brake	= v1_acc_brake if v1_acc_brake is not None else self.params.max_brake
		if self.nav_state != v2.nav_state:
			return True
		elif self.nav_state == VehicleNavState.IN_ROUNDABOUT:
			dist_to_v2		= math_utils.get_dist_on_circle(self.pos_angle, v2.pos_angle) - CAR_LENGTH
		else:
			dist_to_v2		= math_utils.get_dist(self.pos, v2.pos) - CAR_LENGTH
		v1_stop_dist	= self.get_stop_dist(	acc_brake = v1_acc_brake )
		v2_stop_dist	= v2.get_stop_dist(		acc_brake = v2_acc_brake )
		return v1_stop_dist + v2_stop_dist - dist_to_v2
