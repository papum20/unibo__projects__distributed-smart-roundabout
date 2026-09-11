from enum import Enum
import math

from pydantic import BaseModel

from common import math_utils, roundabout
from common.const import CAR_LENGTH, ROUNDABOUT_N_ROADS, ROUNDABOUT_POS, VEHICLE_ANGLE_TOL_RAD, VEHICLE_REACTION_TIME_S
from common.models.models import Position



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



class AbstractVehicle(BaseModel):
	id				: str
	pos				: Position
	# angle from the center's point of view
	pos_angle		: float						# rad
	speed			: float						# m/s
	nav_state		: VehicleNavState	= VehicleNavState.APPROACHING

	def get_reaction_time_dist(self) -> float:
		"""
		Calculate the distance traveled during the vehicle's reaction time.

		@return: distance in meters
		"""
		return self.speed * VEHICLE_REACTION_TIME_S

	def get_safety_dist(self, margin: float) -> float:
		"""
		Calculate a dynamic safety distance based on the vehicle's speed.

		@param margin: additional safety margin (e.g. half a car length, if calculating it from the car on front)
		"""
		# dynamic safety distance based on speed (1s reaction time)
		safe_dist = (self.speed * 1.0) + CAR_LENGTH / 2.0 + margin
		return max(safe_dist, 2 * CAR_LENGTH)

	def get_stop_dist(self, acc_brake: float = VEHICLE_DFLT_ACC_BRAKE_MAX_M_S2) -> float:
		"""
		Calculate the distance required to stop the vehicle, based on its current speed and max braking.

		@param acc_brake: optional braking acceleration to use, otherwise use the vehicle's default max braking.
		@return: distance in meters
		"""
		if self.speed <= 0.0:
			return 0.0
		return abs((self.speed ** 2) / (2 * acc_brake))



class VehiclePosition(AbstractVehicle):
	timestamp	: float | None = None



class Vehicle(AbstractVehicle):
	acceleration	: float				= 0.0	# m/s^2
	state			: VehicleState		= VehicleState.NORMAL

	entry_road		: int				= 0
	exit_road		: int				= 0
	angle_traveled	: float				= 0.0	# traveled distance, to track if we've done a full lap

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

	def get_dist_from_entry(self) -> float:
		"""
		@return: distance from the entry road, while inside the roundabout; inf if not inside
		"""
		if self.nav_state != VehicleNavState.IN_ROUNDABOUT:
			return float("inf")
		entry_angle = roundabout.get_road_angle(self.entry_road)
		return math_utils.get_dist_on_circle(entry_angle, self.pos_angle)

	def get_dist_to_exit(self) -> float:
		"""
		@return: distance to the exit road, while inside the roundabout; inf if not inside
		"""
		if self.nav_state != VehicleNavState.IN_ROUNDABOUT:
			return float("inf")
		exit_angle = roundabout.get_road_angle(self.exit_road)
		return math_utils.get_dist_on_circle(self.pos_angle, exit_angle)


	def has_passed_road(self, road: int) -> bool:
		"""
		@return : True if the vehicle has passed road intersection (while inside the roundabout)
		"""
		if self.nav_state != VehicleNavState.IN_ROUNDABOUT:
			return False
		angle_diff = (self.pos_angle - roundabout.get_road_angle(road)) % (2 * math.pi)
		return self.angle_traveled >= angle_diff

	def is_exiting_next(self) -> bool:
		"""
		@return : True if the vehicle is exiting at the next road
		"""
		return self.get_dist_to_exit() < 2 * math.pi / ROUNDABOUT_N_ROADS

	def is_on_same_road(self, v2: VehiclePosition) -> bool:
		"""
		@return : True if v2 is on the same road (while either approaching or exiting)
		"""
		road_angle	= roundabout.get_road_angle(
			self.entry_road	if self.nav_state == VehicleNavState.APPROACHING
			else self.exit_road )
		angle_diff	= (v2.pos_angle - road_angle) % (2 * math.pi)
		return min(angle_diff, 2 * math.pi - angle_diff) < VEHICLE_ANGLE_TOL_RAD


	def get_stop_behind_margin(
			self, v2: VehiclePosition,
			v1_acc_brake: float|None = None, v2_acc_brake: float = VEHICLE_DFLT_ACC_BRAKE_MAX_M_S2
	) -> float:
		"""
		Check if this vehicle can stop behind another vehicle, i.e. if it could stop without crashing into it
		if they were to both start to brake (both for straight line and circle).  
		
		@param v2: the other vehicle's position
		@return: the available margin before it's to late to be able to stop behind the other vehicle
		"""
		v1_acc_brake	= v1_acc_brake if v1_acc_brake is not None else self.params.max_brake
		if self.nav_state != v2.nav_state:
			return float("inf")
		elif self.nav_state == VehicleNavState.IN_ROUNDABOUT:
			dist_to_v2	= math_utils.get_dist_on_circle(self.pos_angle, v2.pos_angle) - CAR_LENGTH
		else:
			if not self.is_on_same_road(v2):
				return float("inf")
			v1_dist = math_utils.get_dist(self.pos, ROUNDABOUT_POS)
			v2_dist = math_utils.get_dist(v2.pos, 	ROUNDABOUT_POS)
			if (
				(self.nav_state == VehicleNavState.APPROACHING	and v2_dist >= v1_dist) or
				(self.nav_state == VehicleNavState.EXITING		and v2_dist <= v1_dist)
			):
				return float("inf")
			dist_to_v2	= math_utils.get_dist(self.pos, v2.pos) - CAR_LENGTH
		v1_stop_dist	= self.get_stop_dist(	acc_brake = v1_acc_brake )
		v2_stop_dist	= v2.get_stop_dist(		acc_brake = v2_acc_brake )
		return dist_to_v2 + v2_stop_dist - v1_stop_dist
