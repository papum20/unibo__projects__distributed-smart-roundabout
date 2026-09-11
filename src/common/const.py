import math

from common.models.models import Position



# updates per second
UPDATES_P_S_VEHICLE		= 10.0
UPDATES_P_S_CONTROLLER	= 10.0

# time for a network timeout, or to consider data stale
TIMER_NETWORK_TIMEOUT		= 0.5	# s
# order to accelerate, if all vehicles inside have remained stationary for this duration
TIMER_CONTROLLER_DEADLOCK	= 2.0

# visualization area
AREA_RADIUS	= 150.0

ROUNDABOUT_POS			= Position(x=0, y=0)
# meters of boundary from center (furthest point from the center)
ROUNDABOUT_RADIUS		= 30.0
ROUNDABOUT_N_ROADS		= 4
ROUNDABOUT_PERIMETER	= 2 * math.pi * ROUNDABOUT_RADIUS

LANE_WIDTH		= 5.0
ROAD_LENGTH		= AREA_RADIUS - ROUNDABOUT_RADIUS	# length in the visualization
ROAD_WIDTH		= LANE_WIDTH * 2

VEHICLE_ANGLE_TOL_RAD			= 0.15
VEHICLE_ANGLE_TRAVELED_MIN_RAD	= 1.0
VEHICLE_DIST_TOL		= 0.5
VEHICLE_SPEED_TOL_PERC	= 0.1
# reaction time like TIMER_NETWORK_TIMEOUT, since computers don't have much delay
VEHICLE_REACTION_TIME_S	= TIMER_NETWORK_TIMEOUT
CAR_LENGTH				= 4.5
CAR_WIDTH				= 2.0
# car local vision
CAR_VISION_RADIUS_M		= 30.0
VEHICLE_SAFETY_MARGIN_M	= 5.0

# threshold where controller/vehicles consider to be about to enter
ROUNDABOUT_PROXIMITY_DIST	= 2 * CAR_LENGTH

# to avoid flooding of collisions for the same pair
COLLISION_COOLDOWN_S = 1.0
