from common.models.models import Position



FRAMERATE	= 10.0

# visualization area
AREA_RADIUS	= 150.0

ROUNDABOUT_POS			= Position(x=0, y=0)
# meters of boundary from center (furthest point from the center)
ROUNDABOUT_RADIUS		= 30.0
ROUNDABOUT_N_ROADS		= 4

ROAD_LENGTH		= AREA_RADIUS - ROUNDABOUT_RADIUS	# length in the visualization
LANE_WIDTH		= 5.0

VEHICLE_ANGLE_TOL_RAD			= 0.15
VEHICLE_ANGLE_TRAVELED_MIN_RAD	= 1.0
VEHICLE_DIST_TOL		= 0.5
CAR_LENGTH				= 4.5
CAR_WIDTH				= 2.0
