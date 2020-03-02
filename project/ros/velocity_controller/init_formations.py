import numpy as np
from precomputed_rrt_paths import SIMPLE_PATH, CORRIDOR_PATH

# Formation spacing parameter for the formation
SPACING_DIST = 0.3

# NOTE:
# In leader follower, only the followers need be defined in the formation.

#LEADER_ID = 2
INITIAL_YAW = 1.571 # this depends on initial yaw stated in the launch file.
LINE = np.array([[2*SPACING_DIST,0], 
				 [SPACING_DIST, 0],
				 [-SPACING_DIST, 0], 
				 [-2*SPACING_DIST, 0]])

# LEADER_ID = 2
# INITIAL YAW = 
COLUMN = np.array([[0, 2*SPACING_DIST], 
            	   [0, SPACING_DIST],
            	   [0,-SPACING_DIST], 
            	   [0, -2*SPACING_DIST]])

# DIAMOND
#       LEADER (0) not defined
#       1       2       4
#               3

DIAMOND =  np.array([[-SPACING_DIST, -SPACING_DIST], 
					 [0, -SPACING_DIST], 
					 [0, -2.*SPACING_DIST], 
					 [SPACING_DIST, -SPACING_DIST]])

# LEADER_ID = 2
# INITIAL YAW = 
WEDGE = np.array([[2*SPACING_DIST, -SPACING_DIST], 
				  [SPACING_DIST, 0],
				  [-SPACING_DIST, 0], 
				  [-2*SPACING_DIST, -SPACING_DIST]])


# for each map, some parameters are hard coded. Use the map_params to switch between the desired map configuration
# RRT bounds are [[x_min, x_max], [y_min, y_max]]
# Now try setting this pose param to include initial yaw and see if the initial yaw problem goes away...
SIMPLE_MAP = {
	"START_POSE":np.array([-1.5, -1.5, INITIAL_YAW], dtype=np.float32),
	"GOAL_POSITION":np.array([1.5, 1.5], dtype=np.float32), 
	"RRT_BOUNDS":[[-2, 2], [-2, 2]], "MAP_NAME":"map", "RRT_ITERATIONS":500, "RRT_PATH":SIMPLE_PATH}

CORRIDOR_MAP = {
	"START_POSE":np.array([-11.0, -0.2, INITIAL_YAW], dtype=np.float32),
	"GOAL_POSITION":np.array([-2.6, -0.1], dtype=np.float32),
 	"RRT_BOUNDS":[[-12, -2], [-1, 3]], "MAP_NAME":"corridor", "RRT_ITERATIONS":1500, "RRT_PATH":CORRIDOR_PATH}

# Select these params before running code
# NOTE: PATHS DEPEND ON STARTING POSE. IF YOU CHANGE STARTING POSE OF THE ROBOTS, YOU NEED TO RERUN RRT AND GET A NEW PATH
MAP_PARAMS = CORRIDOR_MAP
RUN_RRT = False # set to true to not use the predefined path
FORMATION = DIAMOND
LEADER_ID = 0
