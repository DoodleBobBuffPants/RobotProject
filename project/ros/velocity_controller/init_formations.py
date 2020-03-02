import numpy as np

# Formation spacing parameter for the formation
SPACING_DIST = 0.3

# NOTE:
# In leader follower, only the followers need be defined in the formation.

#LEADER_ID = 2
#INITIAL_YAW = 1.571
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

# LEADER_ID = 0
# INITIAL_YAW = 1.571
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

# RRT bounds are [[x_min, x_max], [y_min, y_max]]
# Now try setting this pose param to include initial yaw and see if the initial yaw problem goes away...
SIMPLE_MAP = {
	"START_POSE": np.array([-1.5, -1.5, 0.], dtype=np.float32),
	"GOAL_POSITION": np.array([0., 1.5], dtype=np.float32), 
	"RRT_BOUNDS": [[-2, 2], [-2, 2]],
	"MAP_NAME": "map",
	"RRT_ITERATIONS": 500,
	"RRT_PATH": None}

CORRIDOR_MAP = {
	"START_POSE": np.array([-1.5, -1.5, 0.], dtype=np.float32),
	"GOAL_POSITION": np.array([-2.6, -0.1], dtype=np.float32),
 	"RRT_BOUNDS": [[-12, -2], [-1, 3]],
 	"MAP_NAME": "corridor",
 	"RRT_ITERATIONS": 1500,
 	"RRT_PATH": None}

# Set these params before running code
MAP_PARAMS = SIMPLE_MAP

# set to false to use a predefined path (not included yet)
RUN_RRT = True 

FORMATION = DIAMOND
LEADER_ID = 0

# can be found in launch file
INITIAL_YAW = 1.571
