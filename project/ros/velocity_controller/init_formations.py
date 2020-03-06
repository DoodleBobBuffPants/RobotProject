from precomputed_rrt_paths import *

import numpy as np

# Formation spacing parameter for the formation
# SPACING_DIST = 0.8
SPACING_DIST = 0.8

# NOTE:
# In leader follower, only the followers need be defined in the formation.
# Initial yaw is the yaw set in the launch file

# LEADER_ID = 2
# INITIAL_YAW = 1.571
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

LEADER_ID = 0
INITIAL_YAW = 1.571
# INITIAL_YAW = 0.
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
	"GOAL_POSITION": np.array([1.5, 1.5], dtype=np.float32), 
	"RRT_BOUNDS": [[-2, 2], [-2, 2]],
	"MAP_NAME": "map",
	"RRT_ITERATIONS": 500,
	"RRT_PATH": SIMPLE_PATH}

CORRIDOR_MAP = {
	"GOAL_POSITION": np.array([-2.6, -0.1], dtype=np.float32),
 	"RRT_BOUNDS": [[-12, -2], [-1, 3]],
 	"MAP_NAME": "corridor",
 	"RRT_ITERATIONS": 1500,
 	"RRT_PATH": CORRIDOR_PATH}

MAZE_MAP = {
	"GOAL_POSITION": np.array([-1.0, -3.2], dtype=np.float32),
 	"RRT_BOUNDS": [[-4.3, -0.3], [-5.8, -2.5]],
 	"MAP_NAME": "maze",
 	"RRT_ITERATIONS": 1000,
 	"RRT_PATH": MAZE_PATH}

DIAMOND_MAP = {
	"GOAL_POSITION": np.array([3.3, -4.0], dtype=np.float32),
 	"RRT_BOUNDS": [[-12, -2], [-1, 3]],
 	"MAP_NAME": "diamond",
 	"RRT_ITERATIONS": 1500,
 	"RRT_PATH": DIAMOND_PATH}

# Set these params before running code
# NOTE: PATHS DEPEND ON STARTING POSE. IF YOU CHANGE STARTING POSE OF THE ROBOTS, YOU NEED TO RERUN RRT AND GET A NEW PATH
MAP_PARAMS = CORRIDOR_MAP

# Set to false to use the predefined path
RUN_RRT = False

FORMATION = DIAMOND
