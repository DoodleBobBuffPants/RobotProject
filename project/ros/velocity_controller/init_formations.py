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

LEADER_ID = 0
INITIAL_YAW = 1.571
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

FORMATION = DIAMOND
