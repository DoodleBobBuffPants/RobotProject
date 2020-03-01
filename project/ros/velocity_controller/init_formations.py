# Formation spacing parameter for the formation
import numpy as np

SPACING_DIST = 0.3
LEADER_ID = 0

# NOTE:
# due to changing to leader follower, the formations need some adjusting (of the order of positions)

# NOTE:
# In leader follower, only the followers need be defined in the formation.

# LINE = np.array([[2*SPACING_DIST,0], 
#         [SPACING_DIST, 0], 
#         [0, 0], 
#         [-1*SPACING_DIST, 0], 
#         [-2*SPACING_DIST, 0]])

# COLUMN =   np.array([[0, 2*SPACING_DIST], 
#             [0, SPACING_DIST], 
#             [0, 0], 
#             [0,-1*SPACING_DIST], 
#             [0, -2*SPACING_DIST]])


# DIAMOND
#         LEADER (0) not defined
#       1       2       4
#               3
#
#
DIAMOND =  np.array([[-SPACING_DIST, -SPACING_DIST], 
					 [0, -SPACING_DIST], 
					 [0, -2.*SPACING_DIST], 
					 [SPACING_DIST, -SPACING_DIST]])

# WEDGE =  np.array([[2*SPACING_DIST, -SPACING_DIST], 
#             [SPACING_DIST, 0], 
#             [0, SPACING_DIST], 
#             [-1*SPACING_DIST, 0], 
#             [-2*SPACING_DIST, -SPACING_DIST]])

FORMATION = DIAMOND