# Formation spacing parameter for the formation
import numpy as np

SPACING_DIST = 0.3
LINE = np.array([[2*SPACING_DIST,0], 
        [SPACING_DIST, 0], 
        [0, 0], 
        [-1*SPACING_DIST, 0], 
        [-2*SPACING_DIST, 0]])

COLUMN =   np.array([[0, 2*SPACING_DIST], 
            [0, SPACING_DIST], 
            [0, 0], 
            [0,-1*SPACING_DIST], 
            [0, -2*SPACING_DIST]])

#TODO: update diamond to be top to bottom, right to left
DIAMOND =  np.array([[SPACING_DIST, 0], 
            [0, SPACING_DIST], 
            [0, 0], 
            [0,-SPACING_DIST], 
            [-SPACING_DIST, 0]])

WEDGE =  np.array([[2*SPACING_DIST, -SPACING_DIST], 
            [SPACING_DIST, 0], 
            [0, SPACING_DIST], 
            [-1*SPACING_DIST, 0], 
            [-2*SPACING_DIST, -SPACING_DIST]])

FORMATION = DIAMOND