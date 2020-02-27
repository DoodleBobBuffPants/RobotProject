from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import argparse
import numpy as np
import matplotlib.pylab as plt
import matplotlib.patches as patches
import scipy.signal
import sys
import yaml
import os
import re
import random
import rospy
import obstacle_avoidance
import rrt_navigation

directory = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../python')
sys.path.insert(0, directory)
try:
  import rrt
except ImportError:
  raise ImportError('Unable to import rrt.py. Make sure this file is in "{}"'.format(directory))


def get_combined_velocities(robot_poses):
    """
    param robot_poses: the ground truth positions of the robot currently
    return: the updated feedback linearized velocities for each robot, combining all velocity objective components
    """
    # TODO: Complete
    us = np.ones_like(robot_poses)
    ws = np.zeros_like(robot_poses)

    return us, ws 

def weight_velocities(goal_velocities, formation_velocities, obstacle_velocities):
    """
    param goal_velocities: the velocity directing the robot towards the goal (e.g to the next point on the path given by RRT)
    param formation_velocities: the velocities directing the robot to its desired position in the formation
    param obstacle_velocities: the velocities drecting the robot away from the obstacles.
    return: normalized weighted sum of the robot velocities
    """
    # TODO: Complete
    return None