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
import maintain_formation

directory = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../python')
sys.path.insert(0, directory)
try:
  import rrt
except ImportError:
  raise ImportError('Unable to import rrt.py. Make sure this file is in "{}"'.format(directory))

# Feedback linearisation epsilon
EPSILON = 0.1

X = 0
Y = 1
YAW = 2

# position for all robots to go to (for now - can change this to have a separate goal for every robot in the formation)
GOAL_POSITION = np.array([1.5, 1.5], dtype=np.float32)

def get_combined_velocities(robot_poses, rrt_velocities, lasers):
    """
    param robot_poses: the ground truth positions of the robot currently
    param occupancy_grid: the C map used by RRT
    return: the updated feedback linearized velocities for each robot, combining all velocity objective components
    """
    # TODO Update velocities come from rrt i.e. the velocity each robot is following to stay on the path
    #rrt_velocities = [[0.5, 0.5] for robot in robot_poses]

    # Velocities needed to maintain formation
    formation_velocities = maintain_formation.maintain_formation(current_poses=robot_poses, update_velocities=rrt_velocities)

    xyoa_velocities = []
    for i in range(len(robot_poses)):
        u, w = obstacle_avoidance.rule_based(*lasers[i].measurements)

        x = u*np.cos(robot_poses[i][YAW])
        y = u*np.sin(robot_poses[i][YAW])

        xyoa_velocities.append([x,y])
    xyoa_velocities = np.array(xyoa_velocities)

    combined_velocities = weight_velocities(rrt_velocities, formation_velocities, xyoa_velocities)

    # Feedback linearization - convert combined_velocities [[x,y], ...] into [[u, w], ...]
    # combined_velocities = rrt_velocities
    us = []
    ws = []
    for i in range(len(combined_velocities)):
      u, w = rrt_navigation.feedback_linearized(pose=robot_poses[i], velocity=combined_velocities[i], epsilon=EPSILON)

      us.append(u)
      ws.append(w)

    return us, ws 
  

def weight_velocities(goal_velocities, formation_velocities, obstacle_velocities):
    """
    param goal_velocities: the velocity directing the robot towards the goal (e.g to the next point on the path given by RRT)
    param formation_velocities: the velocities directing the robot to its desired position in the formation
    param obstacle_velocities: the velocities drecting the robot away from the obstacles.
    return: normalized weighted sum of the robot velocities
    """
    # Using weights from Table 1 in paper
    return (2 * obstacle_velocities) + (0.8 * goal_velocities) + (1 * formation_velocities)