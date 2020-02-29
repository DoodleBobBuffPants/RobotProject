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
THRESHOLD = 0.01
ROBOT_DISTANCE = 0.1
SPEED = .2

X = 0
Y = 1
YAW = 2

# position for all robots to go to (for now - can change this to have a separate goal for every robot in the formation)
GOAL_POSITION = np.array([1.5, 1.5], dtype=np.float32)

def get_obstacle_avoidance_velocities(lasers):
  """
  lasers: laser measurements as a list from all 5 robots

  Return obstacle avoidance velocities [[x,y], ...]
  """
  xyoa_velocities = []
    for i in range(len(robot_poses)):
        u, w = obstacle_avoidance.braitenberg(*lasers[i].measurements)

        # TODO change delta later?
        delta = 0.5
        x = u*np.cos(robot_poses[i][YAW] + delta*w)
        y = u*np.sin(robot_poses[i][YAW] + delta*w)

        xyoa_velocities.append(np.array([x,y]))

  return np.array(xyoa_velocities)

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

    obstacle_avoidance_velocities = get_obstacle_avoidance_velocities(lasers)

    combined_velocities = weight_velocities(rrt_velocities, formation_velocities, obstacle_avoidance_velocities, robot_avoidance_velocities(robot_poses))

    # Feedback linearization - convert combined_velocities [[x,y], ...] into [[u, w], ...]
    # combined_velocities = rrt_velocities
    us = []
    ws = []
    for i in range(len(combined_velocities)):
      u, w = rrt_navigation.feedback_linearized(pose=robot_poses[i], velocity=combined_velocities[i], epsilon=EPSILON)

      us.append(u)
      ws.append(w)

    return us, ws 

def robot_avoidance_velocities(robot_poses):
  #TODO: Better robot avoidance
  v = []
  for i in range(len(robot_poses)):
    v.append(np.array([0,0]))
    for j in range(len(robot_poses)):
      if i != j:
        if np.linalg.norm(robot_poses[i][:Y] - robot_poses[j][:Y]) < ROBOT_DISTANCE:
          #Velocity vector pointing diagonally away from robot. Assumes they face each other
          vec = np.array([np.cos(robot_poses[i][YAW] + np.pi/4.), np.sin(robot_poses[i][YAW] + np.pi/4.)])
          v[i] = v[i] + SPEED*vec
  v = np.array(v)
  return v

def normalize(v):
  """
  Normalise vectors to all be of length 1
  But if length of the vector is already small, (<1e-2), vector becomes all zeros
  n = normalised(v) = root(v[0]**2+v[1]**2)
  Returns [0, 0] if n < 1e-2
  else returns v / n = v / |v|
  """
  n = np.linalg.norm(v)
  if n < 1e-2:
    return np.zeros_like(v)
  return v / n

def weighting(velocities, weight):
  wv = []
  for i in range(len(velocities)):
    theta = 0.
    if np.sqrt(velocities[i][X]**2 + velocities[i][Y]**2) > THRESHOLD:
      theta = np.abs(np.arctan(velocities[i][Y]/velocities[i][X]))

    if np.linalg.norm(velocities[i]) < 0.2 and theta < np.pi/18.:
      wv.append(0. * velocities[i])
    else:
      wv.append(weight * velocities[i])
  return np.array(wv)

def weight_velocities(goal_velocities, formation_velocities, obstacle_velocities, robot_avoidance_velocities):
    """
    param goal_velocities: the velocity directing the robot towards the goal (e.g to the next point on the path given by RRT)
    param formation_velocities: the velocities directing the robot to its desired position in the formation
    param obstacle_velocities: the velocities drecting the robot away from the obstacles.
    return: weighted sum of the robot velocities
    """

    goal = weighting(goal_velocities, .8)
    formation = []
    obstacle = []
    robot_avoidance = []

    #weight 0 if at goal to stop movement
    for i in range(len(goal)):
      theta = 0.
      if np.sqrt(goal[i][X]**2 + goal[i][Y]**2) > THRESHOLD:
        theta = np.abs(np.arctan(goal[i][Y]/goal[i][X]))

      if np.linalg.norm(goal[i]) < 0.2 and theta < np.pi/18.:
        formation.append(np.array([0., 0.]))
        obstacle.append(np.array([0., 0.]))
        robot_avoidance.append(np.array([0., 0.]))
      else:
        formation.append(weighting(np.array([formation_velocities[i]]), 1.).flatten())
        obstacle.append(weighting(np.array([obstacle_velocities[i]]), 1.5).flatten())
        robot_avoidance.append(weighting(np.array([robot_avoidance_velocities[i]]), 2.).flatten())

    weighted_sum = goal + np.array(formation) + np.array(obstacle) + np.array(robot_avoidance)

    return weighted_sum