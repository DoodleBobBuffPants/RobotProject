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
from maintain_formation import maintain_formation

directory = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../python')
sys.path.insert(0, directory)
try:
  import rrt
except ImportError:
  raise ImportError('Unable to import rrt.py. Make sure this file is in "{}"'.format(directory))

from init_formations import LEADER_ID

# Feedback linearisation epsilon
EPSILON = 0.1
THRESHOLD = 0.01
ROBOT_DISTANCE = 0.1
SPEED = .2
GOAL_THRESHOLD = 0.1

X = 0
Y = 1
YAW = 2

# position for all robots to go to (for now - can change this to have a separate goal for every robot in the formation)
GOAL_POSITION = np.array([1.5, 1.5], dtype=np.float32)

def get_obstacle_avoidance_velocities(robot_poses, lasers):
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

def scale_velocities(velocities, min = 0, max=3.5):
  """
  param:
  min: velocities magnitude min = 0
  max: theoretical max velocity magnitude, =3.5 assuming as velocity functions return a capped valocity 

  Given a list of [[x,y], ...] velocities, scale magnitudes to be between 0 and 1
  """
  # Magnitudes of each of the velocities
  magnitudes = np.array([[np.linalg.norm(v)] for v in velocities])

  # Normalised magnitudes Min max scalar: x-min / max-min
  scaled_magnitudes = (magnitudes - min) / (max - min)

  # Scale velocities 
  scaled_velocities = (velocities / magnitudes) * scaled_magnitudes

  return scaled_velocities

def get_combined_velocities(robot_poses, leader_rrt_velocity, lasers):
    """
    param leader_pose: ground truth pose of the leader
    param follower_poses: the ground truth poses of the followers
    param leader_rrt_velocity: rrt_velocity of the leader
    param lasers: the information from each robot on its sensors in the gazebo simulation.

    return: the updated feedback linearized velocities for each robot, combining all velocity objective components
    """
    # get leader and follower poses
    leader_pose = robot_poses[LEADER_ID]
    follower_poses = np.array([robot_poses[i] for i in range(len(robot_poses)) if i != LEADER_ID])

    # Velocities needed to maintain formation
    follower_formation_velocities = maintain_formation(leader_pose=leader_pose, follower_poses=follower_poses, leader_rrt_velocity=leader_rrt_velocity)

    obstacle_avoidance_velocities = get_obstacle_avoidance_velocities(robot_poses, lasers)

    # Scale velocties to be between 0 and 1
    # formation_velocities = scale_velocities(formation_velocities)
    # obstacle_avoidance_velocities = scale_velocities(obstacle_avoidance_velocities)
    # rrt_velocities = scale_velocities(rrt_velocities, min=0, max=0.4)

    # NOTE: for numpy insert, obj is the index of insertion.
    formation_velocities = np.insert(arr=follower_formation_velocities, obj=LEADER_ID,  values=np.array([0.,0.]), axis=0)
    # follower formation velocities is only 4 long
    rrt_velocities = np.insert(arr=np.zeros_like(follower_formation_velocities), obj=LEADER_ID, values=leader_rrt_velocity, axis=0)

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
  # wv = []
  # for i in range(len(velocities)):
  #   theta = 0.
  #   if np.sqrt(velocities[i][X]**2 + velocities[i][Y]**2) > THRESHOLD:
  #     theta = np.abs(np.arctan(velocities[i][Y]/velocities[i][X]))

  #   if np.linalg.norm(velocities[i]) < 0.2 and theta < np.pi/18.:
  #     wv.append(0. * velocities[i])
  #   else:
      # wv.append(weight * velocities[i])
  # return np.array(wv)
  return weight * velocities

def weight_velocities(goal_velocities, formation_velocities, obstacle_velocities, robot_avoidance_velocities):
    """
    param goal_velocities: the velocity directing the robot towards the goal (e.g to the next point on the path given by RRT)
    param formation_velocities: the velocities directing the robot to its desired position in the formation
    param obstacle_velocities: the velocities drecting the robot away from the obstacles.
    return: weighted sum of the robot velocities
    """

    goal = weighting(goal_velocities, 1.)
    formation = weighting(formation_velocities, .6)
    static_obstacle_avoidance = weighting(obstacle_velocities, 0.)
    robot_avoidance = weighting(robot_avoidance_velocities, 0.)

    # print([np.linalg.norm(goal[i]) for i in range(len(goal))])

    # weight 0 if at goal to stop movement
    # for i in range(len(goal)):
      # theta = 0.
      # if np.sqrt(goal[i][X]**2 + goal[i][Y]**2) > THRESHOLD:
      #   theta = np.abs(np.arctan(goal[i][Y]/goal[i][X])

      # if np.linalg.norm(goal[i]) < GOAL_THRESHOLD: #and theta < np.pi/18.: why pi/18
      #   print("goal found")
      #   formation[i] = static_obstacle_avoidance[i] = robot_avoidance[i] = np.array([0., 0.])
      #   goal[i] = np.array([0.,0.])

    # compute the weighted sum of all the competing velocities
    print("goal: ", goal)
    print("formation: ", formation)
    print("soa: ", static_obstacle_avoidance)
    print("robot_avoidance: ", robot_avoidance)
    weighted_sum = goal + formation + static_obstacle_avoidance + robot_avoidance
    return weighted_sum