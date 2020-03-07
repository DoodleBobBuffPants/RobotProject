from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from init_formations import LEADER_ID
from maintain_formation_decentralized import maintain_formation, CONTROLLED_ZONE

import numpy as np
import obstacle_avoidance
import rrt_navigation

# Feedback linearisation epsilon
EPSILON = 0.1

ROBOT_DISTANCE = 0.15

X = 0
Y = 1
YAW = 2

def get_obstacle_avoidance_velocity(robot_pose, laser):

  u, w = obstacle_avoidance.rule_based(*laser.measurements)
  
  x = u*np.cos(robot_pose[YAW]) - EPSILON*w*np.sin(robot_pose[YAW])
  y = u*np.sin(robot_pose[YAW]) + EPSILON*w*np.cos(robot_pose[YAW])

  return np.array([x, y])

def get_combined_velocity(robot_pose, leader_pose, leader_rrt_velocity, laser, robot_id):

    # Velocities
    rrt_velocity = leader_rrt_velocity
    formation_velocity = np.array([0., 0.])
    obstacle_avoidance_velocity = np.array([0., 0.])

    if robot_id != LEADER_ID:
      rrt_velocity = np.array([0., 0.])
      formation_velocity = maintain_formation(leader_pose, robot_pose, leader_rrt_velocity, robot_id)
      obstacle_avoidance_velocity = get_obstacle_avoidance_velocity(robot_pose, laser)

    combined_velocity = weight_velocities(rrt_velocity, formation_velocity, obstacle_avoidance_velocity)

    # Feedback linearization - convert combined_velocities [x,y] [u,w]
    u, w = rrt_navigation.feedback_linearized(pose=robot_pose, velocity=combined_velocity, epsilon=EPSILON)

    return u, w 

def weighting(velocity, weight):
  return np.array(velocity * weight)

def normalize(vec):
  mag = np.linalg.norm(vec)
  if mag < .01:
    return np.zeros_like(vec)
  else:
    return vec / mag

def weight_velocities(goal_velocity, formation_velocity, obstacle_velocity):

    goal_w = .1
    formation_w = .2
    static_obs_avoid_w = .7
    # currently no robot avoidance in decentralized algorithm as we do not keep all robot poses

    ngoal = normalize(goal_velocity)
    nform = normalize(formation_velocity)
    nobst = normalize(obstacle_velocity)

    goal = weighting(ngoal, goal_w)
    formation = weighting(nform, formation_w)
    static_obstacle_avoidance = weighting(nobst, static_obs_avoid_w)

    objective = goal + formation
    weighted_sum = objective + static_obstacle_avoidance

    if np.linalg.norm(objective) == 0.:
      weighted_sum = np.zeros_like(weighted_sum)

    return weighted_sum
