from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import numpy as np
import obstacle_avoidance
import rrt_navigation
from maintain_formation_decentralized import maintain_formation, CONTROLLED_ZONE
from init_formations import LEADER_ID

# Feedback linearisation epsilon
EPSILON = 0.1

ROBOT_DISTANCE = 0.125

X = 0
Y = 1
YAW = 2

def get_obstacle_avoidance_velocity(robot_pose, laser):

  u, w = obstacle_avoidance.braitenberg(*laser.measurements)
  
  # Adjustable deltas that affect the contribution of obstacle avoidance
  du, dw = .4, .2
  x = du*u*np.cos(robot_pose[YAW] + dw*w)
  y = du*u*np.sin(robot_pose[YAW] + dw*w)

  return np.array([x, y])

def get_combined_velocity(robot_pose, leader_pose, leader_rrt_velocity, laser, robot_id):

    # Velocities
    rrt_velocity = leader_rrt_velocity
    formation_velocity = np.array([0., 0.])

    if robot_id != LEADER_ID:
      rrt_velocity = np.array([0., 0.])
      formation_velocity = maintain_formation(leader_pose, robot_pose, leader_rrt_velocity, robot_id)
      formation_velocity = formation_velocity[0]

    obstacle_avoidance_velocity = get_obstacle_avoidance_velocity(robot_pose, laser)

    combined_velocity = weight_velocities(rrt_velocity, formation_velocity, obstacle_avoidance_velocity)

    # Feedback linearization - convert combined_velocities [x,y] [u,w]
    u, w = rrt_navigation.feedback_linearized(pose=robot_pose, velocity=combined_velocity, epsilon=EPSILON)

    return u, w 

def weighting(velocity, weight):
  return np.array(velocity * weight)

def weight_velocities(goal_velocity, formation_velocity, obstacle_velocity):

    goal_w = .8
    formation_w = .8
    static_obs_avoid_w = .8

    goal = weighting(goal_velocity, goal_w)
    formation = weighting(formation_velocity, formation_w)
    static_obstacle_avoidance = weighting(obstacle_velocity, static_obs_avoid_w)

    objective = goal + formation
    weighted_sum = objective + static_obstacle_avoidance

    if np.linalg.norm(objective) == 0.:
      weighted_sum = np.zeros_like(weighted_sum)

    return weighted_sum
