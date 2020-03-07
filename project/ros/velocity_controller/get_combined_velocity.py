from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from init_formations import LEADER_ID
from maintain_formation import maintain_formation, CONTROLLED_ZONE

import numpy as np
import obstacle_avoidance
import rrt_navigation

# Feedback linearisation epsilon
EPSILON = .1

ROBOT_DISTANCE = .125

X = 0
Y = 1
YAW = 2

def get_obstacle_avoidance_velocities(robot_poses, lasers):

  xyoa_velocities = []
  for i in range(len(robot_poses)):
    u, w = obstacle_avoidance.rule_based(*(lasers[i].measurements))

    x = u*np.cos(robot_poses[i][YAW]) - EPSILON*w*np.sin(robot_poses[i][YAW])
    y = u*np.sin(robot_poses[i][YAW]) + EPSILON*w*np.cos(robot_poses[i][YAW])

    xyoa_velocities.append(np.array([x,y]))

  return np.array(xyoa_velocities)

def get_combined_velocities(robot_poses, leader_rrt_velocity, lasers):

    # get leader and follower poses
    leader_pose = robot_poses[LEADER_ID]
    follower_poses = np.array([robot_poses[i] for i in range(len(robot_poses)) if i != LEADER_ID])
    follower_lasers = [lasers[i] for i in range(len(lasers)) if i != LEADER_ID]

    # Velocities
    follower_formation_velocities, desired_positions = maintain_formation(leader_pose=leader_pose, follower_poses=follower_poses, leader_rrt_velocity=leader_rrt_velocity)
    follower_obstacle_avoidance_velocities = get_obstacle_avoidance_velocities(follower_poses, follower_lasers)

    # NOTE: for numpy insert, obj is the index of insertion.
    formation_velocities = np.insert(arr=follower_formation_velocities, obj=LEADER_ID, values=np.array([0., 0.]), axis=0)
    # follower formation velocities is only 4 long
    rrt_velocities = np.insert(arr=np.zeros_like(follower_formation_velocities), obj=LEADER_ID, values=leader_rrt_velocity, axis=0)
    obstacle_avoidance_velocities = np.insert(arr=follower_obstacle_avoidance_velocities, obj=LEADER_ID, values=np.array([0., 0.]), axis=0)

    combined_velocities = weight_velocities(rrt_velocities, formation_velocities, obstacle_avoidance_velocities, robot_avoidance_weights(robot_poses))

    # Feedback linearization - convert combined_velocities [[x,y], ...] into [[u, w], ...]
    us = []
    ws = []
    for i in range(len(combined_velocities)):
      u, w = rrt_navigation.feedback_linearized(pose=robot_poses[i], velocity=combined_velocities[i], epsilon=EPSILON)

      us.append(u)
      ws.append(w)

    return us, ws, desired_positions

def robot_avoidance_weights(robot_poses):
  # Earlier robot stops to let other pass
  v = []
  for i in range(len(robot_poses)):
    v.append(1.)
    for j in range(i+1, len(robot_poses)):
      if np.linalg.norm(robot_poses[i][:2] - robot_poses[j][:2]) < ROBOT_DISTANCE:
        v[i] = 0.
  v = np.array(v)
  return v

def weighting(velocities, weight):
  wv = np.array(velocities)
  for i in range(len(velocities)):
    wv[i] = velocities[i] * weight
  return wv

def weight_velocities(goal_velocities, formation_velocities, obstacle_velocities, robot_avoidance_weights):

    # weights
    goal_w = .8
    formation_w = 1.2
    static_obs_avoid_w = 2.

    # formation is the goal for followers
    goal = weighting(goal_velocities, goal_w)
    formation = weighting(formation_velocities, formation_w)
    static_obstacle_avoidance = weighting(obstacle_velocities, static_obs_avoid_w)

    # print("goal: ", goal)
    # print("formation: ", formation)
    # print("static obstacles: ", static_obstacle_avoidance)

    # only leader has the goal, the rest have formation constraints
    objective = goal + formation

    # sum of all velocity components
    weighted_sum = objective + static_obstacle_avoidance

    # RULE: For each robot, if it is nearer other robots, let the first robot (by id) pass
    for i in range(len(objective)):
      if robot_avoidance_weights[i] == 0.:
        weighted_sum[i] = np.zeros_like(weighted_sum[i])

    # RULE: For each robot, if it has reached its objective, stop (ignore obstacle input)
    for i in range(len(objective)):
      if np.linalg.norm(objective[i]) == 0.:
        weighted_sum[i] = np.zeros_like(weighted_sum[i])

    return weighted_sum
