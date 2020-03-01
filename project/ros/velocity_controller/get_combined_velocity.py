from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import numpy as np
import obstacle_avoidance
import rrt_navigation
from maintain_formation import maintain_formation
from init_formations import LEADER_ID

# Feedback linearisation epsilon
EPSILON = 0.1

THRESHOLD = 0.01
ROBOT_DISTANCE = 0.1

X = 0
Y = 1
YAW = 2

def get_obstacle_avoidance_velocities(robot_poses, lasers):

  xyoa_velocities = []
  for i in range(len(robot_poses)):
      u, w = obstacle_avoidance.braitenberg(*lasers[i].measurements)

      du, dw = 1., .5
      x = du*u*np.cos(robot_poses[i][YAW] + dw*w)
      y = du*u*np.sin(robot_poses[i][YAW] + dw*w)

      xyoa_velocities.append(np.array([x,y]))

  return np.array(xyoa_velocities)

def scale_velocities(velocities, min = 0, max=3.5):

  magnitudes = np.array([[np.linalg.norm(v)] for v in velocities])

  scaled_velocities = np.array(velocities)
  for i in range(len(magnitudes)):
    # Don't divide by very small magnitudes
    if magnitudes[i] > THRESHOLD:
      # Normalised magnitudes Min max scalar: x-min / max-min
      scaled_magnitude = (magnitudes[i] - min) / (max - min)
      scaled_velocities[i] = (velocities[i] / magnitudes[i]) * scaled_magnitudes

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

    # Velocities
    follower_formation_velocities = maintain_formation(leader_pose=leader_pose, follower_poses=follower_poses, leader_rrt_velocity=leader_rrt_velocity)
    obstacle_avoidance_velocities = get_obstacle_avoidance_velocities(robot_poses, lasers)

    # Scale velocties to be between 0 and 1
    # formation_velocities = scale_velocities(formation_velocities)
    # obstacle_avoidance_velocities = scale_velocities(obstacle_avoidance_velocities)
    # rrt_velocities = scale_velocities(rrt_velocities, max=0.4)

    # NOTE: for numpy insert, obj is the index of insertion.
    formation_velocities = np.insert(arr=follower_formation_velocities, obj=LEADER_ID, values=np.array([0., 0.]), axis=0)
    # follower formation velocities is only 4 long
    rrt_velocities = np.insert(arr=np.zeros_like(follower_formation_velocities), obj=LEADER_ID, values=leader_rrt_velocity, axis=0)

    combined_velocities = weight_velocities(rrt_velocities, formation_velocities, obstacle_avoidance_velocities, robot_avoidance_weights(robot_poses))

    # Feedback linearization - convert combined_velocities [[x,y], ...] into [[u, w], ...]
    us = []
    ws = []
    for i in range(len(combined_velocities)):
      u, w = rrt_navigation.feedback_linearized(pose=robot_poses[i], velocity=combined_velocities[i], epsilon=EPSILON)

      us.append(u)
      ws.append(w)

    return us, ws 

def robot_avoidance_weights(robot_poses):
  v = []
  for i in range(len(robot_poses)):
    v.append(1.)
    for j in range(i+1, len(robot_poses)):
      if np.linalg.norm(robot_poses[i][:2] - robot_poses[j][:2]) < ROBOT_DISTANCE:
        v[i] = 0.
  v = np.array(v)
  return v

def weighting(velocities, weights):
  wv = np.array(velocities)
  for i in range(len(velocities)):
    wv[i] = velocities[i] * weights[i]
  return wv

def weight_velocities(goal_velocities, formation_velocities, obstacle_velocities, robot_avoidance_weights):
    """
    param goal_velocities: the velocity directing the robot towards the goal (e.g to the next point on the path given by RRT)
    param formation_velocities: the velocities directing the robot to its desired position in the formation
    param obstacle_velocities: the velocities drecting the robot away from the obstacles.
    return: weighted sum of the robot velocities
    """

    goal = weighting(goal_velocities, 1. * robot_avoidance_weights)
    formation = weighting(formation_velocities, 1. * robot_avoidance_weights)
    static_obstacle_avoidance = weighting(obstacle_velocities, 0. * robot_avoidance_weights)

    # print([np.linalg.norm(goal[i]) for i in range(len(goal))])

    # print("goal: ", goal)
    # print("formation: ", formation)
    # print("soa: ", static_obstacle_avoidance)
    # print("robot_avoidance: ", robot_avoidance)

    weighted_sum = goal + formation + static_obstacle_avoidance

    return weighted_sum