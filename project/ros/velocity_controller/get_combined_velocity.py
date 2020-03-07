from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from init_formations import LEADER_ID
from maintain_formation import maintain_formation, CONTROLLED_ZONE

import numpy as np
import obstacle_avoidance
import rrt_navigation

# Feedback linearisation epsilon
EPSILON = .2

# these might be too big for smaller formations...
# ROBOT_DISTANCE = .125 * 2
# ROBOT_DISTANCE = .15
ROBOT_DISTANCE = .19
# ROBOT_EXTRA_DISTANCE = 0.29
ROBOT_EXTRA_DISTANCE = 0.21

X = 0
Y = 1
YAW = 2

def get_obstacle_avoidance_velocities(robot_poses, lasers):

  xyoa_velocities = []
  min_distances = []
  corridor_status = []
  for i in range(len(robot_poses)):
    u, w, min_distance, in_corridor = obstacle_avoidance.braitenburg(robot_poses, i, *(lasers[i].measurements))
    # u, w = obstacle_avoidance.rule_based(*(lasers[i].measurements))

    x = u*np.cos(robot_poses[i][YAW]) - EPSILON*w*np.sin(robot_poses[i][YAW])
    y = u*np.sin(robot_poses[i][YAW]) + EPSILON*w*np.cos(robot_poses[i][YAW])

    xyoa_velocities.append(np.array([x,y]))
    min_distances.append(min_distance)
    corridor_status.append(in_corridor)

  return np.array(xyoa_velocities), np.array(min_distances), np.array(corridor_status)

def get_noise():
  noise = np.random.uniform(low=-1., high=1., size=[5, 2])
  noise = normalise_velocities(noise)

  return noise

def normalise_velocities(velocities):
  # Accounts for small magnitudes
  for i in range(len(velocities)):
    n = np.linalg.norm(velocities[i])

    if n < 1e-2: 
      velocities[i] = np.zeros_like(velocities[i])
    else:
      velocities[i] = velocities[i] / n
    
  return velocities
      

def get_combined_velocities(robot_poses, leader_rrt_velocity, lasers):

    # get leader and follower poses
    leader_pose = robot_poses[LEADER_ID]
    follower_poses = np.array([robot_poses[i] for i in range(len(robot_poses)) if i != LEADER_ID])

    # Velocities
    follower_formation_velocities, desired_positions, in_dead_zone = maintain_formation(leader_pose=leader_pose, follower_poses=follower_poses, leader_rrt_velocity=leader_rrt_velocity, lasers=lasers)
    
    obstacle_avoidance_velocities, min_obstacle_distances, corridor_status = get_obstacle_avoidance_velocities(robot_poses, lasers)

    # NOTE: for numpy insert, obj is the index of insertion.
    formation_velocities = np.insert(arr=follower_formation_velocities, obj=LEADER_ID, values=np.array([0., 0.]), axis=0)
    # follower formation velocities is only 4 long
    rrt_velocities = np.insert(arr=np.zeros_like(follower_formation_velocities), obj=LEADER_ID, values=leader_rrt_velocity, axis=0)
    #obstacle_avoidance_velocities = np.insert(arr=follower_obstacle_avoidance_velocities, obj=LEADER_ID, values=np.array([0., 0.]), axis=0)

    # normalized noise and rrt velocities.
    noise_velocities = get_noise()
    rrt_velocities = normalise_velocities(rrt_velocities)

    combined_velocities = weight_velocities(rrt_velocities, formation_velocities, obstacle_avoidance_velocities, robot_avoidance_weights(robot_poses), noise_velocities, in_dead_zone, min_obstacle_distances, corridor_status)

    # Feedback linearization - convert combined_velocities [[x,y], ...] into [[u, w], ...]
    us = []
    ws = []
    for i in range(len(combined_velocities)):
      u, w = rrt_navigation.feedback_linearized(pose=robot_poses[i], velocity=combined_velocities[i], epsilon=EPSILON)

    #   if u < 0.05:
    #     u = 0.05
    #     w = w * 0.7

      us.append(u)
      ws.append(w)

    return us, ws, desired_positions

def robot_avoidance_weights(robot_poses):
  # Earlier robot stops to let other pass
  # v = []
  # for i in range(len(robot_poses)):
  #   v.append(1.)
  #   for j in range(i+1, len(robot_poses)):
  #     if np.linalg.norm(robot_poses[i][:2] - robot_poses[j][:2]) < ROBOT_DISTANCE:
  #       v[i] = 0.
  # v = np.array(v)
  # return v

  # now robots stops if there is a robot infront of it.
  # for each robot, if any of the other robot poses are within say pi/2 or pi-delta infront of the robot at a small distance,
  # stop and wait for them to move.
  v = []
  for i in range(len(robot_poses)):
    v.append(1.)
    for j in range(len(robot_poses)):
      # for robots other than this one
      if j != i:
        # if the other robot is infront of it, and distance is < avoidance_threshold
        vector_to_robot = robot_poses[j] - robot_poses[i]
        distance = np.linalg.norm(vector_to_robot[0:2])

        angle_to_robot = np.arctan2(vector_to_robot[Y], vector_to_robot[X]) - robot_poses[i][YAW]

        # print("distance {}, {}: ".format(i, j), distance)
        # print("angle to robot: ", angle_to_robot)

        if -np.pi/2. < angle_to_robot < np.pi/2. and distance < ROBOT_EXTRA_DISTANCE:
          # stop dealock (if angle is big, i.e robots are next to each other, let the one with lower id go first.)
          if abs(angle_to_robot) > np.pi/3. and i > j:
            #print("AVOIDING DEADLOCK: {} {}".format(i, j))
            continue

          # if the robots are very close (or quite close but next to each other)
          elif distance < ROBOT_DISTANCE or abs(angle_to_robot) > np.pi/3:
            #print("STOPPING ROBOT: ", i)
            v[i] = 0.
          break
  
  return v

def weighting(velocities, weight):
  wv = np.array(velocities)
  for i in range(len(velocities)):
    wv[i] = velocities[i] * weight
  return wv

def normalize(vec):
  mag = np.linalg.norm(vec)
  if mag < .01:
    return np.zeros_like(vec)
  else:
    return vec / mag

def weight_velocities(goal_velocities, formation_velocities, obstacle_velocities, robot_avoidance_weights, noise_velocities, in_dead_zone, distances_to_obstacle, in_corridor):
    # SQUARE MAP WEIGHTS
    # goal_w = 0.8
    # formation_w = 0.9
    # static_obs_avoid_w = 1.
    # noise_w = 0.2
    # overall_weight = 0.8

    # # SIMPLE MAP WEIGHTS
    # goal_w = 0.2
    # formation_w = 0.2
    # static_obs_avoid_w = .2
    # noise_w = 0.05
    # overall_weight = 0.8

    # CORRIDOR MAP WEIGHTS
    goal_w = 0.35
    formation_w = 0.2
    static_obs_avoid_w = .22
    noise_w = 0.05
    overall_weight = 1.5

    # formation is the goal for followers
    # goal_velocities[LEADER_ID] = np.array([1., -1.]) / np.linalg.norm(np.array([1., -1.]))
    # goal_velocities[LEADER_ID] = np.array([0., 1.])
    goal = weighting(goal_velocities, goal_w)

    formation = weighting(formation_velocities, formation_w)
    static_obstacle_avoidance = weighting(obstacle_velocities, static_obs_avoid_w)
    noise = weighting(noise_velocities, noise_w)
    # print("goal: ", goal)
    # print("formation: ", formation)
    # print("static obstacles: ", static_obstacle_avoidance)

    # only leader has the goal, the rest have formation constraints
    objective = goal + formation

    # # RULE: if the follower robots are not in the deadzone and have a low minimum distance, tell the leader to stop, and give the follower goal velocity not formation.
    # all_robots_deadzone = np.insert(arr=in_dead_zone, obj=LEADER_ID, values=True, axis=0)
    # for i in range(len(objective)):
    #   if i != LEADER_ID:
    #     if not all_robots_deadzone[i] and distances_to_obstacle[i] < 0.22 and not in_corridor[i]:
    #       objective[i] = objective[LEADER_ID]
    #       objective[LEADER_ID] = np.zeros_like(goal[LEADER_ID])
    #       static_obstacle_avoidance[LEADER_ID] = np.zeros_like(goal[LEADER_ID])
          

    # sum of all velocity components
    weighted_sum = objective + static_obstacle_avoidance + noise

    # RULE: For each robot, if it is nearer other robots, let the first robot (by id) pass
    for i in range(len(objective)):
      if robot_avoidance_weights[i] == 0.:
        weighted_sum[i] = np.zeros_like(weighted_sum[i])

    # RULE: For each robot, if it has reached its objective, stop (ignore obstacle input)
    for i in range(len(objective)):
      if np.linalg.norm(objective[i]) == 0.:
        weighted_sum[i] = np.zeros_like(weighted_sum[i])

    # print([np.linalg.norm(o) for o in objective])

    # RULE: if the robots are outside the deadzone, the leader has to slow down, by how many robots are missing from the dead zone
    not_in_dead_zone = len(in_dead_zone) - sum(in_dead_zone)
    if not_in_dead_zone > 0:
      weighted_sum[LEADER_ID] = weighted_sum[LEADER_ID] * 0.3
    # weighted_sum[LEADER_ID] = weighted_sum[LEADER_ID] - ((weighted_sum[LEADER_ID]) * (not_in_dead_zone / len(in_dead_zone)))
          

    # goal[0] += static_obstacle_avoidance[0] + noise[0]
    # return goal
    return weighted_sum * overall_weight
