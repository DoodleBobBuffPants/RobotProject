from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from rrt_navigation import feedback_linearized

import numpy as np


OBSTACLE_DETECTION_THRESHOLD = 0.5
LEADER_OBS_DETECT_THRESHOLD = 0.4
ROBOT_DETECTION_THRESHOLD = OBSTACLE_DETECTION_THRESHOLD + 0.06
RIGHT, FRONT_RIGHT, FRONT, FRONT_LEFT, LEFT = 0, 1, 2, 3, 4
MAX_DISTANCE = 3.5 # max distance measured by the robot sensor
X = 0
Y = 1
YAW = 2

# Feedback linearisation epsilon
EPSILON = .1


def rule_based(front, front_left, front_right, left, right):
  u = 0.  # [m/s]
  w = 0.  # [rad/s] going counter-clockwise.

  # apply tanh to the input to deal with infinity
  sens_inp = np.array([front, front_left, front_right, left, right])
  sens_inp = np.tanh(sens_inp)

  # if close to front, move away
  if sens_inp[0] < np.tanh(.4):
    u = -.25
    if sens_inp[1] < sens_inp[2]:
      w = w - (np.pi/2.)
    else:
      w = w + (np.pi/2.)
    return u, w

  return u * 0.33, w * 0.33

  # if all sensors detect obstacles are far away, adjust slightly using a highest absolute value for direction
  if (sens_inp > np.tanh(.8)).all():
    # move in coward motion, away from the closest object
    return u, sens_inp[2] + sens_inp[4] - sens_inp[1] - sens_inp[3]

  # breakout! (mentioned in part C only - rest of rules are as described in part a) 
  # If the front sensor reports a much greater distance than the side sensors, 
  # don't change direction so much, use the left and right sensors for small adjustments
  if sens_inp[0] > np.tanh(.8) and sens_inp[0] > 2.*sens_inp[1] and sens_inp[0] > 2.*sens_inp[2]:
    # override w updates and steer using left and right sensors, which should be close enough to detect collisions
    # in tight areas
    if sens_inp[3] < np.tanh(.4):
      w = w + 1.5*(1.-sens_inp[3])
    if sens_inp[4] < np.tanh(.4):
      w = w - 1.5*(1.-sens_inp[4])
    if sens_inp[1] < np.tanh(.4):
      w = w + 1.5*(1.-sens_inp[1])
    if sens_inp[2] < np.tanh(.4):
      w = w - 1.5*(1.-sens_inp[2])

    return u, w


  # if the right hand side detects an approaching object , alter w to move left
  if sens_inp[2] < np.tanh(.4):
    w = w + 4.*(1.-sens_inp[2])

  # if the left hand side detects and approaching object, alter w to move to the right
  if sens_inp[1] < np.tanh(.4):
    w = w - 4.*(1.-sens_inp[1])

  # if robot is very close to the right hand slide, adjust left a little
  if sens_inp[4] < np.tanh(.4):
    w = w + 2.*(1.-sens_inp[4])

  # if robot is very close to the left hand slide, adjust right a little
  if sens_inp[3] < np.tanh(.4):
    w = w - 2.*(1.-sens_inp[3])
  
  return u, w


def rule_based_leader(front, front_left, front_right, left, right, formation_pose, robot_poses, robot_id):
  u = 0.
  w = 0.

  # smooth sensor inputs with tanh
  sens_inp = np.array([right, front_right, front, front_left, left])
  sens_inp = np.tanh(sens_inp)
  detection_level = np.tanh(LEADER_OBS_DETECT_THRESHOLD)

  # CHECK IF SENSORS ARE BEING TRIGGERED BY OTHER ROBOTS
  # compute distances to other robots
  robot_pose = robot_poses[robot_id]
  x = robot_pose[X]
  y = robot_pose[Y]
  distances = [np.sqrt(np.square(x- r_p[X]) + np.square(y - r_p[Y])) for r_p in robot_poses]
  # vectors from current robot to all the other robots
  vectors =  [r_p[0:2] - robot_pose[0:2] for r_p in robot_poses]

  pi = np.pi
  sensor_angles = [-pi/2., -pi/4., 0., pi/4, pi/2.]

  # check if the front sensor detects something
  # for each sensor, if there is a robot infront of it, ignore it (by setting the distance it measures to max distance)
  for i in range(len(sensor_angles)):
    if robot_infront_of_sensor(i, sensor_angles[i], robot_poses, distances, vectors, robot_id):
      sens_inp[i] = np.tanh(MAX_DISTANCE)


  # AVOID OBSTACLES

  # if no sensors detect an obstacle, return nothing
  obstacle = False
  for sensor in sens_inp:
    if sensor < detection_level:
      obstacle = True
      break
  
  if not obstacle:
    return u, w

  # OBSTACLE DETECTED
  obstacle_closeness = 1 - min(sens_inp)
  # sensors_detected = np.array(["right", "front_right", "front", "front_left", "left"])[sens_inp < detection_level]
  # if robot_id == 0:
  #   print(sensors_detected)
  #   print(sens_inp[sens_inp < detection_level])
  #   print(["right", "front_right", "front", "front_left", "left"])
  #   print(sens_inp)
  #   print("------")

  # could have a rule if the formation centre behind the robot, go to it, but this might discourage split and merge
  
  # find the sensor that points in the direction closest to the formation: (sensors are pi/4 apart, between -pi/2 and pi/2)
  to_formation_vector = formation_pose[0:2] - robot_pose[0:2]
  to_formation_angle = np.arctan2(to_formation_vector[Y], to_formation_vector[X])

  # corridor rule
  # if the front sensor is much bigger than the front right or front left sensors, just go forward, adjusting slightly front left or front right
  if sens_inp[FRONT] > 0.88 and sens_inp[FRONT_LEFT] > 0.22 and sens_inp[FRONT_RIGHT] > 0.22:
    # print("CORRIDOR RULE TRIGGERED")
    w = w - .5*(1.-sens_inp[FRONT_LEFT]) + .5*(1.-sens_inp[FRONT_RIGHT])
    return u, w

  # best direction to turn in based on front side sensors
  closest = (1- sens_inp[RIGHT]) - (1- sens_inp[LEFT])
  direction = 1. if front_right < front_left else -1.
  # print("direction: ", direction)

  # THE FRONT SENSOR DETECTS FIRST
  if sens_inp[FRONT] < detection_level:
    # print("FRONT TURN AWAY RULE")
    # move either left or right depending on which side front sensor is the furthest away
    w = w + direction * (1- sens_inp[FRONT]) * .75

  # steer when sens_inp is low on the front sides
  if sens_inp[FRONT_LEFT] < detection_level or sens_inp[FRONT_RIGHT] < detection_level:
    # print("SIDE STEER RULE")
    w = w + direction * (1- min(sens_inp[FRONT_RIGHT], sens_inp[FRONT_LEFT])) * .65 * obstacle_closeness

  return u, w

def rule_based_followers(front, front_left, front_right, left, right, formation_pose, robot_poses, robot_id):
  u = 0.
  w = 0.

  # smooth sensor inputs with tanh
  sens_inp = np.array([right, front_right, front, front_left, left])
  sens_inp = np.tanh(sens_inp)
  detection_level = np.tanh(OBSTACLE_DETECTION_THRESHOLD)

  # CHECK IF SENSORS ARE BEING TRIGGERED BY OTHER ROBOTS
  # compute distances to other robots
  robot_pose = robot_poses[robot_id]
  x = robot_pose[X]
  y = robot_pose[Y]
  distances = [np.sqrt(np.square(x- r_p[X]) + np.square(y - r_p[Y])) for r_p in robot_poses]
  # vectors from current robot to all the other robots
  vectors =  [r_p[0:2] - robot_pose[0:2] for r_p in robot_poses]

  pi = np.pi
  sensor_angles = [-pi/2., -pi/4., 0., pi/4, pi/2.]

  # check if the front sensor detects something
  # for each sensor, if there is a robot infront of it, ignore it (by setting the distance it measures to max distance)
  for i in range(len(sensor_angles)):
    if robot_infront_of_sensor(i, sensor_angles[i], robot_poses, distances, vectors, robot_id):
      sens_inp[i] = np.tanh(MAX_DISTANCE)

  # if robot_infront_of_sensor(FRONT, 0., robot_poses, distances, vectors, robot_id):
  #   print("ROBOT INFRONT OF ROBOT: ", robot_id)
  # else:
  #   print("not detected: ", robot_id)
  #   print(".")

  # AVOID OBSTACLES

  # if no sensors detect an obstacle, return nothing
  obstacle = False
  for sensor in sens_inp:
    if sensor < detection_level:
      obstacle = True
      break
  
  if not obstacle:
    return u, w

  # OBSTACLE DETECTED
  obstacle_closeness = 1 - min(sens_inp)
  sensors_detected = np.array(["right", "front_right", "front", "front_left", "left"])[sens_inp < detection_level]
  test_rob_id = 50
  if robot_id == test_rob_id:
    print(sensors_detected)
    print(sens_inp[sens_inp < detection_level])
    print(["right", "front_right", "front", "front_left", "left"])
    print(sens_inp)
    print("------")

  # could have a rule if the formation centre behind the robot, go to it, but this might discourage split and merge
  
  # find the sensor that points in the direction closest to the formation: (sensors are pi/4 apart, between -pi/2 and pi/2)
  to_formation_vector = formation_pose[0:2] - robot_pose[0:2]
  to_formation_angle = np.arctan2(to_formation_vector[Y], to_formation_vector[X])

  # corridor rule
  # if the front sensor is much bigger than the front right or front left sensors, just go forward, adjusting slightly front left or front right
  if sens_inp[FRONT] > 0.88 and sens_inp[FRONT_LEFT] > 0.22 and sens_inp[FRONT_RIGHT] > 0.22:
    if robot_id == test_rob_id:
      print("CORRIDOR RULE TRIGGERED")
    w = w - .5*(1.-sens_inp[FRONT_LEFT]) + .5*(1.-sens_inp[FRONT_RIGHT])
    return u, w

  # best direction to turn in based on front side sensors
  closest = (1- sens_inp[RIGHT]) - (1- sens_inp[LEFT])
  direction = 1. if sens_inp[FRONT_RIGHT] < sens_inp[FRONT_LEFT] else -1.
  if robot_id == test_rob_id:
    print("direction: ", direction)

  # THE FRONT SENSOR DETECTS FIRST, THERE SHOULD BE A RULE FOR THE FRONT SENSOR
  if sens_inp[FRONT] < detection_level:
    if robot_id == test_rob_id:
      print("FRONT TURN AWAY RULE")
    # move either left or right depending on which side front sensor is the furthest away
    w = w + direction * (1- sens_inp[FRONT]) * .75

  # steer when sens_inp is low on the front sides
  if sens_inp[FRONT_LEFT] < detection_level or sens_inp[FRONT_RIGHT] < detection_level:
    if robot_id == test_rob_id:
      print("SIDE STEER RULE")
    w = w + direction * (1- min(sens_inp[FRONT_RIGHT], sens_inp[FRONT_LEFT])) * .65 * obstacle_closeness

  return u, w


def robot_infront_of_sensor(sensor_id, sensor_angle, robot_poses, robot_distances, robot_displacement_vectors, robot_id):
  # check if any of the other robots are infront of the sensor (within a distance of ROBOT_DETECTION_THRESHOLD)
  
  robot_pose = robot_poses[robot_id]

  # if robot_id == 1:
  #   print("distance: ", robot_distances[0])

  found_robot = False

  for i in range(len(robot_poses)):
    if i != robot_id:
      # if robot within range
      if robot_distances[i] < ROBOT_DETECTION_THRESHOLD:
        # if the angle between the two robots (minus the current robot yaw) is within += pi/8, there is a robot infront of the sensor
        angle_to_robot = np.arctan2(robot_displacement_vectors[i][Y], robot_displacement_vectors[i][X])
        angle_to_robot -= robot_pose[YAW]

        # pi/8. is used since the sensors are pi/4. apart on the robot
        if abs(angle_to_robot - sensor_angle) < np.pi/8.:
          found_robot = True
          break
  
  return found_robot

