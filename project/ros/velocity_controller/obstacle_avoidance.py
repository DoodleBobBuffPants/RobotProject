from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from rrt_navigation import feedback_linearized
from init_formations import INITIAL_YAW

import numpy as np


OBSTACLE_DETECTION_THRESHOLD = 2.
SENSOR_ROBOT_TOLERANCE = 0.08

LEADER_OBS_DETECT_THRESHOLD = 0.4
ROBOT_DETECTION_THRESHOLD = 3.5
RIGHT, FRONT_RIGHT, FRONT, FRONT_LEFT, LEFT = 0, 1, 2, 3, 4
MAX_DISTANCE = 3.5 # max distance measured by the robot sensor
X = 0
Y = 1
YAW = 2

# Sigmoid constants
XSCALE = 4.
XTRANSLATE = - 1.5 

# Feedback linearisation epsilon
EPSILON = .1

print_robot_id = -1

pi = np.pi
sensor_angles = [-pi/2., -pi/4., 0., pi/4, pi/2.]
relative_sensor_positions = np.array([[1., 0.], [1., 1.], [0., 1.], [-1., 1.], [-1., 0.]])

def sigmoid(x):
  x = XSCALE * (x + XTRANSLATE)
  return 1. / (1. + np.exp(-x))

def braitenburg(robot_poses, robot_id, front, front_left, front_right, left, right):
  u = .1  # [m/s] so when not facing an obstacle the robot accelerates
  w = 0.  # [rad/s] going counter-clockwise.
  
  # create an array of sensor input from the robot
  sens_inp = np.array([right, front_right, front, front_left, left])

  # detect the other robots within the sensor range
  sens_inp = detect_robot_presence(robot_poses, sens_inp, robot_id)

  # min distance of the front sensors
  min_distance = np.min(sens_inp[1:4])
  
  # smooth with arctan (could use tanh), but then take the gradient of this, so big inputs give little change, but small imputs give the most change
  sens_inp = 1.0 / (1.0 + 3*(sens_inp**2))  # this is the gradient of the tanh function

  # rearrange for braitenburg
  sens_inp = np.array([sens_inp[FRONT],sens_inp[FRONT_LEFT],sens_inp[FRONT_RIGHT],sens_inp[LEFT],sens_inp[RIGHT]])

  # Direction to turn from front sensor
  if robot_id < len(robot_poses) // 2:
    direction = 1.
  else:
    direction = -1.

  # this should be a better form of weights, when it is surrounded, acceleration is negative default
  # a bias on the front sensor to steer slighty left is given so that it does not get stuck still in reverse (it can manage to turn around, test this with extra objects)
  # actually the plus on left and right makes sense, i want it to get smaller when it gets bigger
  weights = np.array([
    # front front left front right left right
  	[-0.3, -0.2, -0.2, 0., 0.],
	  [0.5 * direction, -5.0, 5.0, 0.3, 0.3]
  ])

  # multiply sense inputs by the weights.
  params = np.matmul(weights, sens_inp)

  # add a function to turn around the robot when it gets very very close
  # prevents the non turning issue when it goes head on into something
  if robot_id == 4 and front < 0.4:
    print("front: {}  | front_left: {}  |  front_right: {}  ".format(front, front_left, front_right))
  if front < 0.5 and front_left > 0.2 and front_right > 0.2:
    w = w + (40 * (1 / (1 + 100 * (front**2))) * direction)
  
  # extract params noet this doesn't consider previous step since we setp them to 0 / 0.5 at the start.. it does do what you think nice.
  u, w = u + params[0], w + params[1]

  # robot having default velocity of 5, so it always keep moving. 

  return u, w, min_distance


def get_obstacle_velocity(robot_poses, robot_id, front, front_left, front_right, left, right):
  """TODOS:
  - differentiate between obstacles and moving robots
  - get smooth distances
  - get vectors for each sensor
  - weighted sum of the vectors
  - min max scale
  """
  sens_inp = np.array([right, front_right, front, front_left, left])

  # print('before seeing other robots: \n', sens_inp)
  # account for other robots infront of the sensors
  sens_inp = detect_robot_presence(robot_poses, sens_inp, robot_id)
  # print('after seeing other robots: \n', sens_inp)

  robot_pose = robot_poses[robot_id]

  # smooth sensor inputs with tanh
  sens_inp = np.tanh(sens_inp)
  if robot_id == print_robot_id:
    print('sensors after smoothing other robots: \n', sens_inp)
    print("robot pose: ", robot_pose)

  # generate sensor positions in global frame
  sensor_positions = np.zeros_like(relative_sensor_positions)
  # print("avoidance centre: ", avoidance_centre)
  theta = robot_pose[YAW] - INITIAL_YAW
  for i in range(len(relative_sensor_positions)):
    # Rotate
    sensor_positions[i] = np.matmul([[np.cos(theta), -np.sin(theta)],
                                  [np.sin(theta),  np.cos(theta)]], relative_sensor_positions[i])
                                  
    # Translate
    sensor_positions[i] = robot_pose[:2] + sensor_positions[i]

  # if robot_id ==print_robot_id:
  #   print("transformed positions: ", sensor_positions)
  #   print()

  # sensor weights
  s_weights = [0.3, 2.2, 0.8, 2.2, 0.3]
  # s_weights = [0.2, 1.9, 1.6, 1.9, 0.2]


  # RULE: if front sensor fires and side sensors are clear, robot needs to turn to avoid narrow obstacle
  if sens_inp[FRONT] < 0.6 and sens_inp[FRONT_RIGHT] > 0.85 and sens_inp[FRONT_LEFT] > 0.85:
    print("rule fired")
    # sensor_positions[FRONT] = sensor_positions[FRONT] + 1.5 * (sensor_positions[LEFT] if True else sensor_positions[RIGHT])
    sensor_positions[FRONT] = sensor_positions[LEFT] if sens_inp[FRONT_LEFT] < sens_inp[FRONT_RIGHT] else sensor_positions[RIGHT]
    s_weights[FRONT] = 2.2
    # print("front after: ", sensor_positions[FRONT])
    # print("front after: {}".format(sensor_positions[FRONT])


  # # RULE: if the front sensor can see far into the distance and the other sensors are small, just go forward
  # if sens_inp[FRONT] > 0.88 and sens_inp[FRONT_LEFT] > 0.3 and sens_inp[FRONT_RIGHT] > 0.3:
  #   s_weights = np.zeros_like(s_weights)

  # get vectors from each sensor
  sensor_to_robot_vecs = robot_pose[0:2] - sensor_positions

  # normalise the sensor vectors
  for i in range(len(sensor_to_robot_vecs)):
    sensor_to_robot_vecs[i] = sensor_to_robot_vecs[i] / np.linalg.norm(sensor_to_robot_vecs[i])
    
    if sens_inp[i] > np.tanh(OBSTACLE_DETECTION_THRESHOLD):
      sensor_to_robot_vecs[i] = np.zeros_like(sensor_to_robot_vecs[i])

  # weight by distance to nearest obstacle
  weight = 1. - sens_inp

  for i in range(len(sensor_to_robot_vecs)):
    sensor_to_robot_vecs[i] = sensor_to_robot_vecs[i] * weight[i] * s_weights[i]

  # if robot_id ==print_robot_id:
  #   print('robot %s sensor vecs: ' % (robot_id))
  #   print('   RIGHT ', sensor_to_robot_vecs[RIGHT])
  #   print('   FRONT_RIGHT ', sensor_to_robot_vecs[FRONT_RIGHT])
  #   print('   FRONT ', sensor_to_robot_vecs[FRONT])
  #   print('   FRONT_LEFT ', sensor_to_robot_vecs[FRONT_LEFT])
  #   print('   LEFT ', sensor_to_robot_vecs[LEFT])

  # sum the vectors together
  weighted_sum = np.sum(sensor_to_robot_vecs, axis=0)

  if robot_id == print_robot_id:
    print('mag: ', np.linalg.norm(weighted_sum))
  # smooth weighted sums magnitude using sigmoid function
  magnitude = np.linalg.norm(weighted_sum)
  if robot_id == print_robot_id:
    print('magnitued before: ', magnitude)
  if magnitude < 1e-2:
    weighted_sum = np.zeros_like(weighted_sum)
  else:
    if robot_id ==print_robot_id:
      print('s(m): ', sigmoid(magnitude))
    # weighted_sum = weighted_sum * (sigmoid(magnitude) / magnitude)
    weighted_sum = weighted_sum / 3.3

  # if robot_id ==0:
  #   print('final:', weighted_sum)
  #   print('mag: ', np.linalg.norm(weighted_sum))  
  #   print()
  return weighted_sum
  # return np.array([-1, 0]) * (1-sens_inp[FRONT])
  
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
  if robot_id == 0:
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
    # print("CORRIDOR RULE TRIGGERED")
    w = w - .5*(1.-sens_inp[FRONT_LEFT]) + .5*(1.-sens_inp[FRONT_RIGHT])
    return u, w

  # best direction to turn in based on front side sensors
  closest = (1- sens_inp[RIGHT]) - (1- sens_inp[LEFT])
  direction = 1. if sens_inp[FRONT_RIGHT] < sens_inp[FRONT_LEFT] else -1.
  print("direction: ", direction)

  # THE FRONT SENSOR DETECTS FIRST
  if sens_inp[FRONT] < detection_level:
    print("FRONT TURN AWAY RULE")
    # move either left or right depending on which side front sensor is the furthest away
    w = w + direction * (1- sens_inp[FRONT]) * .75

  # steer when sens_inp is low on the front sides
  if sens_inp[FRONT_LEFT] < detection_level or sens_inp[FRONT_RIGHT] < detection_level:
    print("SIDE STEER RULE")
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

def detect_robot_presence(robot_poses, raw_sensor_input, robot_id):
  # CHECK IF SENSORS ARE BEING TRIGGERED BY OTHER ROBOTS
  # compute distances to other robots
  robot_pose = robot_poses[robot_id]
  x = robot_pose[X]
  y = robot_pose[Y]
  distances = [np.sqrt(np.square(x- r_p[X]) + np.square(y - r_p[Y])) for r_p in robot_poses]
  # vectors from current robot to all the other robots
  vectors = [r_p[0:2] - robot_pose[0:2] for r_p in robot_poses]

  pi = np.pi
  sensor_angles = [-pi/2., -pi/4., 0., pi/4, pi/2.]

  # check if the front sensor detects something
  # for each sensor, if there is a robot infront of it, ignore it (by setting the distance it measures to max distance)
  for i in range(len(sensor_angles)):
    if robot_infront_of_sensor(i, sensor_angles[i], raw_sensor_input[i], robot_poses, distances, vectors, robot_id):
      raw_sensor_input[i] = MAX_DISTANCE

  return raw_sensor_input

def robot_infront_of_sensor(sensor_id, sensor_angle, raw_sensor_value, robot_poses, robot_distances, robot_displacement_vectors, robot_id):
  # check if any of the other robots are infront of the sensor (within a distance of ROBOT_DETECTION_THRESHOLD)
  
  robot_pose = robot_poses[robot_id]

  # if robot_id == 1:
  # print("distance: ", robot_distances[0])

  found_robot = False

  for i in range(len(robot_poses)):
    if i != robot_id:
      # if robot within range and sensor_value is similar to robot_distance[i]
      if robot_distances[i] < ROBOT_DETECTION_THRESHOLD and abs(raw_sensor_value - robot_distances[i]) < SENSOR_ROBOT_TOLERANCE:
        # if the angle between the two robots (minus the current robot yaw) is within += pi/8, there is a robot infront of the sensor
        angle_to_robot = np.arctan2(robot_displacement_vectors[i][Y], robot_displacement_vectors[i][X])
        angle_to_robot -= robot_pose[YAW]

        # pi/8. is used since the sensors are pi/4. apart on the robot
        if abs(angle_to_robot - sensor_angle) < np.pi/8.:
          found_robot = True
          break
  
  return found_robot

