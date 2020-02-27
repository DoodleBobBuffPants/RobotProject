#!/usr/bin/env python
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import argparse
import numpy as np
import rospy

# Robot motion commands:
# http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html
from geometry_msgs.msg import Twist
# Laser scan message:
# http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
from sensor_msgs.msg import LaserScan
# For groundtruth information.
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion


def braitenberg(front, front_left, front_right, left, right):
  u = 0.5  # [m/s] so when not facing an obstacle the robot accelerates
  w = 0.  # [rad/s] going counter-clockwise.

  # MISSING: Implement a braitenberg controller that takes the range
  # measurements given in argument to steer the robot.
  
  # create an array of sensor input from the robot
  sens_inp = np.array([front, front_left, front_right, left, right])
  
  # smooth with arctan (could use tanh), but then take the gradient of this, so big inputs give little change, but small imputs give the most change
  sens_inp = 1.0 / (1.0 + 3*(sens_inp**2))  # this is the gradient of the tanh function

  # apply weights matrix
  # implement a coward, but we have negatives as we are manipulating w, not the wheel velocities.
  # first row is u weights, second row goes to w weights
  
  # this one still tries to accelerate forwards even when something is infront of it due to the side being positive 
  #weights = np.array([
  #	[-3.0, -1.0, -1.0, 2.5, 2.5],
  #	[0.4, -1.0, 1.0, -4.0, 4.0]
  #])

  # this should be a better form of weights, when it is surrounded, acceleration is negative default
  # a bias on the front sensor to steer slighty left is given so that it does not get stuck still in reverse (it can manage to turn around, test this with extra objects)
  # actually the plus on left and right makes sense, i want it to get smaller when it gets bigger
  weights = np.array([
  	[-0.3, -0.2, -0.2, 0., 0.],
	[0.0, -5.0, 5.0, 0., 0.]
  ])
  # multiply sense inputs by the weights.
  params = np.matmul(weights, sens_inp)

  # U IS VELOCITY NOT ACCELERATION!!!!!!!!!!!!!!!

  # add a function to turn around the robot when it gets very very close
  # prevents the non turning issue when it goes head on into something
  w = w + 40 * (1 / (1 + 100 * (front**2)))
  
  # extract params noet this doesn't consider previous step since we setp them to 0 / 0.5 at the start.. it does do what you think nice.
  u, w = u + params[0], w + params[1]

  # robot having default velocity of 5, so it always keep moving. 

  return u, w


def rule_based(front, front_left, front_right, left, right):
  u = 0.  # [m/s]
  w = 0.  # [rad/s] going counter-clockwise.

  # MISSING: Implement a rule-based controller that avoids obstacles.

  # apply tanh to the input to deal with infinity
  sens_inp = np.array([front, front_left, front_right, left, right])
  sens_inp = np.tanh(sens_inp)
  
  # default velocity
  u = 0.3


  # if all sensors detect obstacles are far away, adjust slightly using a highest absolute value for direction
  if (sens_inp > 0.6).all():
    # move in coward motion, away from the closest object
    return u, sens_inp[2] + sens_inp[4] - (sens_inp[1] + sens_inp[3])

  else:
    # breakout! (mentioned in part C only - rest of rules are as described in part a) 
    # If the front sensor reports a much greater distance than the side sensors, 
    # don't change direction so much, use the left and right sensors for small adjustments
    front = sens_inp[0]
    if front > 0.3 and front > 2.*sens_inp[1] and front > 2.*sens_inp[2]:
      # override w updates and steer using left and right sensors, which should be close enough to detect collisions
      # in tight areas
      if sens_inp[3] < 0.2:
        w = w + 1.5 * (1-sens_inp[3])
      if sens_inp[4] < 0.2:
        w = w - 1.5 * (1-sens_inp[4])
      if sens_inp[1] < 0.2:
        w = w + 1.5 * (1-sens_inp[1])
      if sens_inp[2] < 0.2:
        w = w - 1.5 * (1-sens_inp[2])

      u = 0.1
      return u, w


    # if the right hand side detects an approaching object , alter w to move left
    if sens_inp[2] < 0.6:
      w = w + 4 * (1-sens_inp[2])
  
    # if the left hand slide detects and approaching object, alter w to move to the right
    if sens_inp[1] < 0.6:
      w = w - 4 * (1-sens_inp[1])

    # if robot is very close to the right hand slide, adjust left a little
    if sens_inp[4] < 0.2:
      w = w + 2 * (1-sens_inp[4])

    # if robot is very close to the left hand slide, adjust right a little
    if sens_inp[3] < 0.2:
      w = w - 2 * (1-sens_inp[3])

    # if any front sensor is close, reduce speed
    if (sens_inp[0:3] < 0.3).any():
      u = 0.1

    # turnaround function, if very close to front, turn around
    if sens_inp[0] < 0.2:
      w = 7.5 * (1.0 - sens_inp[0])	

  # todo: make these adaptive to different u values...
  
  return u, w


class SimpleLaser(object):
  def __init__(self):
    rospy.Subscriber('/scan', LaserScan, self.callback)
    self._angles = [0., np.pi / 4., -np.pi / 4., np.pi / 2., -np.pi / 2.]
    self._width = np.pi / 180. * 10.  # 10 degrees cone of view.
    self._measurements = [float('inf')] * len(self._angles)
    self._indices = None

  def callback(self, msg):
    # Helper for angles.
    def _within(x, a, b):
      pi2 = np.pi * 2.
      x %= pi2
      a %= pi2
      b %= pi2
      if a < b:
        return a <= x and x <= b
      return a <= x or x <= b;

    # Compute indices the first time.
    if self._indices is None:
      self._indices = [[] for _ in range(len(self._angles))]
      for i, d in enumerate(msg.ranges):
        angle = msg.angle_min + i * msg.angle_increment
        for j, center_angle in enumerate(self._angles):
          if _within(angle, center_angle - self._width / 2., center_angle + self._width / 2.):
            self._indices[j].append(i)

    ranges = np.array(msg.ranges)
    for i, idx in enumerate(self._indices):
      # We do not take the minimum range of the cone but the 10-th percentile for robustness.
      self._measurements[i] = np.percentile(ranges[idx], 10)

  @property
  def ready(self):
    return not np.isnan(self._measurements[0])

  @property
  def measurements(self):
    return self._measurements


class GroundtruthPose(object):
  def __init__(self, name='turtlebot3_burger'):
    rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback)
    self._pose = np.array([np.nan, np.nan, np.nan], dtype=np.float32)
    self._name = name

  def callback(self, msg):
    idx = [i for i, n in enumerate(msg.name) if n == self._name]
    if not idx:
      raise ValueError('Specified name "{}" does not exist.'.format(self._name))
    idx = idx[0]
    self._pose[0] = msg.pose[idx].position.x
    self._pose[1] = msg.pose[idx].position.y
    _, _, yaw = euler_from_quaternion([
        msg.pose[idx].orientation.x,
        msg.pose[idx].orientation.y,
        msg.pose[idx].orientation.z,
        msg.pose[idx].orientation.w])
    self._pose[2] = yaw

  @property
  def ready(self):
    return not np.isnan(self._pose[0])

  @property
  def pose(self):
    return self._pose
  

def run(args):
  rospy.init_node('obstacle_avoidance')
  avoidance_method = globals()[args.mode]

  # Update control every 100 ms.
  rate_limiter = rospy.Rate(100)
  publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
  laser = SimpleLaser()
  # Keep track of groundtruth position for plotting purposes.
  groundtruth = GroundtruthPose()
  pose_history = []
  with open('/tmp/gazebo_exercise.txt', 'w'):
    pass

  while not rospy.is_shutdown():
    # Make sure all measurements are ready.
    if not laser.ready or not groundtruth.ready:
      rate_limiter.sleep()
      continue

    u, w = avoidance_method(*laser.measurements)
    vel_msg = Twist()
    vel_msg.linear.x = u
    vel_msg.angular.z = w
    publisher.publish(vel_msg)

    # Log groundtruth positions in /tmp/gazebo_exercise.txt
    pose_history.append(groundtruth.pose)
    if len(pose_history) % 10:
      with open('/tmp/gazebo_exercise.txt', 'a') as fp:
        fp.write('\n'.join(','.join(str(v) for v in p) for p in pose_history) + '\n')
        pose_history = []
    rate_limiter.sleep()


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description='Runs obstacle avoidance')
  parser.add_argument('--mode', action='store', default='braitenberg', help='Method.', choices=['braitenberg', 'rule_based'])
  args, unknown = parser.parse_known_args()
  try:
    run(args)
  except rospy.ROSInterruptException:
    pass
