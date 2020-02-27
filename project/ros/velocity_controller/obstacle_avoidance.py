#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import numpy as np

# Robot motion commands:
# http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html
from geometry_msgs.msg import Twist
# Laser scan message:
# http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
from sensor_msgs.msg import LaserScan
# For groundtruth information.
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion

SPEED = .5

def braitenberg(front, front_left, front_right, left, right):
  u = 0.  # [m/s]
  w = 0.  # [rad/s] going counter-clockwise.

  # MISSING: Implement a braitenberg controller that takes the range
  # measurements given in argument to steer the robot.

  u = 0.8 * np.tanh(front+front_right+front_left+0.5*(right+left))
  w = np.pi*(1-np.tanh(front))
  return u, w


def rule_based(front, front_left, front_right, left, right):
  u = 0.  # [m/s]
  w = 0.  # [rad/s] going counter-clockwise.

  # MISSING: Implement a rule-based controller that avoids obstacles.

  u = SPEED
  w = 0
  if front < 1:
  	u = 0.
  	w = np.pi/5.

  return u, w
