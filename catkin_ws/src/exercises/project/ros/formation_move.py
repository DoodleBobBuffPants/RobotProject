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

robot_names = ["tb3_0", "tb3_1", "tb3_2"]

def move(front1, front2, front3):
  u = 0.  # [m/s]
  w = 0.  # [rad/s] going counter-clockwise.

  if front1 > 0.5 and front2 > 0.5 and front3 > 0.5:
  	u = 0.5

  return u, w


class SimpleLaser(object):
  def __init__(self, name):
    rospy.Subscriber('/' + name + '/scan', LaserScan, self.callback)
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
  

def run():
  rospy.init_node('obstacle_avoidance')

  # Update control every 100 ms.
  rate_limiter = rospy.Rate(100)
  publisher = [None] * len(robot_names)
  laser = [None] * len(robot_names)
  groundtruth = [None] * len(robot_names)
  for i,name in enumerate(robot_names):
  	publisher[i-1] = rospy.Publisher('/' + name + '/cmd_vel', Twist, queue_size=5)
  	laser[i-1] = SimpleLaser(name)
  	groundtruth[i-1] = GroundtruthPose(name)

  while not rospy.is_shutdown():
    # Make sure all measurements are ready.
    if not all(lasers.ready for lasers in laser) or not all(groundtruths.ready for groundtruths in groundtruth):
      rate_limiter.sleep()
      continue

    u, w = move(laser[0].measurements[0], laser[1].measurements[0], laser[2].measurements[0])
    vel_msg = Twist()
    vel_msg.linear.x = u
    vel_msg.angular.z = w
    publisher[0].publish(vel_msg)
    publisher[1].publish(vel_msg)
    publisher[2].publish(vel_msg)

    rate_limiter.sleep()


if __name__ == '__main__':
  try:
    run()
  except rospy.ROSInterruptException:
    pass
