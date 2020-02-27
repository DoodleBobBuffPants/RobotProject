#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import argparse
import numpy as np
import matplotlib.pylab as plt
import matplotlib.patches as patches
import scipy.signal
import sys
import yaml
import os
import re
import random
import rospy


directory = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../python')
sys.path.insert(0, directory)

directory = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../ros/velocity_controller')
sys.path.insert(0, directory)
import get_combined_velocity as gcv
import rrt_navigation
from init_formations import FORMATION

try:
  import rrt
except ImportError:
  raise ImportError('Unable to import rrt.py. Make sure this file is in "{}"'.format(directory))

# Robot motion commands:
# http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html
from geometry_msgs.msg import Twist
# Laser scan message:
# http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
from sensor_msgs.msg import LaserScan
# For groundtruth information.
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion

robot_names = ["tb3_0", "tb3_1", "tb3_2", "tb3_3", "tb3_4"]
GOAL_POSITION = np.array([0, 1.5], dtype=np.float32)
EPSILON = .1
MAX_SPEED = 0.25

X = 0
Y = 1
YAW = 2

FREE = 0
UNKNOWN = 1
OCCUPIED = 2

def cap(v, max_speed):
  n = np.linalg.norm(v)
  if n > max_speed:
    return v / n * max_speed
  return v

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

  # Update control every 50 ms.
  rate_limiter = rospy.Rate(50)
  
  publishers = [None] * len(robot_names)
  lasers = [None] * len(robot_names)

  # ground truth pose of robots
  groundtruth_poses = [None] * len(robot_names)

  # the path RRT get_navigation returns
  current_path = [None] * len(robot_names)

  vel_msgs = [None] * len(robot_names)
  for i,name in enumerate(robot_names):
  	publishers[i] = rospy.Publisher('/' + name + '/cmd_vel', Twist, queue_size=5)
  	lasers[i] = SimpleLaser(name)
  	groundtruth_poses[i] = GroundtruthPose(name)

  stop_msg = Twist()
  stop_msg.linear.x = 0.
  stop_msg.angular.z = 0.

  # Load map. (in here so it is only computed once)
  with open(os.path.expanduser('~/catkin_ws/src/exercises/project/python/map.yaml')) as fp:
    data = yaml.load(fp)
  img = rrt.read_pgm(os.path.expanduser('~/catkin_ws/src/exercises/project/python/map.pgm'), data['image'])
  occupancy_grid = np.empty_like(img, dtype=np.int8)
  occupancy_grid[:] = UNKNOWN
  occupancy_grid[img < .1] = OCCUPIED
  occupancy_grid[img > .9] = FREE
  # Transpose (undo ROS processing).
  occupancy_grid = occupancy_grid.T
  # Invert Y-axis.
  occupancy_grid = occupancy_grid[:, ::-1]
  occupancy_grid = rrt.OccupancyGrid(occupancy_grid, data['origin'], data['resolution'])

  while not rospy.is_shutdown():
    # Make sure all measurements are ready.
    if not all(laser.ready for laser in lasers) or not all(groundtruth.ready for groundtruth in groundtruth_poses):
      rate_limiter.sleep()
      continue

    # get our RRT velocities
    rrt_velocities = []
    for i,_ in enumerate(robot_names):
      #do rrt once
      while not current_path[i]:
        robot_goal = GOAL_POSITION#-FORMATION[i]
        start_node, final_node = rrt.rrt(groundtruth_poses[i].pose, robot_goal, occupancy_grid)

        current_path[i] = rrt_navigation.get_path(final_node)
      
      #stop if at goal
      if np.linalg.norm(groundtruth_poses[i].pose[:2] - robot_goal) < .05:
        vel_msgs[i] = stop_msg
        continue

      position = np.array([groundtruth_poses[i].pose[X] + EPSILON*np.cos(groundtruth_poses[i].pose[YAW]),
    		               groundtruth_poses[i].pose[Y] + EPSILON*np.sin(groundtruth_poses[i].pose[YAW])], dtype=np.float32)
      v = rrt_navigation.get_velocity(position, np.array(current_path[i], dtype=np.float32))

      rrt_velocities.append(v)

    rrt_velocities = np.array(rrt_velocities)

    # get our combined velocity for each robot
    # get poses from ground truth objects
    robot_poses = np.array([groundtruth_poses[i].pose for i in range(len(groundtruth_poses))])
    us, ws = gcv.get_combined_velocities(robot_poses=robot_poses, rrt_velocities=rrt_velocities, lasers=lasers)

    # cap and mod angle
    for i in range(len(us)):
      us[i] = cap(us[i], MAX_SPEED)

      # get results and publish them
      vel_msgs[i] = Twist()
      vel_msgs[i].linear.x = us[i]
      vel_msgs[i].angular.z = ws[i]

    for i,_ in enumerate(robot_names):
      publishers[i].publish(vel_msgs[i])

    rate_limiter.sleep()


if __name__ == '__main__':
  try:
    run()
  except rospy.ROSInterruptException:
    pass
