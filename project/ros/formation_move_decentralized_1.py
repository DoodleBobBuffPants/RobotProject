#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import numpy as np
import matplotlib.pylab as plt
import matplotlib.patches as patches
import sys
import yaml
import os
import re
import random
import rospy

directory = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../python')
sys.path.insert(0, directory)
import rrt

directory = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../ros/velocity_controller')
sys.path.insert(0, directory)
import get_combined_velocity_decentralized as gcv
import rrt_navigation
from init_formations import FORMATION, LEADER_ID

# Robot motion commands:
# http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html
from geometry_msgs.msg import Twist
# Laser scan message:
# http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
from sensor_msgs.msg import LaserScan
# For groundtruth information.
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion

ROBOT_NAMES = ["tb3_0", "tb3_1", "tb3_2", "tb3_3", "tb3_4"]

# This robot's information
ROBOT_NAME = "tb3_1"
ROBOT_ID = 1

# Belief of leader
LEADER_NAME = ROBOT_NAMES[LEADER_ID]
LEADER_POSE = [None, None, None]
LEADER_VELOCITY = [None, None]

GOAL_POSITION = np.array([0, 1.5], dtype=np.float32)
# GOAL_POSITION = np.array([-1, 1.5], dtype=np.float32)
# GOAL_POSITION = np.array([-0.21, 1.3], dtype=np.float32)

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
    self._leader_pose = np.array([np.nan, np.nan, np.nan], dtype=np.float32)
    self._leader_velocity = np.array([np.nan, np.nan], dtype=np.float32)
    self._name = name

  def callback(self, msg):
    # Store belief of current robot and leader, as it is the only state required
    ind_name = [(i, n) for i, n in enumerate(msg.name) if n == self._name or n == LEADER_NAME]
    if not ind_name:
      raise ValueError('Specified name "{}" does not exist.'.format(self._name + " or " + LEADER_NAME))
    for ind, name in ind_name:
        # Pose for current robot
        pose = np.array(self._pose)
        pose[0] = msg.pose[ind].position.x
        pose[1] = msg.pose[ind].position.y
        _, _, yaw = euler_from_quaternion([
            msg.pose[ind].orientation.x,
            msg.pose[ind].orientation.y,
            msg.pose[ind].orientation.z,
            msg.pose[ind].orientation.w])
        pose[2] = yaw

        # Velocity for leader (u,w)
        velocity = np.array(self._leader_velocity)
        velocity[0] = msg.twist[ind].linear.x
        velocity[1] = msg.twist[ind].angular.z

        if name == self._name:
            self._pose = np.array(pose)
        if name == LEADER_NAME:
            self._leader_pose = np.array(pose)
            self._leader_velocity = np.array(velocity)
            
  @property
  def ready(self):
    return not np.isnan(self._pose[0]) and not np.isnan(self._leader_pose[0]) and not np.isnan(self._leader_velocity[0])

  @property
  def pose(self):
    return self._pose

  @property
  def leader_pose(self):
    return self._leader_pose

  @property
  def leader_velocity(self):
    return self._leader_velocity


def run():
  rospy.init_node('obstacle_avoidance')

  # Update control every 50 ms.
  rate_limiter = rospy.Rate(50)

  publisher = rospy.Publisher('/' + ROBOT_NAME + '/cmd_vel', Twist, queue_size=5)
  laser = SimpleLaser(ROBOT_NAME)
  groundtruth = GroundtruthPose(ROBOT_NAME)
  vel_msg = None

  # RRT path
  current_path = None
  
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
    if not laser.ready or not groundtruth.ready:
      rate_limiter.sleep()
      continue

    LEADER_POSE = groundtruth.leader_pose

    # Compute RRT on the leader only
    if ROBOT_ID == LEADER_ID:
        while not current_path:
          start_node, final_node = rrt.rrt(groundtruth.pose, GOAL_POSITION, occupancy_grid)

          # plot rrt path
          # useful debug code
          # fig, ax = plt.subplots()
          # occupancy_grid.draw()
          # plt.scatter(.3, .2, s=10, marker='o', color='green', zorder=1000)
          # rrt.draw_solution(start_node, final_node)
          # plt.scatter(groundtruth.pose[0], groundtruth.pose[1], s=10, marker='o', color='green', zorder=1000)
          # plt.scatter(GOAL_POSITION[0], GOAL_POSITION[1], s=10, marker='o', color='red', zorder=1000)
          
          # plt.axis('equal')
          # plt.xlabel('x')
          # plt.ylabel('y')
          # plt.xlim([-.5 - 2., 2. + .5])
          # plt.ylim([-.5 - 2., 2. + .5])
          # plt.show()

          current_path = rrt_navigation.get_path(final_node)

        # get the RRT velocity for the leader robot
        position = np.array([groundtruth.pose[X] + EPSILON*np.cos(groundtruth.pose[YAW]),
                             groundtruth.pose[Y] + EPSILON*np.sin(groundtruth.pose[YAW])], dtype=np.float32)
        LEADER_VELOCITY = rrt_navigation.get_velocity(position, np.array(current_path, dtype=np.float32))
    else:
        # Get (x,y) velocity from (u,w) velocity for leader
        u = groundtruth.leader_velocity[0]
        w = groundtruth.leader_velocity[1]
        yaw = groundtruth.leader_pose[YAW]
        d = .2
        LEADER_VELOCITY = u * np.array([np.cos(yaw + d*w), np.sin(yaw + d*w)])

    # get the velocity for this robot
    u, w = gcv.get_combined_velocity(groundtruth.pose, LEADER_POSE, LEADER_VELOCITY, laser, ROBOT_ID)

    # cap speed
    vel_msg = Twist()
    vel_msg.linear.x = cap(u, MAX_SPEED)
    vel_msg.angular.z = w

    publisher.publish(vel_msg)

    rate_limiter.sleep()


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
