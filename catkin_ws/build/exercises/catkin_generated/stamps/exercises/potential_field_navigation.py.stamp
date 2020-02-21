#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import argparse
import numpy as np
import os
import rospy
import sys

# Robot motion commands:
# http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html
from geometry_msgs.msg import Twist
# For groundtruth information.
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion

# Import the potential_field.py code rather than copy-pasting.
directory = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../python')
sys.path.insert(0, directory)
try:
  import potential_field
except ImportError:
  raise ImportError('Unable to import potential_field.py. Make sure this file is in "{}"'.format(directory))


ROBOT_RADIUS = 0.105 / 2.
CYLINDER_POSITION = np.array([.3, .2], dtype=np.float32)
CYLINDER_RADIUS = .3
GOAL_POSITION = np.array([1.5, 1.5], dtype=np.float32)
MAX_SPEED = .5
EPSILON = .2

USE_RELATIVE_POSITIONS = False

X = 0
Y = 1
YAW = 2


def feedback_linearized(pose, velocity, epsilon):
  u = 0.  # [m/s]
  w = 0.  # [rad/s] going counter-clockwise.

  # MISSING: Implement feedback-linearization to follow the velocity
  # vector given as argument. Epsilon corresponds to the distance of
  # linearized point in front of the robot.

  return u, w


def get_relative_position(absolute_pose, absolute_position):
  relative_position = absolute_position.copy()

  # MISSING: Compute the relative position of absolute_position in the
  # coordinate frame defined by absolute_pose.

  return relative_position


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
    self._pose[X] = msg.pose[idx].position.x
    self._pose[Y] = msg.pose[idx].position.y
    _, _, yaw = euler_from_quaternion([
        msg.pose[idx].orientation.x,
        msg.pose[idx].orientation.y,
        msg.pose[idx].orientation.z,
        msg.pose[idx].orientation.w])
    self._pose[YAW] = yaw

  @property
  def ready(self):
    return not np.isnan(self._pose[0])

  @property
  def pose(self):
    return self._pose


def get_velocity(point_position, goal_position, obstacle_position):
  v_goal = potential_field.get_velocity_to_reach_goal(point_position, goal_position)
  v_avoid = potential_field.get_velocity_to_avoid_obstacles(point_position, [obstacle_position],
                                                            [CYLINDER_RADIUS + ROBOT_RADIUS])
  v = v_goal + v_avoid
  return potential_field.cap(v, max_speed=MAX_SPEED)
  

def run(args):
  rospy.init_node('potential_field_navigation')

  # Update control every 100 ms.
  rate_limiter = rospy.Rate(100)
  publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
  # Keep track of groundtruth position for plotting purposes.
  groundtruth = GroundtruthPose()
  pose_history = []
  with open('/tmp/gazebo_exercise.txt', 'w'):
    pass

  while not rospy.is_shutdown():
    # Make sure all measurements are ready.
    if not groundtruth.ready:
      rate_limiter.sleep()
      continue

    absolute_point_position = np.array([
        groundtruth.pose[X] + EPSILON * np.cos(groundtruth.pose[YAW]),
        groundtruth.pose[Y] + EPSILON * np.sin(groundtruth.pose[YAW])], dtype=np.float32)

    if USE_RELATIVE_POSITIONS:
      point_position = get_relative_position(groundtruth.pose, absolute_point_position)
      goal_position = get_relative_position(groundtruth.pose, GOAL_POSITION)
      obstacle_position = get_relative_position(groundtruth.pose, CYLINDER_POSITION)
      pose = np.array([0., 0., 0.], dtype=np.float32)
    else:
      point_position = absolute_point_position
      goal_position = GOAL_POSITION
      obstacle_position = CYLINDER_POSITION
      pose = groundtruth.pose

    # Get velocity.
    v = get_velocity(point_position, goal_position, obstacle_position)

    u, w = feedback_linearized(pose, v, epsilon=EPSILON)
    vel_msg = Twist()
    vel_msg.linear.x = u
    vel_msg.angular.z = w
    publisher.publish(vel_msg)

    # Log groundtruth positions in /tmp/gazebo_exercise.txt
    pose_history.append(np.concatenate([groundtruth.pose, absolute_point_position], axis=0))
    if len(pose_history) % 10:
      with open('/tmp/gazebo_exercise.txt', 'a') as fp:
        fp.write('\n'.join(','.join(str(v) for v in p) for p in pose_history) + '\n')
        pose_history = []
    rate_limiter.sleep()


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description='Runs potential field navigation')
  args, unknown = parser.parse_known_args()
  try:
    run(args)
  except rospy.ROSInterruptException:
    pass
