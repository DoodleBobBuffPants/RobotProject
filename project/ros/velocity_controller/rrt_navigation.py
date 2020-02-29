#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import numpy as np
import os
import sys

directory = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../python')
sys.path.insert(0, directory)
try:
  import rrt
except ImportError:
  raise ImportError('Unable to import rrt.py. Make sure this file is in "{}"'.format(directory))

# Robot motion commands:
# http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html
from geometry_msgs.msg import Twist
# Occupancy grid.
from nav_msgs.msg import OccupancyGrid
# Position.
from tf import TransformListener
# Goal.
from geometry_msgs.msg import PoseStamped
# Path.
from nav_msgs.msg import Path
# For pose information.
from tf.transformations import euler_from_quaternion

SPEED = .5
EPSILON = .1

X = 0
Y = 1
YAW = 2


def feedback_linearized(pose, velocity, epsilon):

  # u = velocity[X]*np.cos(pose[YAW]) + velocity[Y]*np.sin(pose[YAW])
  # w = (velocity[Y]*np.cos(pose[YAW]) - velocity[X]*np.sin(pose[YAW]))/epsilon

  # return u, w

  u = 0.  # [m/s]
  w = 0.  # [rad/s] going counter-clockwise.

  # MISSING: Implement feedback-linearization to follow the velocity
  # vector given as argument. Epsilon corresponds to the distance of
  # linearized point in front of the robot.

  # check if velocity is near zero:
  threshold = 0.01
  if np.sqrt(velocity[X]**2 + velocity[Y]**2) < threshold:
    return u, w 

  d_xp_t = velocity[X]
  d_yp_t = velocity[Y]

  # theta
  theta = pose[YAW]

  u = d_xp_t * np.cos(theta) + d_yp_t * np.sin(theta)
  w = (d_yp_t * np.cos(theta) - d_xp_t * np.sin(theta))/epsilon

  return u, w


def get_velocity(position, path_points):
  v = np.zeros_like(position)
  if len(path_points) == 0:
    return v
  # Stop moving if the goal is reached.
  if np.linalg.norm(position - path_points[-1]) < .2:
    return v

  # MISSING: Return the velocity needed to follow the
  # path defined by path_points. Assume holonomicity of the
  # point located at position.

  minp = path_points[0]
  mind = np.linalg.norm(path_points[0]-position)
  nextp = path_points[1]

  for u, v in zip(path_points[1:], path_points[2:]):
    if np.linalg.norm(u-position) < mind:
      minp = u
      mind = np.linalg.norm(u-position)
      nextp = v

  vec = nextp - position
  vec = vec / np.linalg.norm(vec)

  return SPEED * vec
  

def get_path(final_node):
  # Construct path from RRT solution.
  if final_node is None:
    return []
  path_reversed = []
  path_reversed.append(final_node)
  while path_reversed[-1].parent is not None:
    path_reversed.append(path_reversed[-1].parent)
  path = list(reversed(path_reversed))
  # Put a point every 5 cm.
  distance = 0.05
  offset = 0.
  points_x = []
  points_y = []
  for u, v in zip(path, path[1:]):
    center, radius = rrt.find_circle(u, v)
    du = u.position - center
    theta1 = np.arctan2(du[1], du[0])
    dv = v.position - center
    theta2 = np.arctan2(dv[1], dv[0])
    # Check if the arc goes clockwise.
    clockwise = np.cross(u.direction, du).item() > 0.
    # Generate a point every 5cm apart.
    da = distance / radius
    offset_a = offset / radius
    if clockwise:
      da = -da
      offset_a = -offset_a
      if theta2 > theta1:
        theta2 -= 2. * np.pi
    else:
      if theta2 < theta1:
        theta2 += 2. * np.pi
    angles = np.arange(theta1 + offset_a, theta2, da)
    offset = distance - (theta2 - angles[-1]) * radius
    points_x.extend(center[X] + np.cos(angles) * radius)
    points_y.extend(center[Y] + np.sin(angles) * radius)
  return zip(points_x, points_y)
  