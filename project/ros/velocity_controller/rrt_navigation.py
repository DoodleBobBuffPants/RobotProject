from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import numpy as np
import rrt

SPEED = .5

X = 0
Y = 1
YAW = 2


def feedback_linearized(pose, velocity, epsilon):

  u = 0.  # [m/s]
  w = 0.  # [rad/s] going counter-clockwise.

  # check if velocity is near zero:
  threshold = 0.01
  if np.sqrt(velocity[X]**2 + velocity[Y]**2) < threshold:
    return u, w 

  d_xp_t = velocity[X]
  d_yp_t = velocity[Y]
  theta = pose[YAW]

  u = d_xp_t*np.cos(theta) + d_yp_t*np.sin(theta)
  w = (d_yp_t*np.cos(theta) - d_xp_t*np.sin(theta))/epsilon

  return u, w


def get_velocity(position, path_points):
  # v = np.zeros_like(position)
  # if len(path_points) == 0:
  #   return v
  # # Stop moving if the goal is reached.
  # if np.linalg.norm(position - path_points[-1]) < .2:
  #   return v

  # # MISSING: Return the velocity needed to follow the
  # # path defined by path_points. Assume holonomicity of the
  # # point located at position.

  # minp = path_points[0]
  # mind = np.linalg.norm(path_points[0]-position)
  # nextp = path_points[1]

  # for u, v in zip(path_points[1:], path_points[2:]):
  #   if np.linalg.norm(u-position) < mind:
  #     minp = u
  #     mind = np.linalg.norm(u-position)
  #     nextp = v

  # vec = nextp - position
  # vec = vec / np.linalg.norm(vec)

  # return SPEED * vec
  v = np.zeros_like(position)
  if len(path_points) == 0:
    return v
  # Stop moving if the goal is reached.
  if np.linalg.norm(position - path_points[-1]) < .2:
    return v

  # MISSING: Return the velocity needed to follow the
  # path defined by path_points. Assume holonomicity of the
  # point located at position.

  # the robot is somewhere near the position
  # this position is some point infront of the robot at distance epsilon.
  # this point can be moved holonomically, the robot is pulled by a rod.
  # this allows us to approximate holonomic motion of the robot
  # by formulating its control inputs as functions of the velocity of the
  # point position (lecture 3, feed back linearization slides)...
  #print(position)

  # so now we're gonna DRAGGGGGGG that point to point in the best direction of the robot
  # our method for doing this is compute the euclidean distance between all the points
  # so that we can figure out which point is the best point to drag it to.
  #print(path_points)

  # to figure out which point to pull it to, we first compute the distance between the position and all the points
  # that are on the curve
  distance = lambda pos, path_point: np.sqrt(np.square(pos[X] - path_point[X]) + np.square(pos[Y] - path_point[Y]))
  distances = np.array([distance(position, path_p) for path_p in path_points])

  if len(path_points) > 1:
    # now find the index of the two points it is closest to
    closest_index = np.argmin(distances)
    close_1_distance = distances[closest_index]
    # change the distance value at the closest element so that it is not the min anymore
    distances[closest_index] = np.max(distances)
    #print("closest: ", closest_index)
    #print("distances: ", distances)
    second_closest_index = np.argmin(distances)

    # pick the index that is the furthest along the path
    path_index = closest_index if closest_index > second_closest_index else second_closest_index
    
    if path_index < len(path_points) - 1:
      path_index += 1 # pick the next one along
    
    # get the point on the path
    path_point = path_points[path_index]

  else:
    # the path only has one element in the list
    path_point = path_points[0]

  v = path_point - position
  k = 1.

  return v * k
  

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
  