from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import argparse
import matplotlib.pylab as plt
import matplotlib.patches as patches
import numpy as np
import os
import re
import scipy.signal
import yaml
import random


# Constants used for indexing.
X = 0
Y = 1
YAW = 2

# Constants for occupancy grid.
FREE = 0
UNKNOWN = 1
OCCUPIED = 2

ROBOT_RADIUS = 0.105 / 2.
GOAL_POSITION = np.array([1.5, 1.5], dtype=np.float32)  # Any orientation is good.
START_POSE = np.array([-1.5, -1.5, 0.], dtype=np.float32)
MAX_ITERATIONS = 500


def sample_random_position(occupancy_grid):
  # Sample a valid random position (do not sample the yaw).
  # The corresponding cell must be free in the occupancy grid.
  def is_valid(position):
    # Return whether a position is valid i.e. is it free 
    # and are positions on its circumerence free and in the arena

    is_in_arena = lambda pos : (-2 <= pos[X] <= 2) and (-2 <= pos[Y] <= 2) 
    is_free = lambda pos : occupancy_grid.is_free(pos)

    # Check that points at 4 quadrants of the robot are all in the arena and valid
    r = ROBOT_RADIUS
    for pos in np.add(position, [[r, 0], [-r, 0], [0, r], [0, -r]]):
      if not(is_free(pos) and is_in_arena(pos)):
        return False
    
    # Check that position of robot (center) is free 
    return is_free(position)

  # Generate random position in the arena
  rand_pos = lambda : np.random.uniform(low=-2, high=2, size=2)

  # Generate random positions until get a valid one
  position = rand_pos()
  while not is_valid(position):
    position = rand_pos()

  return position


def adjust_pose(node, final_position, occupancy_grid):
  # Check whether there exists a simple path that links node.pose
  # to final_position. 
  # This function needs to return a new node that has
  # the same position as final_position and a VALID YAW. The yaw is such that
  # there exists an arc of a circle that passes through node.pose and the
  # adjusted final pose. If no such arc exists (e.g., collision) return None.
  # Assume that the robot always goes forward.
  # Feel free to use the find_circle() function below.
  final_pose = node.pose.copy()
  final_pose[:2] = final_position # set x, y of final_pose. YAW as is.
  final_node = Node(final_pose)

  pi, pi2 = np.pi, 2*np.pi

  # Calculate Euclidian distance between 2 points
  d = lambda p1, p2 : np.sqrt((p1[X] - p2[X])**2 + (p1[Y] - p2[Y])**2)

  def get_angle(vec):
    # Normalise angle by making it positive so robot is tunring anticlockwise
    # Get an angle for the the vector and ensure its in the range   
    angle = np.arctan2(vec[Y], vec[X])

    # If x is negative then add pi to theta
    if vec[X] < 0:
      angle = angle + np.pi
    
    return (angle + pi2) % pi2

  # Circle passing through node and final_node
  center, radius = find_circle(node, final_node)

  # Arc is valid if the robot can get to the next point by traversing along any arc inside of this arc
  theta_robot = node.pose[YAW]
  
  # Angle of vector from robot node to the to the center of the circle
  theta_rad = get_angle(center - node.pose[:2])

  # For valid arcs: theta_node must be between [theta_node_to_radius - pi/2 , theta_node_to_radius + pi/2]
  if ((theta_rad - pi/2) <= theta_robot <= (theta_rad + pi/2)):
    # if theta_rad - theta_robot > 0:
    #   anticlockwise
    # if theta_rad - theta_robot < 0: 
    #   clockwise
    # else: 
    #   moving in a straight line ahead
    CLOCKWISE = None
    if theta_rad - theta_robot > 0:
      CLOCKWISE = True
    if theta_rad - theta_robot < 0:
      CLOCKWISE = False
    
    # Final node points 
    final_node.pose[YAW] = ((2 * theta_rad) - theta_robot) % pi2 
    # --------------------------------------------------------------------------------------------------

    def find_circle_center_and_radius(node_a, node_b):
      # Find a circle with an arc from node a to node b where node_a and node_b are not at the diameter

      # First create 2 lines of the form y = mx + c that are perpendicular to the tangents at node.pose and final_position
      # The point of intersection of these 2 lines is the center of the circle 
      # Then we can work out phi = the angle of this circle sector 
      # Split this angle into steps and work out new points along the arc at intervals of phi/n and check if they're all valid

      # rotate the tangent vector by pi/2 degress so its now perpendicular to the target i.e. points to center of the circle
      # offset is +/- pi/2 for anti and clockwise rotating robots respectively 
      offset = -pi/2 if CLOCKWISE else pi/2
      # y_a = m_a x + c_a
      # y_b = m_b x + c_b
      m_a = np.tan(node_a.pose[YAW] + offset) 
      m_b = np.tan(node_b.pose[YAW] + offset) 

      # Rearrage simultaneous eqns to get the c of both lines
      # c_a = y_a - m_a x
      # c_b = y_b - m_b x
      c_a = node_a.pose[Y] - (m_a * node_a.pose[X])
      c_b = node_b.pose[Y] - (m_b * node_b.pose[X])

      # Center = Point of intersection between these 2 lines
      center = np.array([(c_b - c_a) / (m_a - m_b), 
                          m_a * ((c_b - c_a) / (m_a - m_b)) + c_a])

      assert (np.abs(d(center, node_a.pose[:2]) - d(center, node_b.pose[:2])) < 0.5) , 'Differences in radii of new cirlce > 0.5'

      # Radius = distance from center to node.pose
      radius = max(np.abs(d(center, node_a.pose[:2])), np.abs(d(center, node_b.pose[:2])))
      return center, radius

    # Check n points along the arc are all in free space -------------------------------------------------------
    # node_a = Initial_node = node
    # node_b = Final_node = final_node
    node_a, node_b = node, final_node

    step_size = 0.1 # 0.1m

    # Circle passing the arc robot actually follows from node to final_node
    center, radius = find_circle_center_and_radius(node_a, node_b)

    # # d_ab = distance from node to final_position
    d_ab = d(node_a.pose[:2], node_b.pose[:2])

    # Angle of the sector in this new cirlce, angle that subtends the arc, phi = sin-1(d_ab / 2*r) * 2
    phi = np.arcsin(d_ab / (2 * radius)) * 2

    # To calculate the number of steps needed, arc_length/step_size
    # Arc length = sector_angle * radius
    arc_length = phi * radius
    n_steps = int(arc_length / step_size)

    # To get points along the arc from a -> b. Find a circle with radius r 
    parametric_formula_point = lambda r, c, theta : ((r * np.cos(theta)) + c[X], (r * np.sin(theta)) + c[Y])
    # Get initial angle by doing theta_a +- pi/2 depending on direction and final angle +- phi depending on the direction of motion
    # If going clockwise: [theta_a + pi/2, theta_a + pi/2 - phi]
    # If going anticlockwise: [theta_a - pi/2, theta_a - pi/2 + phi]
    if CLOCKWISE:
      initial_theta = node_a.pose[YAW] - pi/2.
      final_theta = initial_theta + phi
    else:
      initial_theta = node_a.pose[YAW] + pi/2.
      final_theta = initial_theta - phi
      

    # Get points along the arc and check if theyre valid
    for theta in np.linspace(initial_theta, final_theta, num=n_steps):
      point = parametric_formula_point(radius, center, theta)

      if not occupancy_grid.is_free(point):
        return None, np.float('inf')

  else:
    return None, np.float('inf')

  return final_node, arc_length 


# Defines an occupancy grid.
class OccupancyGrid(object):
  def __init__(self, values, origin, resolution):
    self._original_values = values.copy()
    self._values = values.copy()
    # Inflate obstacles (using a convolution).
    inflated_grid = np.zeros_like(values)
    inflated_grid[values == OCCUPIED] = 1.
    w = 2 * int(ROBOT_RADIUS / resolution) + 1
    inflated_grid = scipy.signal.convolve2d(inflated_grid, np.ones((w, w)), mode='same')
    self._values[inflated_grid > 0.] = OCCUPIED
    self._origin = np.array(origin[:2], dtype=np.float32)
    self._origin -= resolution / 2.
    assert origin[YAW] == 0.
    self._resolution = resolution

  @property
  def values(self):
    return self._values

  @property
  def resolution(self):
    return self._resolution

  @property
  def origin(self):
    return self._origin

  def draw(self):
    plt.imshow(self._original_values.T, interpolation='none', origin='lower',
               extent=[self._origin[X],
                       self._origin[X] + self._values.shape[0] * self._resolution,
                       self._origin[Y],
                       self._origin[Y] + self._values.shape[1] * self._resolution])
    plt.set_cmap('gray_r')

  def get_index(self, position):
    # compute index into the values table for that position 
    # returns ([0, 384], [0, 384])
    idx = ((position - self._origin) / self._resolution).astype(np.int32)
    if len(idx.shape) == 2:
      idx[:, 0] = np.clip(idx[:, 0], 0, self._values.shape[0] - 1)
      idx[:, 1] = np.clip(idx[:, 1], 0, self._values.shape[1] - 1)
      return (idx[:, 0], idx[:, 1])
    idx[0] = np.clip(idx[0], 0, self._values.shape[0] - 1)
    idx[1] = np.clip(idx[1], 0, self._values.shape[1] - 1)
    return tuple(idx)

  def get_position(self, i, j):
    # converts given indices into a position for the robot in the arena
    return np.array([i, j], dtype=np.float32) * self._resolution + self._origin

  def is_occupied(self, position):
    return self._values[self.get_index(position)] == OCCUPIED

  def is_free(self, position):
    return self._values[self.get_index(position)] == FREE


# Defines a node of the graph.
class Node(object):
  def __init__(self, pose):
    self._pose = pose.copy()
    self._neighbors = []
    self._parent = None
    self._cost = 0.

  @property
  def pose(self):
    return self._pose

  def add_neighbor(self, node):
    self._neighbors.append(node)

  def remove_neighbor(self, node):
    self._neighbors.remove(node)

  @property
  def parent(self):
    return self._parent

  @parent.setter
  def parent(self, node):
    self._parent = node

  @property
  def neighbors(self):
    return self._neighbors

  @property
  def position(self):
    return self._pose[:2]

  @property
  def yaw(self):
    return self._pose[YAW]
  
  @property
  def direction(self):
    return np.array([np.cos(self._pose[YAW]), np.sin(self._pose[YAW])], dtype=np.float32)

  @property
  def cost(self):
      return self._cost

  @cost.setter
  def cost(self, c):
    self._cost = c

  @yaw.setter
  def yaw(self, y):
    self._pose[YAW] = c


def rrt(start_pose, goal_position, occupancy_grid):
  # RRT* builds a graph one node at a time.
  graph = []
  start_node = Node(start_pose)
  final_node = None
  if not occupancy_grid.is_free(goal_position):
    return start_node, final_node
  graph.append(start_node)
  for _ in range(MAX_ITERATIONS): 
    position = sample_random_position(occupancy_grid)
    # With a random chance, draw the goal position.
    if np.random.rand() < .05:
      position = goal_position
    
    # Sort all nodes in graph by distance to new position 
    potential_parent = sorted(((n, np.linalg.norm(position - n.position)) for n in graph), key=lambda x: x[1])
       
    # Collect potential parents within a radius, search_radius of position
    # Filter out the nodes that are very close and nodes that are not aligned
    search_radius = 2
    potential_parents_filtered = []

    u = None
    for n, d in potential_parent:
      # Sorted list so break after a certain distance 
      if d < 1.5:
        # Pick a node at least some distance away 
        # We also verify that the angles are aligned (within pi / 4).
        if d > .2 and n.direction.dot(position - n.position) / d > 0.70710678118:
          potential_parents_filtered.append(n)
      else:
        break
   
    # Find a parent, u, of the new random node v, u -> v
    v = None

    # Considers **all** the nodes within a neighbourhood of the u and find node with min cost
    if potential_parents_filtered:
      # Min node initially the first node
      min_u = potential_parents_filtered[0]
      v, arc_length = adjust_pose(min_u, position, occupancy_grid)
      min_cost = min_u.cost + arc_length

      # Iterate over the nodes to find parent with best cost when attached to position
      for potential_u in potential_parents_filtered:
        potential_v, uv_arc_length = adjust_pose(potential_u, position, occupancy_grid)
        potential_cost = uv_arc_length + potential_u.cost

        # This node is a better choice for parent of v 
        if potential_cost < min_cost:
          min_u, min_cost = potential_u, potential_cost

      u = min_u
      # Parent of v is the node min_u; node in radius of v with minimum cost when attached to v
      v, arc_length = adjust_pose(min_u, position, occupancy_grid)
      
    # No potential parents 
    else: 
      continue
    
    # No valid paths between potential parents 
    if v is None:
      continue

    v.cost = min_cost
    u.add_neighbor(v)
    v.parent = u
    graph.append(v)

    # REWIRE GRAPH - Is it better to make v parent of all nodes in potential_parents_filtered?
    # v = node of sampled position
    # u = parent of v
    # potential_parents_filters = nodes in neighbourhood of v

    # Iterate over potential_parents_filtered \ u: nodes to consider rewiring
    for w in potential_parents_filtered:
      # Old cost of w.parent -> w
      old_cost = w.cost

      # Adjust_pose returns w with YAW and arc_length if went from v -> w   
      w_new, vw_arc_length = adjust_pose(v, w.position, occupancy_grid) 
  
      # If valid path between v and w
      if w_new:
        new_cost = v.cost + vw_arc_length

        # Replace parent of w with v
        print(('old: %f, new: %f')% (old_cost, new_cost))
        if new_cost < old_cost:
          # Update its old parent
          w.parent.remove_neighbor(w)
          # Update its new parent
          w.parent = v
          v.add_neighbor(w)
          # Update yaw
          w.yaw = w_new.yaw
          print('updated!!')
        
    if np.linalg.norm(v.position - goal_position) < .2:
      final_node = v
      break
  return start_node, final_node


def find_circle(node_a, node_b):
  """
  Given 2 nodes, finds a circle passing through the 2 nodes: nodes are at diameter
  """
  def perpendicular(v): 
    w = np.empty_like(v)
    w[X] = -v[Y]
    w[Y] = v[X]
    return w
  db = perpendicular(node_b.direction)
  dp = node_a.position - node_b.position
  t = np.dot(node_a.direction, db)
  if np.abs(t) < 1e-3:
    # By construction node_a and node_b should be far enough apart,
    # so they must be on opposite end of the circle.
    center = (node_b.position + node_a.position) / 2.
    radius = np.linalg.norm(center - node_b.position)
  else:
    radius = np.dot(node_a.direction, dp) / t
    center = radius * db + node_b.position
  return center, np.abs(radius)


def read_pgm(filename, byteorder='>'):
  """Read PGM file."""
  with open(filename, 'rb') as fp:
    buf = fp.read()
  try:
    header, width, height, maxval = re.search(
        b'(^P5\s(?:\s*#.*[\r\n])*'
        b'(\d+)\s(?:\s*#.*[\r\n])*'
        b'(\d+)\s(?:\s*#.*[\r\n])*'
        b'(\d+)\s(?:\s*#.*[\r\n]\s)*)', buf).groups()
  except AttributeError:
    raise ValueError('Invalid PGM file: "{}"'.format(filename))
  maxval = int(maxval)
  height = int(height)
  width = int(width)
  img = np.frombuffer(buf,
                      dtype='u1' if maxval < 256 else byteorder + 'u2',
                      count=width * height,
                      offset=len(header)).reshape((height, width))
  return img.astype(np.float32) / 255.


def draw_solution(start_node, final_node=None):
  ax = plt.gca()

  def draw_path(u, v, arrow_length=.1, color=(.8, .8, .8), lw=1):
    du = u.direction
    plt.arrow(u.pose[X], u.pose[Y], du[0] * arrow_length, du[1] * arrow_length,
              head_width=.05, head_length=.1, fc=color, ec=color)
    dv = v.direction
    plt.arrow(v.pose[X], v.pose[Y], dv[0] * arrow_length, dv[1] * arrow_length,
              head_width=.05, head_length=.1, fc=color, ec=color)
    center, radius = find_circle(u, v)
    du = u.position - center
    theta1 = np.arctan2(du[1], du[0])
    dv = v.position - center
    theta2 = np.arctan2(dv[1], dv[0])
    # Check if the arc goes clockwise.
    if np.cross(u.direction, du).item() > 0.:
      theta1, theta2 = theta2, theta1
    ax.add_patch(patches.Arc(center, radius * 2., radius * 2.,
                             theta1=theta1 / np.pi * 180., theta2=theta2 / np.pi * 180.,
                             color=color, lw=lw))

  points = []
  s = [(start_node, None)]  # (node, parent).
  while s:
    v, u = s.pop()
    if hasattr(v, 'visited'):
      continue
    v.visited = True
    # Draw path from u to v.
    if u is not None:
      draw_path(u, v)
    points.append(v.pose[:2])
    for w in v.neighbors:
      s.append((w, v))

  points = np.array(points)
  plt.scatter(points[:, 0], points[:, 1], s=10, marker='o', color=(.8, .8, .8))
  if final_node is not None:
    plt.scatter(final_node.position[0], final_node.position[1], s=10, marker='o', color='k')
    # Draw final path.
    v = final_node
    while v.parent is not None:
      draw_path(v.parent, v, color='k', lw=2)
      v = v.parent


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description='Uses RRT to reach the goal.')
  parser.add_argument('--map', action='store', default='map', help='Which map to use.')
  args, unknown = parser.parse_known_args()

  # Load map.
  with open(args.map + '.yaml') as fp:
    data = yaml.load(fp)
  img = read_pgm(os.path.join(os.path.dirname(args.map), data['image']))
  occupancy_grid = np.empty_like(img, dtype=np.int8)
  occupancy_grid[:] = UNKNOWN
  occupancy_grid[img < .1] = OCCUPIED
  occupancy_grid[img > .9] = FREE
  # Transpose (undo ROS processing).
  occupancy_grid = occupancy_grid.T
  # Invert Y-axis.
  occupancy_grid = occupancy_grid[:, ::-1]
  occupancy_grid = OccupancyGrid(occupancy_grid, data['origin'], data['resolution'])

  # Run rrt
  import time
  start_time = time.time()
  start_node, final_node = rrt(START_POSE, GOAL_POSITION, occupancy_grid)
  print("--- %s seconds ---" % (time.time() - start_time))

  # Plot environment.
  fig, ax = plt.subplots()
  occupancy_grid.draw()
  plt.scatter(.3, .2, s=10, marker='o', color='green', zorder=1000)
  draw_solution(start_node, final_node)
  plt.scatter(START_POSE[0], START_POSE[1], s=10, marker='o', color='green', zorder=1000)
  plt.scatter(GOAL_POSITION[0], GOAL_POSITION[1], s=10, marker='o', color='red', zorder=1000)
  
  plt.axis('equal')
  plt.xlabel('x')
  plt.ylabel('y')
  plt.xlim([-.5 - 2., 2. + .5])
  plt.ylim([-.5 - 2., 2. + .5])
  plt.show()
  