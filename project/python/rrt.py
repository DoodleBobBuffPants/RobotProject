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
  position = np.zeros(2, dtype=np.float32)

  # MISSING: Sample a valid random position (do not sample the yaw).
  # The corresponding cell must be free in the occupancy grid.

  # This is implementing the first primitive of the RRT algorithm: sample configurations in free C-space.
  # C free = C \ C obs, the occupancy_grid function is giving us a nice 

  # sample function not including yaw, capped at dimensions of the arena
  sample_pos = lambda: np.random.uniform(low=-2, high=3.5, size=2)

  position = sample_pos()
  while not is_valid(position, occupancy_grid):
    position = sample_pos()

  return position

 # function to check that this position is valid for the robot to be in (check points at the robot's circumpherence)
def is_valid(position, occupancy_grid):
  # function for checking if a position is valid
  pos_is_valid = lambda pos: occupancy_grid.is_free(pos) and not occupancy_grid.is_occupied(pos)

  # check 4 corners (and the centre itself) (for now), this can be extended
  r = ROBOT_RADIUS
  rob_radius_points = [[0,0], [0, r], [0, -r], [r, 0], [-r, 0]]
  for radius_marker in rob_radius_points:
    # get radius position
    radius_pos = position + radius_marker
    # check if it is valid
    if not pos_is_valid(radius_pos):
      return False
    
    # no obstacles found at this position, return true
    return True


def adjust_pose(node, final_position, occupancy_grid):
  final_pose = node.pose.copy()
  final_pose[:2] = final_position
  # final node is constructed using the final_position x and y components
  # the aim of this task is to find out if it is valid, and if so compute a corresponding valid YAW
  final_node = Node(final_pose)

  # MISSING: Check whether there exists a simple path that links node.pose
  # to final_position. This function needs to return a new node that has
  # the same position as final_position and a valid yaw. The yaw is such that
  # there exists an arc of a circle that passes through node.pose and the
  # adjusted final pose. If no such arc exists (e.g., collision) return None.
  # Assume that the robot always goes forward.
  # Feel free to use the find_circle() function below.

  # use the current node and the next node, check if there is a circle
  # if no arc return None
  # can the circle function return more than one value... who knowns
  # can you not I do not

  # get the circle that connects the robot to the result node
  centre, radius = find_circle(node, final_node)

  # theta of the robot from the starting node YAW
  theta_robot = node.pose[YAW]

  # compute if theta direction is valid given the circle
  # first get the vector from the robot to the centre
  radius_vec = - node.pose[:2] + centre

  # get radius angle
  theta_rad = get_angle_of_vector(radius_vec)
  
  # rotate the vector by -pi/2 to get the tangent of the circle
  p_div_2 = np.pi / 2.
  tangent_vec = np.matmul([
    [np.cos(p_div_2), np.sin(p_div_2)],
    [-np.sin(p_div_2), np.cos(p_div_2)]
  ],
   radius_vec)

  # check if the robot's  yaw is within pi/4 rads either side of the radius angle
  # if not, the arc is not valid, return None
  pi_div2 = np.pi / 2
  # the robot must be within this range otherwise it would have to modify w as it was moving between points
  # to smoothly curve between them, here we are just assuming a constant w between positions I think.
  if theta_rad - pi_div2 <= theta_robot <= theta_rad + pi_div2:
    # the angle is valid, compute the new yaw

    # see paper write up, the new yaw is 2 * theta_radius - theta_robot
    # if (thetar - theta robot) > 0, its going anticlockwise, yaw = theta r + (theta r - theta robot) = 2 theta r - theta robot
    # if (thetar - theta robot) < 0, its going clockwise, yaw = theta r - (theta robot - theta r) = 2 theta r - theta robot
    final_pose[YAW] = (2 * theta_rad) - theta_robot

    # now we need to sample this arc to check that there are no collisions
    # make the image much bigger with more obstacles so that in the report, you can show that your rrt really does go around the obstacles.

    # get the new centre of the circle given the arc that the robot actually will traverse
    centre = get_new_circle_centre(node.pose, final_pose)

    dist_between_points = lambda a, b: np.sqrt(np.square(a[X]-b[X]) + np.square(a[Y]-b[Y]))
    # next get the angle that subtends the arc of the circle
    #print(dist_between_points(centre, final_position))
    #print(dist_between_points(node.position, centre))
    assert dist_between_points(centre, final_position) - dist_between_points(node.position, centre) < 0.001

    # opp = distance between final position and starting position
    # hype = distance between centre and either of the poses
    opp = dist_between_points(final_position, node.position) / 2.
    hyp = dist_between_points(final_position, centre)

    # angle that subtends the arc is two times the angle in the triangle defined using opp and hyp
    phi = np.arcsin(opp/hyp) * 2.
    #print("ratio:", opp/hyp)
    #print("phi: ", phi)
    # if np.isnan(phi):
    #   phi = np.pi

    # step size at which to check points on the line between start and finish for occupancy
    step_size = 0.1

    # check for straight line case (infinte new circle radius):
    straight_tolerance = 0.01
    if abs(theta_rad - theta_robot) < straight_tolerance:
      #print("Picked straight line")
      # if there is a straight line between the points, check validity on points on the line between them
      distance = opp * 2.
      vector = final_position - node.position
      steps = distance / step_size
      for step in range(0, int(round(steps+1))):
        step_position = node.position + step * step_size * vector
        if not is_valid(step_position, occupancy_grid):
          return None
    
    elif dist_between_points(final_position, node.position) < 2. * ROBOT_RADIUS:
      # tiny circle, just compute is valid at the destination
      if not is_valid(final_position, occupancy_grid):
        return None
    
    else: # we are going in a clockwise or anticlockwise circle
      # compute new arc length
      new_radius_length = hyp
      arc_length = new_radius_length * phi
      steps = arc_length / step_size
      step_angle = phi / steps

      #print("radisu length: ", new_radius_length)
      #print("arc length: ", arc_length)
      #print("steps: ", steps)

      points = []

      # offset is pi/2 if going clockwise, or -pi/2 if going anticlockwise
      # using the **origianl** circle's theta radius
      offset = -np.pi/2. if (theta_rad - theta_robot) > 0 else +np.pi/2. # theta_rad-theta_tobot is < 0 for anticlockwise
      direction = 1 if (theta_rad - theta_robot) > 0 else -1. # theta_rad-theta_tobot is < 0 for anticlockwise
  
      # step along the arc, looking for collisions
      for step in range(0, int(round(steps+1))):
        angle = node.pose[YAW] + offset + (step * step_angle * direction)
        
        # compute x, y, coords
        x = centre[X] + new_radius_length * np.cos(angle)
        y = centre[Y] + new_radius_length * np.sin(angle)

        points.append([x, y])

        # compute the position is okay in the map
        # if it is not, return False
        if not is_valid(np.array([x,y]), occupancy_grid):
          return None

      # check the destination
      if not is_valid(final_position, occupancy_grid):
        return None

      #print("#######################")
      #print("a: ", node.position)
      #print("b: ", final_position)
      #print("step points: ", points)
      #print("#######################")

  else:
    return None

  #print("robot theta:   ", theta_robot)
  #print("tangent theta: ", theta_rad)
  #print("------------------------------")
  #print("------------------------------")

  final_node = Node(final_pose)
  return final_node

def get_new_circle_centre(pose_a, pose_b):
  # get gradients of radius lines from points a and b, by shifting tangent gradient by pi/4 degrees

  m_a = np.tan(pose_a[YAW] + (np.pi/2.))
  m_b = np.tan(pose_b[YAW] + (np.pi/2.))
  #m_a = np.tan(pose_a[YAW])
  #m_b = np.tan(pose_b[YAW])

  # compute constant c components using the pose coords and the gradients
  # y = mx + c, => c = y - mx
  c_a = pose_a[Y] - m_a * pose_a[X]
  c_b = pose_b[Y] - m_b * pose_b[X]

  # compute the centre of the new circle
  centre = np.array([
    (c_b - c_a) / (m_a - m_b),
    (c_a - m_a * c_b / m_b) / (1 - m_a / m_b)
  ])

  return centre


def get_angle_of_vector(vector):
  angle = np.arctan(vector[Y]/vector[X])
  # make sure our angle is positive only
  if vector[X] < 0:
    angle = np.pi + angle
  angle = (angle + (np.pi * 2)) % (np.pi * 2)

  return angle

# # now compute the angle of the tangent vector
#   theta_vec = np.arctan(vec[Y]/vec[X])
#   pi2 = np.pi * 2

#   # account for arctan function
#   if vec[X] < 0:
#     theta_vec = np.pi + theta_vec
#   theta_vec = (theta_vec + pi2) % pi2

#   print("robot theta:   ", theta_rob)
#   print("tangent theta: ", theta_vec)

#   # now check if robot yaw or robot yaw + pi is within tolerance of the theta_vec
#   tolerance = 2.
#   if abs(theta_vec - theta_rob) < tolerance or abs(-np.pi + theta_vec - theta_rob) < tolerance:
#     # for now don't do any path checking, just return theta_rob + 180 and return
#     theta_rob = (theta_vec + np.pi) % pi2
#     final_pose[YAW] = theta_rob

#   else:
#     # robot is not facing in an approporiate direction to traverse this circle, return None
#     return None
#     #pass
  

#   final_node = Node(final_pose)
#   return final_node



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
    # computes an index into the values table for that position.
    idx = ((position - self._origin) / self._resolution).astype(np.int32)
    if len(idx.shape) == 2:
      idx[:, 0] = np.clip(idx[:, 0], 0, self._values.shape[0] - 1)
      idx[:, 1] = np.clip(idx[:, 1], 0, self._values.shape[1] - 1)
      return (idx[:, 0], idx[:, 1])
    idx[0] = np.clip(idx[0], 0, self._values.shape[0] - 1)
    idx[1] = np.clip(idx[1], 0, self._values.shape[1] - 1)
    return tuple(idx)

  def get_position(self, i, j):
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


def rrt(start_pose, goal_position, occupancy_grid):
  # RRT builds a graph one node at a time.
  graph = []
  start_node = Node(start_pose)
  final_node = None
  if not occupancy_grid.is_free(goal_position):
    print('Goal position is not in the free space.')
    return start_node, final_node
  graph.append(start_node)
  for _ in range(MAX_ITERATIONS): 
    position = sample_random_position(occupancy_grid)
    # With a random chance, draw the goal position.
    if np.random.rand() < .2:
    #if np.random.rand() < .05:
      position = goal_position
    # Find closest node in graph.
    # In practice, one uses an efficient spatial structure (e.g., quadtree).
    potential_parent = sorted(((n, np.linalg.norm(position - n.position)) for n in graph), key=lambda x: x[1])
    # Pick a node at least some distance away but not too far.
    # We also verify that the angles are aligned (within pi / 4).
    u = None
    for n, d in potential_parent:
      if d > .2 and d < 1.5 and n.direction.dot(position - n.position) / d > 0.70710678118:
        u = n
        break
    else:
      continue
    v = adjust_pose(u, position, occupancy_grid)
    if v is None:
      continue
    u.add_neighbor(v)
    v.parent = u
    graph.append(v)
    if np.linalg.norm(v.position - goal_position) < .2:
      final_node = v
      break
  return start_node, final_node


def find_circle(node_a, node_b):
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

  # Run RRT.
  start_node, final_node = rrt(START_POSE, GOAL_POSITION, occupancy_grid)

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
  