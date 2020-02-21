from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import numpy as np
import matplotlib.pylab as plt


if __name__ == '__main__':
  data = np.genfromtxt('/tmp/gazebo_exercise.txt', delimiter=',')

  plt.figure()
  plt.plot(data[:, 0], data[:, 1], 'b', label='true')
  plt.plot(data[:, 3], data[:, 4], 'g', label='linearized')
  # Cylinder.
  a = np.linspace(0., 2 * np.pi, 20)
  x = np.cos(a) * .3 + .3
  y = np.sin(a) * .3 + .2
  plt.plot(x, y, 'k')
  # Walls.
  plt.plot([-2, 2], [-2, -2], 'k')
  plt.plot([-2, 2], [2, 2], 'k')
  plt.plot([-2, -2], [-2, 2], 'k')
  plt.plot([2, 2], [-2, 2], 'k')
  plt.axis('equal')
  plt.xlabel('x')
  plt.ylabel('y')
  plt.xlim([-2.5, 2.5])
  plt.ylim([-2.5, 2.5])

  if data.shape[1] == 6:
    plt.figure()
    error = np.linalg.norm(data[:, :2] - data[:, 3:5], axis=1)
    plt.plot(error, c='b', lw=2)
    plt.ylabel('Error [m]')
    plt.xlabel('Timestep')

  plt.show()