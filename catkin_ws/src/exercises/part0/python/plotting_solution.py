from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import matplotlib.pylab as plt
import numpy as np
import sys

import helper


def sine(fig, ax):
  # MISSING: Plot a sine function between 0 and 2*np.pi. See example function identiy().
  x = np.linspace(0., np.pi * 2., 100)
  plt.plot(x, np.sin(x), c='b', lw=2, label='$f(x) = \\sin(x)$')
  plt.xlabel('$x$')
  plt.ylabel('$f(x)$')
  plt.legend(loc='upper right')

  pass


def normal(fig, ax):
  # MISSING: Plot the PDF of the standard Normal distribution between -3 and 3. See example function identiy().
  x = np.linspace(-3., 3., 100)
  y = np.exp(-x ** 2 / 2.) / np.sqrt(2 * np.pi)
  plt.plot(x, y, c='b', lw=2, label='$f(x) = \\frac{\\exp(-x^2 / 2)}{\\sqrt{2 \\pi}}$')
  plt.xlabel('$x$')
  plt.ylabel('$f(x)$')
  plt.legend(loc='lower center')

  pass


def normal2(fig, ax):
  # MISSING: Plot the contours of the two-dimensional standard Normal distribution.
  # See https://matplotlib.org/api/_as_gen/matplotlib.pyplot.contour.html
  X, Y = np.meshgrid(np.linspace(-3, 3, 20), np.linspace(-3, 3, 20))
  Z = np.exp(-(X ** 2 + Y ** 2) / 2.) / (2. * np.pi)
  # Note that we could also have used scipy:
  # from scipy.stats import multivariate_normal
  # Z = np.empty(X.shape + (2,))
  # Z[:, :, 0] = X; Z[:, :, 1] = Y
  # Z = multivariate_normal([0., 0.], [[1., 0.], [0., 1.]]).pdf(Z)
  # plt.contourf(X, Y, rv)
  cset = plt.contourf(X, Y, Z, cmap='RdBu')
  plt.colorbar(label='$f(x) = \\frac{\\exp(-(x_1^2 / 2 - x_2^2 / 2)}{2 \\pi}$')
  plt.contour(X, Y, Z, cset.levels, colors='k')
  plt.xlabel('$x_1$')
  plt.ylabel('$x_2$')

  pass



def identity(fig, ax):
  # This function plots the y = x function between -1 and 1.
  x = np.linspace(-1, 1, 100)
  plt.plot(x, x, c='b', lw=2, label='$f(x) = x$')
  plt.xlabel('$x$')
  plt.ylabel('$f(x)$')
  plt.legend(loc='lower right')
  plt.axis('equal')


def main():
  fig = plt.figure()
  ax = plt.subplot(111)

  if len(sys.argv) == 1:
    fn = identity
  else:
    try:
      fn = globals()[sys.argv[1]]
    except KeyError:
      helper.warn('Unknown function: "{}"'.format(sys.argv[1]))
      return
  fn(fig, ax)

  plt.show()


if __name__ == '__main__':
  main()