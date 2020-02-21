from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import matplotlib.pylab as plt
import numpy as np
import sys

import helper


def sine(fig, ax):
  # MISSING: Plot a sine function between 0 and 2*np.pi. See example function identiy().
  pass


def normal(fig, ax):
  # MISSING: Plot the PDF of the standard Normal distribution between -3 and 3. See example function identiy().
  pass


def normal2(fig, ax):
  # MISSING: Plot the contours of the two-dimensional standard Normal distribution.
  # See https://matplotlib.org/api/_as_gen/matplotlib.pyplot.contour.html
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