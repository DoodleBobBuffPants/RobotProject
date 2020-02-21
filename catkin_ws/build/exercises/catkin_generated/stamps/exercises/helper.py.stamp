from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import collections
import numpy as np


def check_solution(fn, expected, *args, **kwargs):
  approx = kwargs.get('approx', False)
  def terminal(a):
    return not isinstance(a, collections.Iterable) or isinstance(a, str)
  def equal(a, b, eps=None):
    if not terminal(a):
      if terminal(b):
        return False
      if len(a) != len(b):
        return False
      for x, y in zip(a, b):
        if not equal(x, y, eps):
          return False
      return True
    if not terminal(b):
      return False
    if eps is None:
      return a == b
    return abs(a - b) <= eps

  RED_COLOR = '\033[91m'
  GREEN_COLOR = '\033[92m'
  END_COLOR = '\033[0m'
  result = fn(*args)
  success = equal(result, expected, eps=1e-3 if approx else None)
  output = ((GREEN_COLOR + '[ OK ]') if success else
            (RED_COLOR + '[FAIL]')) + END_COLOR
  if args:
    print('Check {}({}): {}'.format(fn.__name__, ', '.join(str(a) for a in args), output))
  else:
    print('Check {}: {}'.format(fn.__name__, output))
  if not success:
    print('  Expected {}\n  Got {}'.format(repr(expected), repr(result)))


def warn(message):
  RED_COLOR = '\033[91m'
  END_COLOR = '\033[0m'
  print(RED_COLOR + message + END_COLOR)