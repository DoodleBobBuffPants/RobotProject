from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import math
import helper


def for_loop():
  # MISSING: Write a for-loop that builds an array containing the
  # cubic values of 5, 6, 7, 8, 9 and 10.
  # BONUS: Can you use list comprehensions instead?
  a = []
  for x in range(5, 11):
    a.append(x ** 3)
  return a
  # Or:
  # return [x ** 3 for x in range(5, 11)]


def format_string(v):
  # MISSING: Return a string displaying the number v with 2 decimals
  # places. If v = 1.2345, output is '1.23'.
  return '%.2f' % v


def filter_list(a, pred):
  # MISSING: Return a list that contains only elements v of a for
  # which pred(v) returns True.
  # E.g.: filter_list(range(4), lambda v: return v % 2 == 0)
  #       returns [0, 2].
  return [v for v in a if pred(v)]


def find_root(fn):
  # MISSING: Returns the value x for which fn(x) ~= 0 using the bisection method.
  # x should be correct within 1e-3.
  # fn is strictly monotonic and is always such that fn(0.) < 0 and fn(1.) > 0.

  # Number of iteration needs to be at least 10.
  a = 0.
  b = 1.
  for _ in range(10):
    x = (a + b) / 2.
    if fn(x) < 0.:
      a = x
    else:
      b = x
  return x


def finished_tictactoe(state):
  # MISSING: state is a list of strings. Each string contains 3 characters.
  # Each character can be 'X', 'O', or ' '. Return whether the Tic Tac Toe
  # position represented by this list is an ending position (no one else can
  # play).
  # E.g.: state = ['XO ']
  #                ' X ',
  #                'O X'] returns True.
  # E.g.: state = ['XO ']
  #                ' OX',
  #                'O X'] returns False.
  # BONUS: Can you think of a concise way of writing this function using NumPy?

  # Rows.
  if any(all(c == 'X' for c in row) for row in state):
    return True
  if any(all(c == 'O' for c in row) for row in state):
    return True
  # Columns.
  for j in range(3):
    all_xs = True
    all_os = True
    for i in range(3):
      all_xs = all_xs and state[i][j] == 'X'
      all_os = all_os and state[i][j] == 'O'
    if all_xs or all_os:
      return True
  # Diagonals.
  all_xs = True
  all_os = True
  for i in range(3):
    all_xs = all_xs and state[i][i] == 'X'
    all_os = all_os and state[i][i] == 'O'
  if all_xs or all_os:
    return True
  all_xs = True
  all_os = True
  for i in range(3):
    all_xs = all_xs and state[i][2-i] == 'X'
    all_os = all_os and state[i][2-i] == 'O'
  if all_xs or all_os:
    return True
  # Full board.
  return not any((' ' in row) for row in state)


def main():
  print('Welcome to the Mobile Robots course.\n')

  helper.check_solution(for_loop, [125, 216, 343, 512, 729, 1000])

  helper.check_solution(format_string, '1.23', 1.2345)
  helper.check_solution(format_string, '3.14', math.pi)

  helper.check_solution(filter_list, [1, 3], range(5), lambda v: v % 2 == 1)
  helper.check_solution(filter_list, [], range(5), lambda v: False)

  helper.check_solution(find_root, 0.5, lambda x: x * 2 - 1., approx=True)
  helper.check_solution(find_root, 0.7071, lambda x: x ** 2 - .5, approx=True)

  helper.check_solution(finished_tictactoe, True, ['XO ', ' X ', 'O X'])
  helper.check_solution(finished_tictactoe, True, [' O ', 'XXX', 'O  '])
  helper.check_solution(finished_tictactoe, True, ['XO ', 'X  ', 'X O'])
  helper.check_solution(finished_tictactoe, False, ['XO ', ' OX', 'O X'])
  helper.check_solution(finished_tictactoe, True, ['XOO', 'XOX', 'OXX'])


if __name__ == '__main__':
  main()