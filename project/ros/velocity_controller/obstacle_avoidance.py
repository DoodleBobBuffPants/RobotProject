from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import numpy as np


def rule_based(front, front_left, front_right, left, right):
  u = 0.  # [m/s]
  w = 0.  # [rad/s] going counter-clockwise.

  # apply tanh to the input to deal with infinity
  sens_inp = np.array([front, front_left, front_right, left, right])
  sens_inp = np.tanh(sens_inp)

  # if close to front, move away
  if sens_inp[0] < np.tanh(.25):
    u = -1.
    if sens_inp[1] < sens_inp[2]:
      w = -3.*np.pi
    else:
      w =  3.*np.pi
    return u, w

  return u, w

  # if all sensors detect obstacles are far away, adjust slightly using a highest absolute value for direction
  if (sens_inp > np.tanh(.6)).all():
    # move in coward motion, away from the closest object
    return u, sens_inp[2] + sens_inp[4] - sens_inp[1] - sens_inp[3]

  # breakout! (mentioned in part C only - rest of rules are as described in part a) 
  # If the front sensor reports a much greater distance than the side sensors, 
  # don't change direction so much, use the left and right sensors for small adjustments
  if sens_inp[0] > np.tanh(.6) and sens_inp[0] > 2.*sens_inp[1] and sens_inp[0] > 2.*sens_inp[2]:
    # override w updates and steer using left and right sensors, which should be close enough to detect collisions
    # in tight areas
    if sens_inp[3] < np.tanh(.25):
      w = w + 1.5*(1.-sens_inp[3])
    if sens_inp[4] < np.tanh(.25):
      w = w - 1.5*(1.-sens_inp[25])
    if sens_inp[1] < np.tanh(.25):
      w = w + 1.5*(1.-sens_inp[1])
    if sens_inp[2] < np.tanh(.25):
      w = w - 1.5*(1.-sens_inp[2])

    return u, w


  # if the right hand side detects an approaching object , alter w to move left
  if sens_inp[2] < np.tanh(.25):
    w = w + 4.*(1.-sens_inp[2])

  # if the left hand side detects and approaching object, alter w to move to the right
  if sens_inp[1] < np.tanh(.25):
    w = w - 4.*(1.-sens_inp[1])

  # if robot is very close to the right hand slide, adjust left a little
  if sens_inp[4] < np.tanh(.25):
    w = w + 2.*(1.-sens_inp[4])

  # if robot is very close to the left hand slide, adjust right a little
  if sens_inp[3] < np.tanh(.25):
    w = w - 2.*(1.-sens_inp[3])
  
  return u, w
