from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import numpy as np


def braitenberg(front, front_left, front_right, left, right):
  u = .5  # [m/s] so when not facing an obstacle the robot accelerates
  w = 0.  # [rad/s] going counter-clockwise.

  # MISSING: Implement a braitenberg controller that takes the range
  # measurements given in argument to steer the robot.
  
  # create an array of sensor input from the robot
  sens_inp = np.array([front, front_left, front_right, left, right])
  
  # smooth with arctan (could use tanh), but then take the gradient of this, so big inputs give little change, but small imputs give the most change
  sens_inp = 1.0 / (1.0 + 3*(sens_inp**2))  # this is the gradient of the tanh function

  # apply weights matrix
  # implement a coward, but we have negatives as we are manipulating w, not the wheel velocities.
  # first row is u weights, second row goes to w weights
  
  # this one still tries to accelerate forwards even when something is infront of it due to the side being positive 
  #weights = np.array([
  #	[-3.0, -1.0, -1.0, 2.5, 2.5],
  #	[0.4, -1.0, 1.0, -4.0, 4.0]
  #])

  # this should be a better form of weights, when it is surrounded, acceleration is negative default
  # a bias on the front sensor to steer slighty left is given so that it does not get stuck still in reverse (it can manage to turn around, test this with extra objects)
  # actually the plus on left and right makes sense, i want it to get smaller when it gets bigger
  weights = np.array([
  	[-0.3, -0.2, -0.2, 0., 0.],
	  [ 0.0, -5.0,  5.0, 0., 0.]
  ])
  # multiply sense inputs by the weights.
  params = np.matmul(weights, sens_inp)

  # U IS VELOCITY NOT ACCELERATION!!!!!!!!!!!!!!!

  # add a function to turn around the robot when it gets very very close
  # prevents the non turning issue when it goes head on into something
  w = w + 40 * (1 / (1 + 100 * (front**2)))
  
  # extract params noet this doesn't consider previous step since we setp them to 0 / 0.5 at the start.. it does do what you think nice.
  u, w = u + params[0], w + params[1]

  # robot having default velocity of 5, so it always keep moving. 

  return u, w


def rule_based(front, front_left, front_right, left, right):
  u = 0.  # [m/s]
  w = 0.  # [rad/s] going counter-clockwise.

  # MISSING: Implement a rule-based controller that avoids obstacles.

  # apply tanh to the input to deal with infinity
  sens_inp = np.array([front, front_left, front_right, left, right])
  sens_inp = np.tanh(sens_inp)
  
  # default velocity
  u = 0.3


  # if all sensors detect obstacles are far away, adjust slightly using a highest absolute value for direction
  if (sens_inp > 0.6).all():
    # move in coward motion, away from the closest object
    return u, sens_inp[2] + sens_inp[4] - (sens_inp[1] + sens_inp[3])

  else:
    # breakout! (mentioned in part C only - rest of rules are as described in part a) 
    # If the front sensor reports a much greater distance than the side sensors, 
    # don't change direction so much, use the left and right sensors for small adjustments
    front = sens_inp[0]
    if front > 0.3 and front > 2.*sens_inp[1] and front > 2.*sens_inp[2]:
      # override w updates and steer using left and right sensors, which should be close enough to detect collisions
      # in tight areas
      if sens_inp[3] < 0.2:
        w = w + 1.5 * (1-sens_inp[3])
      if sens_inp[4] < 0.2:
        w = w - 1.5 * (1-sens_inp[4])
      if sens_inp[1] < 0.2:
        w = w + 1.5 * (1-sens_inp[1])
      if sens_inp[2] < 0.2:
        w = w - 1.5 * (1-sens_inp[2])

      u = 0.1
      return u, w


    # if the right hand side detects an approaching object , alter w to move left
    if sens_inp[2] < 0.6:
      w = w + 4 * (1-sens_inp[2])
  
    # if the left hand slide detects and approaching object, alter w to move to the right
    if sens_inp[1] < 0.6:
      w = w - 4 * (1-sens_inp[1])

    # if robot is very close to the right hand slide, adjust left a little
    if sens_inp[4] < 0.2:
      w = w + 2 * (1-sens_inp[4])

    # if robot is very close to the left hand slide, adjust right a little
    if sens_inp[3] < 0.2:
      w = w - 2 * (1-sens_inp[3])

    # if any front sensor is close, reduce speed
    if (sens_inp[0:3] < 0.3).any():
      u = 0.1

    # turnaround function, if very close to front, turn around
    if sens_inp[0] < 0.2:
      w = 7.5 * (1.0 - sens_inp[0])	

  # todo: make these adaptive to different u values...
  
  return u, w
