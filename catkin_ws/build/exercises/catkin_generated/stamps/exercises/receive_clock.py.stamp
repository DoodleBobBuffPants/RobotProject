#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import rospy
from std_msgs.msg import String

import subprocess
import time


class Clock(object):
  def __init__(self):
    rospy.Subscriber('/date_time', String, self.callback)
    self._ready = False

  def callback(self, msg):
    if not self._ready:
      subprocess.call(['sudo date +%F%T -s "{}"'.format(msg.data)], shell=True)
      print('Time set to', msg.data)
      self._ready = True

  @property
  def ready(self):
    return self._ready


def run():
  rospy.init_node('get_time', anonymous=True)
  clock = Clock()
  rate = rospy.Rate(100)

  # Run sudo once.
  subprocess.call(['sudo echo "Authenticated"'], shell=True)

  while not rospy.is_shutdown():
    if clock.ready:
      break
    rate.sleep()
    

if __name__ == '__main__':
  run()
