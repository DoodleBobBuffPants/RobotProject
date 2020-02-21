#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import rospy

# Robot motion commands:
# http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html
from geometry_msgs.msg import Twist


def run():
  rospy.init_node('hello_world')
  publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
  while not rospy.is_shutdown():
    vel_msg = Twist()
    vel_msg.linear.x = 0.1
    vel_msg.angular.z = 0.2
    publisher.publish(vel_msg)
    rospy.sleep(3)


if __name__ == '__main__':
  try:
    run()
  except rospy.ROSInterruptException:
    pass
