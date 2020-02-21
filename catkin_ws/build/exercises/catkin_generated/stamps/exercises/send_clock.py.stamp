#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import rospy
import datetime
from std_msgs.msg import String


def run():
  rospy.init_node('send_clock', anonymous=True)
  publisher = rospy.Publisher('/date_time', String, queue_size=1)
  rate = rospy.Rate(1000)

  while not rospy.is_shutdown():
    now = rospy.get_rostime().to_sec()
    date_time = datetime.datetime.fromtimestamp(int(now)).strftime('%Y%m%d %H:%M:%S')
    publisher.publish(date_time)
    rate.sleep()


if __name__ == '__main__':
  run()