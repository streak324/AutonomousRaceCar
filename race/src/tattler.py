#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import Image 


def callback(data):
   print data.header.stamp, data.header.seq
   return


if __name__ == '__main__':
    print("Tattler node started")
    rospy.init_node('tattler',anonymous = True)
    rospy.Subscriber("right_edges",Image,callback)
    rospy.Subscriber("/zed/right/image_raw_color",Image,callback)
    rospy.spin()
