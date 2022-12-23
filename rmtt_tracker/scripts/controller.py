#!/usr/bin/env python3

import rospy
import std_msgs.msg
from geometry_msgs.msg import Twist
from sensor_msgs.msg import RegionOfInterest as ROI
import numpy as np

w = 640
h = 640
pid_w = [0.5, 0, 0] # pid parameters of yaw channel
pid_h = [0.8, 0, 0] # pid parameters of up channel
pid_f = [0.8, 0, 0] # pid parameters of forward channel



if __name__ == '__main__':

    rospy.init_node('pid', anonymous=True)
    sub = rospy.Subscriber("roi", ROI, callback)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rospy.spin()
