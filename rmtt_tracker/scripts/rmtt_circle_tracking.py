#!/usr/bin/env python3

from cmath import pi
import rospy
import rospkg
import std_msgs.msg
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import RegionOfInterest as ROI
import cv2
from cv_bridge import CvBridge
import numpy as np

# resize the image to w*h
w = 360
h = 270
pid_w = [0.5, 0, 0] # pid parameters of yaw channel
pid_h = [0.8, 0, 0] # pid parameters of up channel
pid_f = [0.8, 0, 0] # pid parameters of forward channel

def callback(msg):
    tld_circel = msg
    x = tld_circel.x_offset
    y = tld_circel.y_offset
    radius = tld_circel.width
    yaw_speed, up_speed, forward_speed = trackFace(x, y, radius, pid_w, pid_h, pid_f)
    rc = "left: " + "0 " + "forward: " + str(forward_speed) + " up: " + str(up_speed) + "  yaw: " + str(yaw_speed)
    speed = Twist()   
     
    if yaw_speed != 0 or up_speed != 0 or forward_speed != 0:
        speed.linear.x = -forward_speed
        speed.linear.y = 0.0
        speed.linear.z = -up_speed
        speed.angular.x = 0.0
        speed.angular.y = 0.0
        speed.angular.z = -yaw_speed
        rospy.loginfo(rc)
        pub.publish(speed)
    else:
        rospy.loginfo("no circle detected")

def trackFace(x, y, radius, pid_w, pid_h, pid_f):
    ## PID
    # yaw channel
    error_w = x - w // 2
    speed_w = pid_w[0] * error_w
    speed_w = int(np.clip(speed_w, -100, 100)) / 100.0

    # up channel
    error_h = y - (h // 2 - 50)
    speed_h = pid_h[0] * error_h
    speed_h = int(np.clip(speed_h, -100, 100)) / 100.0

    # forward channel
    error_f = radius - 80
    speed_f = pid_f[0] * error_f
    speed_f = int(np.clip(speed_f, -100, 100)) / 100.0

    if x != 0:
        yaw_speed = speed_w
    else:
        yaw_speed = 0
    if y != 0:
        up_speed = speed_h
    else:
        up_speed = 0
    if radius != 0:
        forward_speed = speed_f
    else:
        forward_speed = 0

    return yaw_speed, up_speed, forward_speed

if __name__ == '__main__':

    bridge = CvBridge()
    rospy.init_node('circle_track', anonymous=True)
    sub = rospy.Subscriber('/circle_msg', ROI, callback)
    pub = rospy.Publisher('/cmd_vel_circle', Twist, queue_size=1)
    rospy.spin()

