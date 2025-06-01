#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
from std_msgs.msg import String, UInt8, ColorRGBA
from geometry_msgs.msg import PoseStamped
import numpy as np
import datetime
import time
from sensor_msgs.msg import RegionOfInterest as ROI

def roicallback(msg):
    roi_circel = msg
    x = roi_circel.x_offset
    y = roi_circel.y_offset
    radius = roi_circel.width
    if radius > 50:
        pub_mled.publish(str(3))

def pad_id_callback(msg):
    pad_id = msg.data
    if pad_id == 1:
        pub_mled.publish(str(1))

def pose_callback(msg):
    global start_timing

    pos_x = msg.pose.position.x
    pos_y = msg.pose.position.y

    diff_start_x = pos_x - start_x
    diff_start_y = pos_y - start_y
    rho_start = np.hypot(diff_start_x, diff_start_y)

    diff_goal_x = pos_x - goal_x
    diff_goal_y = pos_y - goal_y
    rho_goal = np.hypot(diff_goal_x, diff_goal_y)

    diff_radar_x = pos_x - radar_x
    diff_radar_y = pos_y - radar_y
    rho_radar = np.hypot(diff_radar_x, diff_radar_y)

    diff_radar1_x = pos_x - radar1_x
    diff_radar1_y = pos_y - radar1_y
    rho_radar1 = np.hypot(diff_radar1_x, diff_radar1_y)

    # print(rho_start, rho_goal, rho_radar, rho_radar1)

    if rho_start < start_r:
        pub_mled.publish(str(2))
    if rho_goal < goal_r:
        pub_mled.publish(str(4))
        start_timing = 1
    if rho_radar < radar_r or rho_radar1 < radar1_r:
        led = ColorRGBA()
        led.r = 255
        led.g = 255
        led.b = 0
        pub_led.publish(led)
        time.sleep(0.1)
            
if __name__ == "__main__":
    start_x = -1.2
    start_y = -1.2
    start_r = 0.5   
    goal_x = 1.2
    goal_y = 1.2
    goal_r = 0.3
    radar_x = 0
    radar_y = -0.6
    radar_r = 0.3
    radar1_x = 0.3
    radar1_y = 1.2
    radar1_r = 0.2
    test_time = 0
    start_timing = 0
    
    # 初始化ROS节点    
    rospy.init_node('score', anonymous=False)
    rospy.Subscriber('/mission_pad_id', UInt8, pad_id_callback)
    rospy.Subscriber('/pose', PoseStamped, pose_callback, queue_size=10)
    pub_mled = rospy.Publisher('/mled', String, queue_size=10)
    pub_led = rospy.Publisher('/led', ColorRGBA, queue_size=10)
    sub = rospy.Subscriber('/circle_msg', ROI, roicallback)
    
    led = ColorRGBA()
    led.r = 0.0
    led.g = 255.0
    led.b = 0.0
    led.a = 0.0
    for i in range(10):
        pub_led.publish(led)
        time.sleep(0.1)
    start_time = datetime.datetime.now()
    rospy.loginfo('start timing!')

    while not rospy.is_shutdown():
        if start_timing == 0:
            test_time = datetime.datetime.now() - start_time
            rospy.loginfo('test time: ' + str(test_time))
        if start_timing == 1:
            rospy.loginfo('test time: ' + str(test_time))
        time.sleep(0.01)

    rospy.spin()