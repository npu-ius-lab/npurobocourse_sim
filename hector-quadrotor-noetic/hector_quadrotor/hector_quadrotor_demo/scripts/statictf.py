#!/usr/bin/env python
#coding=utf-8
# created by Liu binhong
import rospy
import numpy as np
from gazebo_msgs.srv import *
import tf2_ros
from geometry_msgs.msg import Twist, TransformStamped
import rospy 
from tf.transformations import quaternion_from_euler, euler_from_quaternion
model = GetModelStateRequest()
if __name__ == '__main__': 
    rospy.init_node('groundtruthtf')
    rate = rospy.Rate(10.0)
    tf_pub = tf2_ros.StaticTransformBroadcaster()
    while not rospy.is_shutdown():
        ts = TransformStamped()
        ts.header.frame_id = "tianbot_mini/odom"
        ts.child_frame_id = "world"
        ts.header.stamp = rospy.Time.now()
        ts.transform.translation.x = 0
        ts.transform.translation.y = 0
        ts.transform.translation.z = 0

        quat = quaternion_from_euler(0, 0, 0)
        print(quat)
        ts.transform.rotation.x = quat[0]
        ts.transform.rotation.y = quat[1]
        ts.transform.rotation.z = quat[2]
        ts.transform.rotation.w = quat[3]
        #print(state_position)
        print(ts)  
        tf_pub.sendTransform(ts) 
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        rate.sleep()







    
    
