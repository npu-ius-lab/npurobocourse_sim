#!/usr/bin/python3
#coding=utf-8
# created by Liu binhong
import rospy
import numpy as np
from gazebo_msgs.srv import *
import tf2_ros
from geometry_msgs.msg import Twist, TransformStamped
import rospy 

model = GetModelStateRequest()
if __name__ == '__main__': 
    rospy.init_node('groundtruthtf')
    rate = rospy.Rate(10.0)
    tf_pub = tf2_ros.TransformBroadcaster()
    while not rospy.is_shutdown():
        get_state_service = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        model.model_name ='tianbot_mini'
        model.relative_entity_name = "world"
        objstate = get_state_service(model)
        ts = TransformStamped()
        ts.header.frame_id = "world"
        ts.child_frame_id = "tianbot_mini/base_link"
        ts.header.stamp = rospy.Time.now()

        

        ts.transform.translation.x = objstate.pose.position.x
        ts.transform.translation.y = objstate.pose.position.y
        ts.transform.translation.z = objstate.pose.position.z

        ts.transform.rotation.x = objstate.pose.orientation.x
        ts.transform.rotation.y = objstate.pose.orientation.y
        ts.transform.rotation.z = objstate.pose.orientation.z
        ts.transform.rotation.w = objstate.pose.orientation.w
        #print(state_position)
        print(ts)  
        tf_pub.sendTransform(ts) 
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        rate.sleep()







    
    
