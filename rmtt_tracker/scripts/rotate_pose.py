#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import TransformStamped, PoseStamped
import tf2_ros
import numpy as np

def callback(msg):
    # Extract the current pose information from the message
    x = msg.pose.position.x
    y = msg.pose.position.y
    z = msg.pose.position.z
    roll, pitch, yaw = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
    
    # Rotate the pose by 180 degrees around y axis
    pitch -= np.deg2rad(180)
    qx, qy, qz, qw = quaternion_from_euler(yaw, -pitch, roll)

    # Create a new PoseStamped message with the rotated position and orientation
    new_msg = PoseStamped()
    new_msg.header = msg.header
    new_msg.pose.position.x = x
    new_msg.pose.position.y = y
    new_msg.pose.position.z = z
    new_msg.pose.orientation.x = qx
    new_msg.pose.orientation.y = qy
    new_msg.pose.orientation.z = qz
    new_msg.pose.orientation.w = qw

    # Publish the new PoseStamped message
    pub.publish(new_msg)

    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()
    t.header.stamp = msg.header.stamp
    t.header.frame_id = 'map'
    t.child_frame_id = 'new_base_link'
    t.transform.translation.x = new_msg.pose.position.x
    t.transform.translation.y = new_msg.pose.position.y
    t.transform.translation.z = new_msg.pose.position.z
    t.transform.rotation.x = new_msg.pose.orientation.x
    t.transform.rotation.y = new_msg.pose.orientation.y
    t.transform.rotation.z = new_msg.pose.orientation.z
    t.transform.rotation.w = new_msg.pose.orientation.w
    br.sendTransform(t)

rospy.init_node('rotate_pose_node')
sub = rospy.Subscriber('/pose', PoseStamped, callback)
pub = rospy.Publisher('/new_pose', PoseStamped, queue_size=10)
rospy.spin()
