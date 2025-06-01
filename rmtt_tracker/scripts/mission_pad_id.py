#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
from std_msgs.msg import Empty, String, UInt8

def callback(msg):
    pad_id = msg.data
    if pad_id == 1:
        mled_pub.publish(str(1))
    if pad_id == 2:
        mled_pub.publish(str(2))
        empty_msg = Empty()
        land_pub.publish(empty_msg)
    if pad_id == 3:
        mled_pub.publish(str(3))
        empty_msg = Empty()
        land_pub.publish(empty_msg)
    if pad_id == 4:
        mled_pub.publish(str(4))
            
if __name__ == "__main__":   
    # 初始化ROS节点    
    rospy.init_node('pad_to_led', anonymous=False)
    
    rospy.Subscriber('mission_pad_id', UInt8, callback)
    land_pub = rospy.Publisher('land', Empty, queue_size=1)
    mled_pub = rospy.Publisher('mled', String, queue_size=1)  
    
    rospy.spin()
        



