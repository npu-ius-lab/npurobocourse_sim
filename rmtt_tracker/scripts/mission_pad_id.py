#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
from std_msgs.msg import String
from std_msgs.msg import UInt8

def callback(msg):
    pad_id = msg.data
    if pad_id == 1:
        pub.publish('A')
    if pad_id == 2:
        pub.publish('B')
    if pad_id == 3:
        pub.publish('C')
    if pad_id == 4:
        pub.publish('D')
            
if __name__ == "__main__":   
    # 初始化ROS节点    
    rospy.init_node('pad_to_led', anonymous=False)
    sub = rospy.Subscriber('mission_pad_id', UInt8, callback)
    pub = rospy.Publisher('mled', String, queue_size=1)  
    rospy.spin()
        



