#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import RegionOfInterest as ROI
from sensor_msgs.msg import Range
from std_msgs.msg import UInt8
import smach
import smach_ros
import datetime
import numpy as np

class go2pose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['next','stay'])
 
    def execute(self, userdata):
        global target_vel, mission
        rospy.loginfo('Executing state go2pose')
        if target_vel.linear.x <= 0.1 and target_vel.angular.z <= 0.1:
            mission = 6
            return 'next'
        else:
            return 'stay'

class stanley(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['stay','end'])

    def execute(self, userdata):
        global s_vel
        if s_vel.linear.x <= 0.1 and s_vel.angular.z <= 0.1:
            mission = 7
            return 'end'
        else:
            return 'stay'

def callback_go2pose_speed(msg):
    global target_vel, mission
    target_vel = msg
    if mission == 5:
        pub.publish(target_vel)

def callback_stanley_speed(msg):
    global s_vel, mission
    s_vel = msg
    if mission == 6:
        pub.publish(s_vel)

if __name__ == '__main__':
    mission = 5
    target_vel = Twist()
    s_vel = Twist()

    rospy.init_node('state_machine')
    rospy.Subscriber("/cmd_vel_go2pose", Twist, callback_go2pose_speed)
    rospy.Subscriber("/cmd_vel_stanley", Twist, callback_stanley_speed)

    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    sm = smach.StateMachine(outcomes=['end'])

    with sm:    
        # Add states to the container
        smach.StateMachine.add('GO2POSE', go2pose(),
                               transitions={'next':'STANLEY','stay':'GO2POSE'})
        smach.StateMachine.add('STANLEY', stanley(),
                               transitions={'stay':'STANLEY','end':'end'})

    while not rospy.is_shutdown():
        outcome = sm.execute()
        # rate.sleep()
    
    # Wait for ctrl-c to stop the application
    rospy.spin()
    # sis.stop()
