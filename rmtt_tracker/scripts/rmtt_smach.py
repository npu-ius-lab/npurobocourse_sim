#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import RegionOfInterest as ROI
from sensor_msgs.msg import Range
from std_msgs.msg import UInt8
import smach
import smach_ros
import datetime
import numpy as np

# define state Tag_track
class Tag_track(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['next', 'stay'])

    def execute(self, userdata):
        global pad_id, mission
        rospy.loginfo('Executing state Tag_track')
        if pad_id == 1:
            zero_twist = Twist()
            pub.publish(zero_twist)
            mission = 2
            return 'next'
        else:
            return 'stay'

# define state Rise
class Rise(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['next', 'stay'])
        
    def execute(self, userdata):
        global height, mission
        rospy.loginfo('Executing state Rise')  
        if height > 1.5:
            zero_twist = Twist()
            pub.publish(zero_twist)
            mission = 3
            return 'next'
        else:
            vel_rise = Twist()
            vel_rise.linear.z = 0.2
            pub.publish(vel_rise)
            return 'stay'

# define state Turn
class Turn(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['stay'])
        
    def execute(self, userdata):
        global mission, rad
        rospy.loginfo('Executing state Turn')  

        vel_rise = Twist()
        vel_rise.angular.z = 0.2
        pub.publish(vel_rise)
        return 'stay'

def callback_cmd_tag(msg):
    if mission == 1:
        vel_tag = msg
        pub.publish(vel_tag)

def callback_mission_pad(msg):
    global pad_id
    pad_id = msg.data

def callback_tof_btm(msg):
    global height
    height = msg.range
    
if __name__ == '__main__':
    mission = 2
    pad_id = 0
    vel_tag = Twist()
    
    rospy.init_node('state_machine')

    rospy.Subscriber('/mission_pad_id', UInt8, callback_mission_pad)
    rospy.Subscriber('/cmd_vel_tag', Twist, callback_cmd_tag)
    rospy.Subscriber('/tof_btm', Range, callback_tof_btm)
    
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    # rate
    # rate = rospy.Rate(10.0)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['end'])

    # Open the container
    with sm:    
        # Add states to the container
        smach.StateMachine.add('TAG_TRACK', Tag_track(), 
                               transitions={'next':'RISE', 'stay':'TAG_TRACK'})
        smach.StateMachine.add('RISE', Rise(), 
                               transitions={'next':'TURN', 'stay':'RISE'})
        smach.StateMachine.add('TURN', Turn(), 
                               transitions={'stay':'TURN'})

    # Create and start the introspection server
    # sis = smach_ros.IntrospectionServer('my_smach_introspection_server', sm, '/SM_ROOT')
    # sis.start()

    while not rospy.is_shutdown():
        outcome = sm.execute()
        # rate.sleep()
    
    # Wait for ctrl-c to stop the application
    rospy.spin()
    # sis.stop()