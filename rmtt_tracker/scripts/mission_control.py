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
                             outcomes=['next', 'stay'])
        
    def execute(self, userdata):
        global mission, rad
        rospy.loginfo('Executing state Turn')  
        if rad != 0:
            zero_twist = Twist()
            pub.publish(zero_twist)
            mission = 4
            return 'next'
        else:
            vel_rise = Twist()
            vel_rise.angular.z = 0.2
            pub.publish(vel_rise)
            return 'stay'

class Track_circle(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['next','stay']) 
    
    def execute(self, userdata):
        global rad, x_set, misson
        rospy.loginfo('Executing state Track_circle')
        # if rad >= 100 and rad <= 105 and vel.linear.x <= 0.05 and vel.linear.y <= 0.05 and vel.angular.z <= 0.1:
        if np.abs(x_set - 180) <= 5:
            misson = 5
            return 'next'
        else:
            print(rad)
            return 'stay'

class Cross_circle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['stay'])
    
    def execute(self, userdata):
        global start_timing, start_time
        rospy.loginfo('Executing state cross_circle')
        if start_timing == 0:
            start_time = datetime.datetime.now()
            start_timing = 1

        if (datetime.datetime.now() - start_time).seconds < 8.0:
            vel_cross = Twist()
            vel_cross.linear.x = 0.2
            pub.publish(vel_cross)
        else:
            zero_twist = Twist()
            pub.publish(zero_twist)

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

def callback_radius_circle(msg):
    global rad, x_set
    rad = msg.width
    x_set = msg.x_offset

def callback_cmd_vel_circle(msg):
    global vel, mission
    vel = msg
    if mission == 4:
        pub.publish(vel)

# def callback_track_speed(msg):
#     global final_vel
#     final_vel = msg
    
if __name__ == '__main__':
    mission = 1
    pad_id = 0
    start_timing = 0
    start_time = 0
    vel_tag = Twist()
    
    rospy.init_node('state_machine')
    rospy.Subscriber('/mission_pad_id', UInt8, callback_mission_pad)
    rospy.Subscriber("/cmd_vel_tag", Twist, callback_cmd_tag)
    rospy.Subscriber('/tof_btm', Range, callback_tof_btm)
    rospy.Subscriber("/circle_msg", ROI, callback_radius_circle)
    rospy.Subscriber("/cmd_vel_circle", Twist, callback_cmd_vel_circle)
    # rospy.Subscriber("/cmd_vel_cross", Twist, callback_track_speed)
    
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
                               transitions={'next':'TRACK_CIRCLE', 'stay':'TURN'})
        smach.StateMachine.add('TRACK_CIRCLE', Track_circle(),
                               transitions={'next':'CROSS_CIRCLE','stay':'TRACK_CIRCLE'})
        smach.StateMachine.add('CROSS_CIRCLE', Cross_circle(),
                               transitions={'stay':'CROSS_CIRCLE'})

    # Create and start the introspection server
    # sis = smach_ros.IntrospectionServer('my_smach_introspection_server', sm, '/SM_ROOT')
    # sis.start()

    while not rospy.is_shutdown():
        outcome = sm.execute()
        # rate.sleep()
    
    # Wait for ctrl-c to stop the application
    rospy.spin()
    # sis.stop()