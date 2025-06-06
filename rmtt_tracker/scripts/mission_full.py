#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import RegionOfInterest as ROI
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist, PoseStamped
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
        if height > 3.0:
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
            vel_rise.angular.z = -0.2
            pub.publish(vel_rise)
            return 'stay'

# define state Track_circle
class Track_circle(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['next','stay']) 
    
    def execute(self, userdata):
        global x_set, y_set, circle_vel, misson
        rospy.loginfo('Executing state Track_circle')
        # if circle_vel.linear.x <= 0.02 and circle_vel.linear.z <= 0.02 and circle_vel.angular.z <= 0.02:
        if np.abs(x_set - 180) <= 5:
            misson = 5
            return 'next'
        else:
            # print(rad)
            return 'stay'

# define state Cross_circle
class Cross_circle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['stay','next'])
    
    def execute(self, userdata):
        global start_timing, start_time, mission
        rospy.loginfo('Executing state cross_circle')
        if start_timing == 0:
            start_time = datetime.datetime.now()
            start_timing = 1

        if (datetime.datetime.now() - start_time).seconds < 10.0:
            vel_cross = Twist()
            vel_cross.linear.x = 0.2
            pub.publish(vel_cross)
            return 'stay'
        else:
            zero_twist = Twist()
            pub.publish(zero_twist)
            mission = 6
            return 'next'

# define state Go2pose
class Go2pose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['stay'])
 
    def execute(self, userdata):
        global rho, mission,target_vel
        rospy.loginfo('Executing state go2pose')
        return 'stay'

def callback_cmd_tag(msg):
    global mission
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
    global rad, x_set, y_set
    rad = msg.width
    x_set = msg.x_offset
    y_set = msg.y_offset

def callback_cmd_vel_circle(msg):
    global circle_vel, mission
    circle_vel = msg
    if mission == 4:
        pub.publish(circle_vel)

def callback_go2pose_speed(msg):
    global target_vel, mission
    target_vel = msg
    if mission == 6:
        pub.publish(target_vel)

def callback_pose(msg):
    global rho, rho_final,height
    x = msg.pose.position.x
    y = msg.pose.position.y
    z = msg.pose.position.z
    height = z

    x_diff = -1.2 - x
    y_diff = -1.2 - y

    x_goal = 1.2 - x
    y_goal = 1.2 - y

    rho = np.hypot(x_diff, y_diff)
    rho_final = np.hypot(x_goal, y_goal)
    
if __name__ == '__main__':
    mission = 2
    pad_id = 0
    start_timing = 0
    start_time = 0
    rho = 100
    rho_final = 100
    vel_tag = Twist()
    
    global height,rad
    height = 0
    rad = 0

    rospy.init_node('state_machine')

    # rospy.Subscriber('/mission_pad_id', UInt8, callback_mission_pad)
    rospy.Subscriber('/pose', PoseStamped, callback_pose)
    rospy.Subscriber("/cmd_vel_tag", Twist, callback_cmd_tag)
    rospy.Subscriber('/tof_btm', Range, callback_tof_btm)
    rospy.Subscriber("/circle_msg", ROI, callback_radius_circle)
    rospy.Subscriber("/cmd_vel_circle", Twist, callback_cmd_vel_circle)
    rospy.Subscriber("/cmd_vel_go2pose", Twist, callback_go2pose_speed)
    # rospy.Subscriber("/cmd_vel_stanley", Twist, callback_stanley_speed)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    # rate
    # rate = rospy.Rate(10.0)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['end'])

    # Open the container
    with sm:    
        # Add states to the container
        # smach.StateMachine.add('TAG_TRACK', Tag_track(), 
        #                        transitions={'next':'RISE', 'stay':'TAG_TRACK'})
        smach.StateMachine.add('RISE', Rise(), 
                               transitions={'next':'TURN', 'stay':'RISE'})
        smach.StateMachine.add('TURN', Turn(), 
                               transitions={'next':'TRACK_CIRCLE', 'stay':'TURN'})
        smach.StateMachine.add('TRACK_CIRCLE', Track_circle(),
                               transitions={'next':'CROSS_CIRCLE', 'stay':'TRACK_CIRCLE'})
        smach.StateMachine.add('CROSS_CIRCLE', Cross_circle(),
                               transitions={'next':'GO2POSE', 'stay':'CROSS_CIRCLE'})
        smach.StateMachine.add('GO2POSE', Go2pose(),
                               transitions={'stay':'GO2POSE'})

    # Create and start the introspection server
    # sis = smach_ros.IntrospectionServer('my_smach_introspection_server', sm, '/SM_ROOT')
    # sis.start()

    while not rospy.is_shutdown():
        outcome = sm.execute()
        # rate.sleep()
    
    # Wait for ctrl-c to stop the application
    rospy.spin()
    # sis.stop()