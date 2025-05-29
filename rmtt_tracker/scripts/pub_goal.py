#!/usr/bin/python3
# Copyright 2017 HyphaROS Workshop.
# Developer: HaoChih, LIN (hypha.ros@gmail.com)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rospy
import string
import math
import time
import sys
import numpy as np
import message_filters
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseActionResult, MoveBaseActionFeedback
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped

class MultiGoals:
    def __init__(self):

        self.sub1 = rospy.Subscriber('/pose', PoseStamped, self.statusCA, queue_size=10)
        self.pub = rospy.Publisher('/tianbot_mini/move_base_simple/goal', PoseStamped, queue_size=10)   

        self.goalMsg = PoseStamped()
        self.goalMsg.header.frame_id = 'tianbot_mini/odom'

        time.sleep(1)
            
    def statusCA(self, data):

        self.goalMsg.header.stamp = rospy.Time.now()
        self.goalMsg.pose.position.x = data.pose.position.x
        self.goalMsg.pose.position.y = data.pose.position.y
        self.goalMsg.pose.orientation.x = 0
        self.goalMsg.pose.orientation.y = 0
        self.goalMsg.pose.orientation.z = 0
        self.goalMsg.pose.orientation.w = 1

        rospy.sleep(1.0)
        self.pub.publish(self.goalMsg) 
                            
if __name__ == "__main__":
    try:    
        # ROS Init    
        rospy.init_node('pub_goal', anonymous=True)        
        # Constract MultiGoals Obj
        rospy.loginfo("Pub Goal Executing...")
        
        mg = MultiGoals()          
        rospy.spin()

    except KeyboardInterrupt:
        print("shutting down")


