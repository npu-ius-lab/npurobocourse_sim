#!/usr/bin/env python
import rospy
import string
import math
import time
import sys
import numpy as np

from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseActionResult, MoveBaseActionFeedback
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped

class MultiGoals:
    def __init__(self, goalListX, goalListY, QuaternionListX, QuaternionListY, QuaternionListZ, QuaternionListW, retry, map_frame):
        #self.sub = rospy.Subscriber('move_base/result', MoveBaseActionResult, self.statusCB, queue_size=10)
        self.sub = rospy.Subscriber('/tianbot_mini/move_base/feedback', MoveBaseActionFeedback, self.statusCA, queue_size=10)
        self.pub = rospy.Publisher('/tianbot_mini/move_base_simple/goal', PoseStamped, queue_size=10)   
        # params & variables
        self.goalListX = goalListX
        self.goalListY = goalListY
        self.QuaternionListX = QuaternionListX
        self.QuaternionListY = QuaternionListY
        self.QuaternionListZ = QuaternionListZ
        self.QuaternionListW = QuaternionListW
        self.retry = retry
        self.goalId = 0
        self.goalMsg = PoseStamped()
        self.goalMsg.header.frame_id = map_frame

        # Publish the first goal
        time.sleep(1)
        self.goalMsg.header.stamp = rospy.Time.now()
        self.goalMsg.pose.position.x = self.goalListX[self.goalId]
        self.goalMsg.pose.position.y = self.goalListY[self.goalId]
        self.goalMsg.pose.orientation.x = self.QuaternionListX[self.goalId]
        self.goalMsg.pose.orientation.y = self.QuaternionListY[self.goalId]
        self.goalMsg.pose.orientation.z = self.QuaternionListZ[self.goalId]
        self.goalMsg.pose.orientation.w = self.QuaternionListW[self.goalId]
        self.pub.publish(self.goalMsg) 
        rospy.loginfo("Initial goal published! Goal ID is: %d", self.goalId)
        #self.goalId = self.goalId + 1  

    # def statusCB(self, data):
    #     if data.status.status == 3: #reached
    #         self.goalMsg.header.stamp = rospy.Time.now()                
    #         self.goalMsg.pose.position.x = self.goalListX[self.goalId]
    #         self.goalMsg.pose.position.y = self.goalListY[self.goalId]
    #         self.pub.publish(self.goalMsg)
    #         rospy.loginfo("Initial goal published! Goal ID is: %d", self.goalId)    
    #         if self.goalId < (len(self.goalListX)-1):
    #             self.goalId = self.goalId + 1
    #         else:
    #             self.goalId = 0 

    # 改成用距离来做判定             
    def statusCA(self, data):
        x_diff = data.feedback.base_position.pose.position.x - self.goalListX[self.goalId]
        y_diff = data.feedback.base_position.pose.position.y - self.goalListY[self.goalId]
        dis = np.hypot(x_diff, y_diff)
        if dis < 0.5:
            # 先更新goalId到下一个目标
            if self.goalId < (len(self.goalListX)-1):
                self.goalId = self.goalId + 1
            else:
                self.goalId = 0
            
            # 然后发布下一个目标
            self.goalMsg.header.stamp = rospy.Time.now()                
            self.goalMsg.pose.position.x = self.goalListX[self.goalId]
            self.goalMsg.pose.position.y = self.goalListY[self.goalId]
            self.goalMsg.pose.orientation.x = self.QuaternionListX[self.goalId]
            self.goalMsg.pose.orientation.y = self.QuaternionListY[self.goalId]
            self.goalMsg.pose.orientation.z = self.QuaternionListZ[self.goalId]
            self.goalMsg.pose.orientation.w = self.QuaternionListW[self.goalId]

            self.pub.publish(self.goalMsg)
            rospy.loginfo("Next goal published! Goal ID is: %d", self.goalId)
    	
               
                            

if __name__ == "__main__":
    try:    
        # ROS Init    
        rospy.init_node('multi_goals', anonymous=True)

        # Get params
        goalListX = rospy.get_param('~goalListX', '[0, 1, 2]')
        goalListY = rospy.get_param('~goalListY', '[0, 1, 2]')

        QuaternionListX = rospy.get_param('~QuaternionListX', '[0, 1, 2]')
        QuaternionListY = rospy.get_param('~QuaternionListY', '[0, 1, 2]')
        QuaternionListZ = rospy.get_param('~QuaternionListZ', '[0, 1, 2]')
        QuaternionListW = rospy.get_param('~QuaternionListW', '[0, 1, 2]')

        map_frame = rospy.get_param('~map_frame', 'map' )
        retry = rospy.get_param('~retry', '1') 

        goalListX = goalListX.replace("[","").replace("]","")
        goalListY = goalListY.replace("[","").replace("]","")

        QuaternionListX = QuaternionListX.replace("[","").replace("]","")
        QuaternionListY = QuaternionListY.replace("[","").replace("]","")
        QuaternionListZ = QuaternionListZ.replace("[","").replace("]","")
        QuaternionListW = QuaternionListW.replace("[","").replace("]","")

        goalListX = [float(x) for x in goalListX.split(",")]
        goalListY = [float(y) for y in goalListY.split(",")]

        QuaternionListX = [float(x) for x in QuaternionListX.split(",")]
        QuaternionListY = [float(y) for y in QuaternionListY.split(",")]
        QuaternionListZ = [float(x) for x in QuaternionListZ.split(",")]
        QuaternionListW = [float(y) for y in QuaternionListW.split(",")]

        if len(goalListX) == len(goalListY) & len(goalListY) >=2:          
            # Constract MultiGoals Obj
            rospy.loginfo("Multi Goals Executing...")
            mg = MultiGoals(goalListX, goalListY, QuaternionListX, QuaternionListY, QuaternionListZ, QuaternionListW, retry, map_frame)          
            rospy.spin()
        else:
            rospy.logerr("Lengths of goal lists are not the same")
    except KeyboardInterrupt:
        print("shutting down")
