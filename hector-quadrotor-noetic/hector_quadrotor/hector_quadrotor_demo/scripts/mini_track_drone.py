#!/usr/bin/python
# -*- coding: UTF-8 -*-
# 上面两行不可省略，第一行是：告诉操作系统执行这个脚本的时候，调用 /usr/bin 下的 python 解释器。第二行是：定义编码格式 "UTF-8-" 支持中文

from actionlib.action_client import GoalManager
import rospy 
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from gazebo_msgs.srv import *
def send_goals_python():
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()
    #定义发送目标点的对象
    goal = MoveBaseGoal()
    model = GetModelStateRequest()


    while not rospy.is_shutdown():
        get_state_service = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)     
        model.model_name ='quadrotor'
        model.relative_entity_name = "world"
        objstate = get_state_service(model)
        print(objstate)
        goal.target_pose.header.frame_id = 'world'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = objstate.pose.position.x
        goal.target_pose.pose.position.y = objstate.pose.position.y

        goal.target_pose.pose.orientation.z = objstate.pose.orientation.z
        goal.target_pose.pose.orientation.w = objstate.pose.orientation.w

        client.send_goal(goal)

        wait = client.wait_for_result(rospy.Duration.from_sec(10.0))  # 发送完毕目标点之后，根据action 的机制，等待反馈执行的状态，等待时长是：30 s.
        if not wait:
            str_log="The Goal Planning Failed for some reasons" 
            rospy.loginfo(str_log)
            continue
        else:
            str_log="The Goal achieved success !!!" 
            rospy.loginfo(str_log)

    return "Mission Finished."

if __name__ == '__main__':
    rospy.init_node('send_goals_python',anonymous=True)    # python 语言方式下的　初始化 ROS 节点，
    result = send_goals_python()
    rospy.loginfo(result)