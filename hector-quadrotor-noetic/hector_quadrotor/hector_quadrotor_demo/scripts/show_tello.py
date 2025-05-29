#!/usr/bin/python3
#coding=utf-8
# created by Vegicken Ye
import rospy
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
import threading
import time
class ModelMover:
    def __init__(self):
        rospy.init_node('show_tello_py')
        
        # 初始化参数
        self.lock = threading.Lock()
        self.current_pose = [1.0, 0.0, 0.0]  # [x, y, z]
        self.model_name = 'tello'                
        self.orientation = (0.0, 0.0, 0.0, 1.0)  # 固定四元数
        
        # 订阅者
        self.pose_sub = rospy.Subscriber("/pose", PoseStamped, self.pose_callback)

        # 等待Gazebo服务
        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        
    def pose_callback(self, msg):
        """接收新的位置数据并更新"""
        with self.lock:
            self.current_pose = [
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z
            ]

    def run(self):
        rate = rospy.Rate(100)  # 10Hz刷新频率
        while not rospy.is_shutdown():
            # 获取当前位置
            x, y, z = self.current_pose
                
            # 构造模型状态
            model_state = ModelState()
            model_state.model_name = self.model_name
            model_state.pose.position.x = x
            model_state.pose.position.y = y
            model_state.pose.position.z = z
            model_state.pose.orientation.x = self.orientation[0]
            model_state.pose.orientation.y = self.orientation[1]
            model_state.pose.orientation.z = self.orientation[2]
            model_state.pose.orientation.w = self.orientation[3]
            model_state.reference_frame = 'world'
            
            # 更新状态
            try:
                resp = self.set_state(model_state)
                if not resp.success:
                    rospy.logwarn("Update failed: %s", resp.status_message)
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s", str(e))
            
            rate.sleep()

if __name__ == '__main__':
    time.sleep(5)
    mover = ModelMover()
    mover.run()
