#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

class TFToPosePublisher:
    def __init__(self):
        rospy.init_node('tf_to_pose_publisher', anonymous=True)

        self.tf_listener = tf.TransformListener()
        self.pose_publisher = rospy.Publisher('/pose', PoseStamped, queue_size=10)

    def run(self):
        rate = rospy.Rate(10)  # 10Hz

        while not rospy.is_shutdown():
            try:
                # 获取从 quadrotor/base_link 到 world 坐标系的变换
                (trans, rot) = self.tf_listener.lookupTransform('world', 'quadrotor/base_link', rospy.Time(0))

                # 构建 PoseStamped 消息
                pose_msg = PoseStamped()
                pose_msg.header.stamp = rospy.Time.now()
                pose_msg.header.frame_id = 'world'

                # 设置位置信息
                pose_msg.pose.position = Point(*trans)

                # 设置姿态信息
                pose_msg.pose.orientation = Quaternion(*rot)

                # 发布位姿消息
                self.pose_publisher.publish(pose_msg)

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn("TF lookup failed, waiting for transformation...")

            rate.sleep()

if __name__ == '__main__':
    try:
        tf_to_pose_publisher = TFToPosePublisher()
        tf_to_pose_publisher.run()
    except rospy.ROSInterruptException:
        pass
