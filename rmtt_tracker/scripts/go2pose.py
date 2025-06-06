#!/usr/bin/env python3

import rospy
import rospkg
import std_msgs.msg
from geometry_msgs.msg import Twist, PoseStamped
from tf.transformations import euler_from_quaternion
import numpy as np
from random import random

class PathFinderController:
    """
    Constructs an instantiate of the PathFinderController for navigating a
    3-DOF wheeled robot on a 2D plane

    Parameters
    ----------
    Kp_rho : The linear velocity gain to translate the robot along a line
             towards the goal
    Kp_alpha : The angular velocity gain to rotate the robot towards the goal
    Kp_beta : The offset angular velocity gain accounting for smooth merging to
              the goal angle (i.e., it helps the robot heading to be parallel
              to the target angle.)
    """

    def __init__(self, Kp_rho, Kp_alpha, Kp_beta):
        self.Kp_rho = Kp_rho
        self.Kp_alpha = Kp_alpha
        self.Kp_beta = Kp_beta

    def calc_control_command(self, x_diff, y_diff, theta, theta_goal):
        """
        Returns the control command for the linear and angular velocities as
        well as the distance to goal

        Parameters
        ----------
        x_diff : The position of target with respect to current robot position
                 in x direction
        y_diff : The position of target with respect to current robot position
                 in y direction
        theta : The current heading angle of robot with respect to x axis
        theta_goal: The target angle of robot with respect to x axis

        Returns
        -------
        rho : The distance between the robot and the goal position
        v : Command linear velocity
        w : Command angular velocity
        """

        # Description of local variables:
        # - alpha is the angle to the goal relative to the heading of the robot
        # - beta is the angle between the robot's position and the goal
        #   position plus the goal angle
        # - Kp_rho*rho and Kp_alpha*alpha drive the robot along a line towards
        #   the goal
        # - Kp_beta*beta rotates the line so that it is parallel to the goal
        #   angle
        #
        # Note:
        # we restrict alpha and beta (angle differences) to the range
        # [-pi, pi] to prevent unstable behavior e.g. difference going
        # from 0 rad to 2*pi rad with slight turn

        rho = np.hypot(x_diff, y_diff)
        alpha = (np.arctan2(y_diff, x_diff)
                 - theta + np.pi) % (2 * np.pi) - np.pi
        beta = (theta_goal - theta - alpha + np.pi) % (2 * np.pi) - np.pi
        v = self.Kp_rho * rho
        w = self.Kp_alpha * alpha - controller.Kp_beta * beta

        if alpha > np.pi / 2 or alpha < -np.pi / 2:
            v = -v

        return rho, v, w

def callback(msg):
   
    x = msg.pose.position.x
    y = msg.pose.position.y
    z = msg.pose.position.z

    quaternion = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
    theta = euler_from_quaternion(quaternion)[2]

    rc = "pose_x: " + str(x) + " pose_y: " + str(y) + " pose_z: " + str(z) + " theta: " + str(theta)
    # rospy.loginfo(rc)

    x_diff = x_goal - x
    y_diff = y_goal - y

    rho = np.hypot(x_diff, y_diff)

    if rho > 0.2:

        x_diff = x_goal - x
        y_diff = y_goal - y

        rho, v, w = controller.calc_control_command(
            x_diff, y_diff, theta, theta_goal)

        w = w / np.pi * 180 / 100

        if abs(v) > MAX_LINEAR_SPEED:
            v = np.sign(v) * MAX_LINEAR_SPEED

        if abs(w) > MAX_ANGULAR_SPEED:
            w = np.sign(w) * MAX_ANGULAR_SPEED

        vel_msg = Twist()
        vel_msg.linear.x = v * factor_v
        vel_msg.angular.z = w * factor_w
    else:
        vel_msg = Twist()

    pub.publish(vel_msg)

if __name__ == '__main__':
    controller = PathFinderController(2, 5, 2)

    MAX_LINEAR_SPEED = 0.2
    MAX_ANGULAR_SPEED = 0.3 

    factor_v = 1 # 0.3
    factor_w = 0.8 # 0.3

    # x_goal = 1.5 - 3 * random()
    # y_goal = 1.5 - 3 * random()
    # theta_goal = 2 * np.pi * random() - np.pi
    x_goal = -1.2
    y_goal = 1.2
    theta_goal = np.pi/2.0

    rospy.init_node('go2pos', anonymous=True)
    sub = rospy.Subscriber('/pose', PoseStamped, callback, queue_size=10)
    pub = rospy.Publisher('/cmd_vel_go2pose', Twist, queue_size=10)
    # pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.spin()