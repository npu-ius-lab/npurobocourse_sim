#!/usr/bin/env python3

# from msilib import datasizemask
from operator import le
import rospy
# import rospkg
import std_msgs.msg
from geometry_msgs.msg import Twist, PoseStamped
from tf.transformations import euler_from_quaternion
import numpy as np
import sys
sys.path.append("/home/tyang/Downloads/PythonRobotics/PathPlanning/CubicSpline/")

try:
    import potential_field_bezier
    import cubic_spline_planner
except:
    raise

class State(object):
    """
    Class representing the state of a vehicle.

    :param x: (float) x-coordinate
    :param y: (float) y-coordinate
    :param yaw: (float) yaw angle
    :param v: (float) speed
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        """Instantiate the object."""
        super(State, self).__init__()
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v


def pid_control(target, current):
    """
    Proportional control for the speed.

    :param target: (float)
    :param current: (float)
    :return: (float)
    """
    return Kp * (target - current)


def stanley_control(state, cx, cy, cyaw, last_target_idx):
    """
    Stanley steering control.

    :param state: (State object)
    :param cx: ([float])
    :param cy: ([float])
    :param cyaw: ([float])
    :param last_target_idx: (int)
    :return: (float, int)
    """
    current_target_idx, error_front_axle = calc_target_index(state, cx, cy)

    if last_target_idx >= current_target_idx:
        current_target_idx = last_target_idx

    # theta_e corrects the heading error
    theta_e = normalize_angle(cyaw[current_target_idx] - state.yaw)
    # theta_d corrects the cross track error
    theta_d = np.arctan2(k * error_front_axle, state.v)
    # Steering control
    delta = theta_e + theta_d

    return delta, current_target_idx

def normalize_angle(angle):
    """
    Normalize an angle to [-pi, pi].

    :param angle: (float)
    :return: (float) Angle in radian in [-pi, pi]
    """
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle

def calc_target_index(state, cx, cy):
    """
    Compute index in the trajectory list of the target.

    :param state: (State object)
    :param cx: [float]
    :param cy: [float]
    :return: (int, float)
    """
    # Calc front axle position
    fx = state.x + L * np.cos(state.yaw)
    fy = state.y + L * np.sin(state.yaw)

    # Search nearest point index
    dx = [fx - icx for icx in cx]
    dy = [fy - icy for icy in cy]
    d = np.hypot(dx, dy)
    target_idx = np.argmin(d)

    # Project RMS error onto front axle 
    # theta_e corrects vector
    front_axle_vec = [-np.cos(state.yaw + np.pi / 2),
                      -np.sin(state.yaw + np.pi / 2)]
    error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)

    return target_idx, error_front_axle

def callback(msg):
   
    x = msg.pose.position.x
    y = msg.pose.position.y
    z = msg.pose.position.z
    quaternion = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
    theta = euler_from_quaternion(quaternion)[2]

    rc = "pose_x: " + str(x) + " pose_y: " + str(y) + " pose_z: " + str(z) + " theta: " + str(theta)
    rospy.loginfo(rc)

    state.x = x
    state.y = y
    state.yaw = theta
    state.v = v

    target_idx, _ = calc_target_index(state, cx, cy)

    if last_idx > target_idx:
        di, target_idx = stanley_control(state, cx, cy, cyaw, target_idx)
        di = np.clip(di, -max_steer, max_steer)
        w = v / L * np.tan(di)
        w = w / np.pi * 180 / 100
        w = np.clip(w, -max_w, max_w)
        vel_msg = Twist()
        vel_msg.linear.x = v * factor_v
        vel_msg.angular.z = w * factor_w
    else:
        vel_msg = Twist()
        
    pub.publish(vel_msg)

if __name__ == '__main__':
    k = 1.0  # control gain
    Kp = 1.0  # speed proportional gain
    dt = 0.1  # [s] time difference
    L = 0.5  # [m] Wheel base of vehicle
    max_steer = np.radians(60.0)  # [rad] max steering angle
    v = 0.2 # 1.7
    max_w = 0.3
    factor_v = 0.5 # 0.1
    factor_w = 1.5 # 0.07
    state = State()

    """Plot an example of Stanley steering control on a cubic spline."""
    #  target course
    # ax = [0.0, 1.2, 1.2, 0.5, 0.6]
    # ay = [0.0, 0.0, -1.2, -0.6, 0.0]

    # ax, ay, ayaw =potential_field_bezier.planning()
    # cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
    #     ax, ay, ds=0.01)

    cx, cy, cyaw =potential_field_bezier.planning()

    last_idx = len(cx) - 1

    rospy.init_node('stanley', anonymous=True)
    sub = rospy.Subscriber('/pose', PoseStamped, callback, queue_size=10)
    pub = rospy.Publisher('/cmd_vel_stanley', Twist, queue_size=10)
    rospy.spin()
