#!/usr/bin/env python3
# coding=utf-8

import rospy
import os
import cv2
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import std_msgs.msg
from cv_bridge import CvBridge
from std_msgs.msg import Empty
import math
import sys
import mediapipe as mp

def callback(msg):

    cap = bridge.imgmsg_to_cv2(msg)
    
    cap, gest= detect(cap)
    if gest == "stop":
        speed = Twist()
        speed.linear.x = 0.0
        speed.linear.y = 0.0
        speed.linear.z = 0.0 
        speed.angular.x = 0.0
        speed.angular.y = 0.0
        speed.angular.z = 0.0
        pub.publish(speed)
    elif gest == "up":
        speed = Twist()
        speed.linear.x = 0.0
        speed.linear.y = 0.0
        speed.linear.z = 0.2
        speed.angular.x = 0.0
        speed.angular.y = 0.0
        speed.angular.z = 0.0
        pub.publish(speed)
    elif gest == "down":
        speed = Twist()
        speed.linear.x = 0.0
        speed.linear.y = 0.0
        speed.linear.z = -0.2
        speed.angular.x = 0.0
        speed.angular.y = 0.0
        speed.angular.z = 0.0
        pub.publish(speed)
    elif gest == "left":
        speed = Twist()
        speed.linear.x = 0.0
        speed.linear.y = -0.2
        speed.linear.z = 0.0 
        speed.angular.x = 0.0
        speed.angular.y = 0.0
        speed.angular.z = 0.0
        pub.publish(speed)
    elif gest == "right":
        speed = Twist()
        speed.linear.x = 0.0
        speed.linear.y = 0.2
        speed.linear.z = 0.0 
        speed.angular.x = 0.0
        speed.angular.y = 0.0
        speed.angular.z = 0.0
        pub.publish(speed)
    elif gest == "turn":
        speed = Twist()
        speed.linear.x = 0.0
        speed.linear.y = 0.0
        speed.linear.z = 0.0 
        speed.angular.x = 0.0
        speed.angular.y = 0.0
        speed.angular.z = 0.2
        pub.publish(speed)
    elif gest == "backward":
        speed = Twist()
        speed.linear.x = -0.2
        speed.linear.y = 0.0
        speed.linear.z = 0.0 
        speed.angular.x = 0.0
        speed.angular.y = 0.0
        speed.angular.z = 0.0
        pub.publish(speed)
    elif gest == "forward":
        speed = Twist()
        speed.linear.x = 0.2
        speed.linear.y = 0.0
        speed.linear.z = 0.0 
        speed.angular.x = 0.0
        speed.angular.y = 0.0
        speed.angular.z = 0.0
        pub.publish(speed)
    elif gest == "land off":
        speed = Twist()
        speed.linear.x = 0.0
        speed.linear.y = 0.0
        speed.linear.z = 0.0 
        speed.angular.x = 0.0
        speed.angular.y = 0.0
        speed.angular.z = 0.0   
        land_pub = rospy.Publisher('/land', Empty, queue_size=1)
        empty_msg = Empty()
        # pub.publish(speed)
        land_pub.publish(empty_msg)
    cv2.imshow('Frame', cap)
    cv2.waitKey(1)

def angle(v1,v2):
    v1_x=v1[0]
    v1_y=v1[1]
    v2_x=v2[0]
    v2_y=v2[1]
    try:
        angle_= math.degrees(math.acos((v1_x*v2_x+v1_y*v2_y)/(((v1_x**2+v1_y**2)**0.5)*((v2_x**2+v2_y**2)**0.5))))
    except:
        angle_ =65535.
    if angle_ > 180.:
        angle_ = 65535.
    return angle_

def hand_angle(hand_):

    angle_list = []
    #---------------------------- thumb 大拇指角度
    angle_ = angle(
        ((int(hand_[0][0])- int(hand_[2][0])),(int(hand_[0][1])-int(hand_[2][1]))),
        ((int(hand_[3][0])- int(hand_[4][0])),(int(hand_[3][1])- int(hand_[4][1])))
        )
    angle_list.append(angle_)
    #---------------------------- index 食指角度
    angle_ = angle(
        ((int(hand_[0][0])-int(hand_[6][0])),(int(hand_[0][1])- int(hand_[6][1]))),
        ((int(hand_[7][0])- int(hand_[8][0])),(int(hand_[7][1])- int(hand_[8][1])))
        )
    angle_list.append(angle_)
    #---------------------------- middle 中指角度
    angle_ = angle(
        ((int(hand_[0][0])- int(hand_[10][0])),(int(hand_[0][1])- int(hand_[10][1]))),
        ((int(hand_[11][0])- int(hand_[12][0])),(int(hand_[11][1])- int(hand_[12][1])))
        )
    angle_list.append(angle_)
    #---------------------------- ring 无名指角度
    angle_ = angle(
        ((int(hand_[0][0])- int(hand_[14][0])),(int(hand_[0][1])- int(hand_[14][1]))),
        ((int(hand_[15][0])- int(hand_[16][0])),(int(hand_[15][1])- int(hand_[16][1])))
        )
    angle_list.append(angle_)
    #---------------------------- pink 小拇指角度
    angle_ = angle(
        ((int(hand_[0][0])- int(hand_[18][0])),(int(hand_[0][1])- int(hand_[18][1]))),
        ((int(hand_[19][0])- int(hand_[20][0])),(int(hand_[19][1])- int(hand_[20][1])))
        )
    angle_list.append(angle_)
    return angle_list

def h_gesture(hand_,angle_list):

    thr_angle = 65.
    thr_angle_thumb = 53.
    thr_angle_s = 49.
    gesture_str = None
    if 65535. not in angle_list:
        if (angle_list[0]>thr_angle_thumb) and (angle_list[1]>thr_angle) and (angle_list[2]>thr_angle) and (angle_list[3]>thr_angle) and (angle_list[4]>thr_angle):
            gesture_str = "land off"
        elif (angle_list[0]<thr_angle_s) and (angle_list[1]<thr_angle_s) and (angle_list[2]<thr_angle_s) and (angle_list[3]<thr_angle_s) and (angle_list[4]<thr_angle_s):
            gesture_str = "stop"
        elif (angle_list[0]<thr_angle_s)  and (angle_list[1]<thr_angle_s) and (angle_list[2]>thr_angle) and (angle_list[3]>thr_angle) and (angle_list[4]>thr_angle):
            if hand_[8][0] > hand_[6][0]:
                gesture_str = "left"
            elif hand_[8][0] < hand_[6][0]:
                gesture_str = "right"
        elif (angle_list[0]<thr_angle_s)  and (angle_list[1]<thr_angle_s) and (angle_list[2]>thr_angle) and (angle_list[3]>thr_angle) and (angle_list[4]<thr_angle_s):
            gesture_str = "turn"
        elif (angle_list[0]>5)  and (angle_list[1]<thr_angle_s) and (angle_list[2]>thr_angle) and (angle_list[3]>thr_angle) and (angle_list[4]>thr_angle):
            if hand_[4][1] > hand_[2][1]:
                gesture_str = "down"
            elif hand_[4][1] < hand_[2][1]:
                gesture_str = "up"
        elif (angle_list[0]>thr_angle_thumb)  and (angle_list[1]<thr_angle_s) and (angle_list[2]<thr_angle_s) and (angle_list[3]>thr_angle) and (angle_list[4]>thr_angle):
            if hand_[4][1] > hand_[2][1]:
                gesture_str = "backward"
            elif hand_[4][1] < hand_[2][1]:
                gesture_str = "forward"
                
    return gesture_str
    
def detect(frame):
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    #cap = cv2.VideoCapture(-1)
   
    #ret, frame = frame.read()
    gesture_str = None
    results = hands.process(frame)
    #frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        
    if results.multi_hand_landmarks:
            
        for hand_landmarks in results.multi_hand_landmarks:
            mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
            hand_local = []
        for i in range(21):
            x = hand_landmarks.landmark[i].x*frame.shape[1]
            y = hand_landmarks.landmark[i].y*frame.shape[0]
            hand_local.append((x,y))
        if hand_local:
            angle_list = hand_angle(hand_local)
            gesture_str = h_gesture(hand_local,angle_list)
            cv2.putText(frame,gesture_str,(0,100),0,1.3,(0,0,255),3)

    return frame, gesture_str

if __name__ == '__main__':
    mp_drawing = mp.solutions.drawing_utils
    mp_hands = mp.solutions.hands
    hands = mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.75,
            min_tracking_confidence=0.75)
    bridge = CvBridge()
    rospy.init_node('gesture', anonymous=True)
    sub = rospy.Subscriber('/image_raw', Image, callback)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.spin()
    

