#!/usr/bin/env python3

from cmath import pi
import rospy
import rospkg
import std_msgs.msg
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import RegionOfInterest as ROI
import cv2
from cv_bridge import CvBridge
import numpy as np

# resize the image to w*h
w = 360
h = 270

def callback(msg):
    image = bridge.imgmsg_to_cv2(msg)
    image = cv2.resize(image, (w, h))

    img, info = findCircle(image)
    circle_msg = trackCircle(info)
    
    rc = 'cx:' + str(circle_msg.x_offset) + 'cy:' + str(circle_msg.y_offset) + 'radius:' + str(circle_msg.width)
    rospy.loginfo(rc)  
    pub.publish(circle_msg)
     
    cv2.imshow('Frame', img)
    cv2.waitKey(1)

def findCircle(img):
    imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    imgGray = cv2.medianBlur(imgGray,5)
    circles = cv2.HoughCircles(imgGray, cv2.HOUGH_GRADIENT, dp=1, minDist=70, param1=50, param2=50, minRadius=60, maxRadius=120)
    myCircleListC = []
    myCircleListArea = []
    if circles is not None:
        circles = np.uint16(np.around(circles))

    # get the circle of the largest bounding box
        for (x,y,r) in circles[0,:]:
            cv2.circle(img,(x,y),r,(0,255,0),2)
    #画个圆心
            cv2.circle(img,(x,y),2,(0,255,255),2)
            area = r*r*pi
            myCircleListArea.append(area)
            myCircleListC.append([x, y])
        i = myCircleListArea.index(max(myCircleListArea))
        return img, [myCircleListC[i], myCircleListArea[i]]
    else:
        return img, [[0, 0], 0]

def trackCircle(info):
    circle_msg = ROI()
    if int(info[1]) == 0:
        circle_msg.x_offset = 0
        circle_msg.y_offset = 0
        circle_msg.width = 0
        circle_msg.height = 0

    else:
        circle_msg.width = int(np.sqrt(info[1]/pi))
        circle_msg.x_offset = int(info[0][0])
        circle_msg.y_offset = int(info[0][1])
        circle_msg.height = 0
    return (circle_msg)

if __name__ == '__main__':

    bridge = CvBridge()
    rospy.init_node('circle_tracker', anonymous=True)
    sub = rospy.Subscriber('/image_raw', Image, callback)
    pub = rospy.Publisher('/circle_msg', ROI, queue_size=1)
    
    rospy.spin()
