#!/usr/bin/python3
# coding=utf-8
# 环境准备：pip install opencv_contrib_python
# 输入话题：tianbot_mini/image_raw/compressed
# 输出话题：roi

import sys
import os

import rospy
import sensor_msgs.msg
from cv_bridge import CvBridge
import cv2
import numpy as np
from sensor_msgs.msg import RegionOfInterest as ROI
from sensor_msgs.msg import CompressedImage
 
br = CvBridge()

class MessageItem(object):
    # 用于封装信息的类,包含图片和其他信息
    def __init__(self,frame,message):
        self._frame = frame
        self._message = message
 
    def getFrame(self):
        # 图片信息
        return self._frame
 
    def getMessage(self):
        #文字信息,json格式
        return self._message
 
 
class Tracker(object):
    '''
    追踪者模块,用于追踪指定目标
    '''
 
    def __init__(self, tracker_type="TLD", draw_coord=True):
        '''
        初始化追踪器种类
        '''
        # 获得opencv版本
        (major_ver, minor_ver, subminor_ver) = (cv2.__version__).split('.')
        self.tracker_types = ['BOOSTING', 'MIL', 'KCF', 'TLD', 'MEDIANFLOW', 'GOTURN', "CSRT"]
        self.tracker_type = tracker_type
        self.isWorking = False
        self.draw_coord = draw_coord
        # 构造追踪器
        if int(major_ver) < 3:
            self.tracker = cv2.Tracker_create(tracker_type)
        else:
            if tracker_type == 'BOOSTING':
                self.tracker = cv2.TrackerBoosting_create()
            if tracker_type == 'MIL':
                self.tracker = cv2.TrackerMIL_create()
            if tracker_type == 'KCF':
                self.tracker = cv2.TrackerKCF_create()
            if tracker_type == 'TLD':
                self.tracker = cv2.TrackerTLD_create()
            if tracker_type == 'MEDIANFLOW':
                self.tracker = cv2.TrackerMedianFlow_create()
            if tracker_type == 'GOTURN':
                self.tracker = cv2.TrackerGOTURN_create()
            if tracker_type == "CSRT":
                self.tracker = cv2.TrackerCSRT_create()
 
    def initWorking(self, frame, box):
        '''
        追踪器工作初始化
        frame:初始化追踪画面
        box:追踪的区域
        '''
        if not self.tracker:
            raise Exception("追踪器未初始化")
        status = self.tracker.init(frame, box)
        if not status:
            raise Exception("追踪器工作初始化失败")
        self.coord = box
        self.isWorking = True
 
    def track(self, frame):
        '''
        开启追踪
        '''
        message = None
        if self.isWorking:
            status, self.coord = self.tracker.update(frame)
            if status:
                message = {"coord": [((int(self.coord[0]), int(self.coord[1])),
                                      (int(self.coord[0] + self.coord[2]), int(self.coord[1] + self.coord[3])))]}
                if self.draw_coord:
                    p1 = (int(self.coord[0]), int(self.coord[1]))
                    p2 = (int(self.coord[0] + self.coord[2]), int(self.coord[1] + self.coord[3]))
                    cv2.rectangle(frame, p1, p2, (255, 0, 0), 2, 1)
                    message['msg'] = self.tracker_type + " is tracking"

                    # 更新ROI
                    if (int(self.coord[0]) <0 or int(self.coord[1]) <0):
                        tld_roi.x_offset = 0
                        tld_roi.y_offset = 0
                        tld_roi.width = 0
                        tld_roi.height = 0
                    else:
                        tld_roi.x_offset = int(self.coord[0])
                        tld_roi.y_offset = int(self.coord[1])
                        tld_roi.width = int(self.coord[2])
                        tld_roi.height = int(self.coord[3])
                    # 发布ROI
                    pub.publish(tld_roi)
                    
        return MessageItem(frame, message)
 
def compressed_detect_and_draw(compressed_imgmsg):
    global br,gFrame,gCapStatus,getFrame,loopGetFrame
    if ((getFrame == True) or (loopGetFrame == True)):
        gFrame = br.compressed_imgmsg_to_cv2(compressed_imgmsg, "bgr8")
        gCapStatus = True
        getFrame = True
        

gFrame = np.zeros((640,640,3), np.uint8)
gCapStatus = False
getFrame = True
loopGetFrame = False

if __name__ == '__main__':
 
    rospy.init_node('tbm_tld_tracker_node')
    rospy.Subscriber("/image_raw", sensor_msgs.msg.CompressedImage, compressed_detect_and_draw)
    pub = rospy.Publisher("roi",ROI,queue_size=10)
    tld_roi = ROI()
    # rate = rospy.Rate(10)
    # rate.sleep()
    # 选择 框选帧
    print("按 n 渲染下一帧，按 y 设定当前帧作为ROI区域选择帧")
    while True:
        _key = cv2.waitKey(0) & 0xFF
        if(_key == ord('n')):
            # gCapStatus,gFrame = gVideoDevice.read()
            getFrame = True
        if(_key == ord('y')):
            break

        cv2.imshow("Pick frame",gFrame)

    # 框选感兴趣区域region of interest
    cv2.destroyWindow("Pick frame")
    gROI = cv2.selectROI("ROI frame",gFrame,False)
    if (not gROI):
        print("空框选，退出")
        quit()

    # 初始化追踪器
    gTracker = Tracker(tracker_type="TLD")
    gTracker.initWorking(gFrame,gROI)

    # 循环帧读取，开始跟踪
    while not rospy.is_shutdown():
        # gCapStatus, gFrame = gVideoDevice.read()
        loopGetFrame = True
        if(gCapStatus):
            # 展示跟踪图片
            _item = gTracker.track(gFrame)
            cv2.imshow("Track result",_item.getFrame())

            if _item.getMessage():
                # 打印跟踪数据
                print(_item.getMessage())

            _key = cv2.waitKey(1) & 0xFF
            if (_key == ord('q')) | (_key == 27):
                break
            if (_key == ord('r')) :
                # 用户请求用初始ROI
                print("用户请求用初始ROI")
                gTracker = Tracker(tracker_type="TLD")
                gTracker.initWorking(gFrame, gROI)

        else:
            print("捕获帧失败")
            quit()
