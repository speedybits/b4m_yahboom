#!/usr/bin/env python3
# encoding: utf-8
import threading
import cv2 as cv
import numpy as np
from yahboom_esp32ai_car.media_library import *
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool,UInt16
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage

from rclpy.time import Time
import datetime


class HandCtrlArm(Node):
    def __init__(self,name):
        super().__init__(name)
        self.pub_Servo1 = self.create_publisher(Int32,"servo_s1" , 10)
        self.pub_Servo2 = self.create_publisher(Int32,"servo_s2" , 10)

        self.PWMServo_X = 0
        self.PWMServo_Y = 45
        self.s1_init_angle = Int32()
        self.s1_init_angle.data = self.PWMServo_X
        self.s2_init_angle = Int32()
        self.s2_init_angle.data = self.PWMServo_Y
        self.media_ros = Media_ROS()

        #确保角度正常处于中间
        for i in range(10):
            self.pub_Servo1.publish(self.s1_init_angle)
            self.pub_Servo2.publish(self.s2_init_angle)
            time.sleep(0.1)

        self.hand_detector = HandDetector()
        self.arm_status = True
        self.locking = True
        self.init = True
        self.pTime = 0
        self.add_lock = self.remove_lock = 0
        self.event = threading.Event()
        self.event.set()

    def process(self, frame):
        frame, lmList, bbox = self.hand_detector.findHands(frame)
        if len(lmList) != 0:
            threading.Thread(target=self.arm_ctrl_threading, args=(lmList,bbox)).start()
        else:
            self.media_ros.pub_vel(0.0,0.0,0.0)
        self.media_ros.pub_imgMsg(frame)
        return frame

    def arm_ctrl_threading(self, lmList,bbox):
        if self.event.is_set():
            self.event.clear()
            fingers = self.hand_detector.fingersUp(lmList)
            self.hand_detector.draw = True
            gesture = self.hand_detector.get_gesture(lmList)
            self.arm_status = False
            point_x = lmList[9][1]
            point_y = lmList[9][2]

            print("x",point_x)
            if point_y >= 270: x = -0.2
            elif point_y <= 150: x = 0.2
            else: x = 0.0
            if point_x >= 350: y = -0.2
            elif point_x <= 300: y = 0.2
            else: y = 0.0
            self.media_ros.pub_vel(x,0.0,y)
            print("angle: {},value: {}".format(x,y))
            self.arm_status = True
            self.event.set()


class MY_Picture(Node):
    def __init__(self, name):
        super().__init__(name)
        self.bridge = CvBridge()
        self.sub_img = self.create_subscription(
            CompressedImage, '/espRos/esp32camera', self.handleTopic, 1) #获取esp32传来的图像
        
        self.handctrlarm = HandCtrlArm('handctrl')
        self.last_stamp = None
        self.new_seconds = 0
        self.fps_seconds = 1

    def handleTopic(self, msg):
        self.last_stamp = msg.header.stamp  
        if self.last_stamp:
            total_secs = Time(nanoseconds=self.last_stamp.nanosec, seconds=self.last_stamp.sec).nanoseconds
            delta = datetime.timedelta(seconds=total_secs * 1e-9)
            seconds = delta.total_seconds()*100

            if self.new_seconds != 0:
                self.fps_seconds = seconds - self.new_seconds

            self.new_seconds = seconds#保留这次的值

        start = time.time()
        frame = self.bridge.compressed_imgmsg_to_cv2(msg)
        frame = cv.resize(frame, (640, 480))

        action = cv.waitKey(1) & 0xFF
        frame = self.handctrlarm.process(frame)
        if action == ord('q'):
            self.handctrlarm.media_ros.cancel()

        
        end = time.time()
        fps = 1 / ((end - start)+self.fps_seconds)
        
        text = "FPS : " + str(int(fps))
        cv.putText(frame, text, (10,20), cv.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,255), 2)
        cv.imshow('frame', frame)



def main():
    rclpy.init() 
    esp_img = MY_Picture("My_Picture")
    print("start it")
    try:
        rclpy.spin(esp_img)
    except KeyboardInterrupt:
        pass
    finally:
        esp_img.destroy_node()
        rclpy.shutdown()
