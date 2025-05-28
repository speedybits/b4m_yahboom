#!/usr/bin/env python3
# encoding: utf-8
import threading
import cv2 as cv
import numpy as np
from yahboom_esp32ai_car.media_library import *
import  time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool,UInt16
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage

from rclpy.time import Time
import datetime


class PoseCtrlArm(Node):
    def __init__(self,name):
        super().__init__(name)
        self.pub_Servo1 = self.create_publisher(Int32,"servo_s1" , 10)
        self.pub_Servo2 = self.create_publisher(Int32,"servo_s2" , 10)
        self.x = 0
        self.y = 45
        servo1_angle = Int32()
        servo2_angle = Int32()

        servo1_angle.data = self.x
        servo2_angle.data = self.y

        self.car_status = True
        self.stop_status = 0
        self.locking = False
        self.pose_detector = Holistic()
        self.hand_detector = HandDetector()
        self.pTime = self.index = 0
        self.media_ros = Media_ROS()
        self.event = threading.Event()
        self.event.set()

        #确保角度正常处于中间
        for i in range(10):
            self.pub_Servo1.publish(servo1_angle)
            self.pub_Servo2.publish(servo2_angle)
            time.sleep(0.1)

    def process(self, frame):
        #frame = cv.flip(frame, 1)
        if self.media_ros.Joy_active:
            
            frame, lmList, _ = self.hand_detector.findHands(frame)
            if len(lmList) != 0:
                threading.Thread(target=self.hand_threading, args=(lmList,)).start()
            else:self.media_ros.pub_vel(0.0, 0.0,0.0)
        self.media_ros.pub_imgMsg(frame)
        return frame

    def hand_threading(self, lmList):
        if self.event.is_set():
            self.event.clear()
            self.stop_status = 0
            self.index = 0
            fingers = self.hand_detector.fingersUp(lmList)
            print("fingers: ", fingers)
            if sum(fingers) == 5: 
                self.media_ros.pub_vel(0.3, 0.0,0.0)
                time.sleep(0.5)
                
            elif sum(fingers) == 0: 
                self.media_ros.pub_vel(-0.3, 0.0,0.0)
                time.sleep(0.5)
                
            elif sum(fingers) == 1 and fingers[1] == 1: 
                self.media_ros.pub_vel(0.0, 0.0, 0.5)
                time.sleep(0.5)
                
            elif sum(fingers) == 2 and fingers[1] == 1 and fingers[2] == 1: 
                self.media_ros.pub_vel(0.0, 0.0, -0.5)
                time.sleep(0.5)
            else:
                self.media_ros.pub_vel(0.0, 0.0, 0.0)
            self.event.set()

class MY_Picture(Node):
    def __init__(self, name):
        super().__init__(name)
        self.bridge = CvBridge()
        self.sub_img = self.create_subscription(
            CompressedImage, '/espRos/esp32camera', self.handleTopic, 1) #获取esp32传来的图像
        
        self.pose_ctrl_arm = PoseCtrlArm('posectrlarm')
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
        cv.waitKey(1)
        frame = self.pose_ctrl_arm.process(frame)
                
        end = time.time()
        fps = 1 / ((end - start)+self.fps_seconds)
        
        text = "FPS : " + str(int(fps))
        cv.putText(frame, text, (10,20), cv.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,255), 2)
        if len(binary) != 0: 
            cv.imshow('frame', ManyImgs(1, ([frame, binary])))
        else:
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

