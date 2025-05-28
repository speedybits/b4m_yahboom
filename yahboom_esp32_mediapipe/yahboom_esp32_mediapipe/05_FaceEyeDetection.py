#!/usr/bin/env python2
# encoding: utf-8
#import ros lib
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import mediapipe as mp
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage,Image
#import define msg
from yahboomcar_msgs.msg import PointArray
#import commom lib
import cv2 as cv
import numpy as np
import time
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage

from rclpy.time import Time
import datetime

print("import done")


class FaceEyeDetection(Node):
    def __init__(self,name):
        super().__init__(name)
        self.bridge = CvBridge()
        self.eyeDetect = cv.CascadeClassifier( "/home/yahboom/yahboomcar_ws/src/yahboom_esp32_mediapipe/yahboom_esp32_mediapipe/file/haarcascade_eye.xml")
        self.faceDetect = cv.CascadeClassifier("/home/yahboom/yahboomcar_ws/src/yahboom_esp32_mediapipe/yahboom_esp32_mediapipe/file/haarcascade_eye.xml")
        self.pub_rgb = self.create_publisher(Image,"/FaceEyeDetection/image", 500)

    def cancel(self):
        self.pub_rgb.unregister()

    def face(self, frame):
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        faces = self.faceDetect.detectMultiScale(gray, 1.3)
        for face in faces: frame = self.faceDraw(frame, face)
        return frame

    def eye(self, frame):
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        eyes = self.eyeDetect.detectMultiScale(gray, 1.3)
        for eye in eyes:
            cv.circle(frame, (int(eye[0] + eye[2] / 2), int(eye[1] + eye[3] / 2)), (int(eye[3] / 2)), (0, 0, 255), 2)
        return frame

    def faceDraw(self, frame, bbox, l=30, t=10):
        x, y, w, h = bbox
        x1, y1 = x + w, y + h
        cv.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 255), 2)
        # Top left x,y
        cv.line(frame, (x, y), (x + l, y), (255, 0, 255), t)
        cv.line(frame, (x, y), (x, y + l), (255, 0, 255), t)
        # Top right x1,y
        cv.line(frame, (x1, y), (x1 - l, y), (255, 0, 255), t)
        cv.line(frame, (x1, y), (x1, y + l), (255, 0, 255), t)
        # Bottom left x1,y1
        cv.line(frame, (x, y1), (x + l, y1), (255, 0, 255), t)
        cv.line(frame, (x, y1), (x, y1 - l), (255, 0, 255), t)
        # Bottom right x1,y1
        cv.line(frame, (x1, y1), (x1 - l, y1), (255, 0, 255), t)
        cv.line(frame, (x1, y1), (x1, y1 - l), (255, 0, 255), t)
        return frame

    def pub_img(self, frame):
        self.pub_rgb.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

class MY_Picture(Node):
    def __init__(self, name):
        super().__init__(name)
        self.bridge = CvBridge()
        self.sub_img = self.create_subscription(
            CompressedImage, '/espRos/esp32camera', self.handleTopic, 1) #获取esp32传来的图像

        self.last_stamp = None
        self.new_seconds = 0
        self.fps_seconds = 1
        self.face_eye_detection = FaceEyeDetection('face_eye_detection')
        self.content = ["face", "eye", "face_eye"]
        self.content_index = 0

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
        cv.waitKey(10)

        
        action = cv.waitKey(1) & 0xFF
        if action == ord("f") or action == ord("F"):
            self.content_index += 1
            if self.content_index >= len(self.content): self.content_index = 0
        if self.content[self.content_index] == "face": frame = self.face_eye_detection.face(frame)
        elif self.content[self.content_index] == "eye": frame = self.face_eye_detection.eye(frame)
        else: frame = self.face_eye_detection.eye(self.face_eye_detection.face(frame))
                
        end = time.time()
        fps = 1 / ((end - start)+self.fps_seconds)
        text = "FPS : " + str(int(fps))
        cv.putText(frame, text, (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 1)

        cv.imshow('frame', frame)

        self.face_eye_detection.pub_img(frame)
        # print(frame)
    
        cv.waitKey(10)

def main():
    print("start it")
    rclpy.init()
    esp_img = MY_Picture("My_Picture")
    try:
            rclpy.spin(esp_img)
    except KeyboardInterrupt:
        pass
    finally:
        esp_img.destroy_node()
        rclpy.shutdown()
