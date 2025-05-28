import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32, Bool,UInt16
from geometry_msgs.msg import Twist
import cv2
import time
import numpy as np
import pyzbar.pyzbar as pyzbar
from PIL import Image

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage

from rclpy.time import Time
import datetime

class QR_Tracker(Node):
    def __init__(self,name):
        super().__init__(name)
        self.pub_cmdVel = self.create_publisher(Twist,"/cmd_vel",1)
        self.pub_Buzzer = self.create_publisher(UInt16,"/beep",1)
        self.pub_Servo1 = self.create_publisher(Int32,"servo_s1" , 10)
        self.pub_Servo2 = self.create_publisher(Int32,"servo_s2" , 10)

        self.PWMServo_X = 0
        self.PWMServo_Y = 0
        self.s1_init_angle = Int32()
        self.s1_init_angle.data = self.PWMServo_X
        self.s2_init_angle = Int32()
        self.s2_init_angle.data = self.PWMServo_Y

        for i in range(10): #确保舵机初始化角度正常
            self.pub_Servo2.publish(self.s2_init_angle)
            self.pub_Servo1.publish(self.s1_init_angle)
            time.sleep(0.1) #100ms



    def detect_qrcode(self,image):
        #self.pub_Servo2.publish(self.s2_init_angle)
        #self.pub_Servo1.publish(self.s1_init_angle)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        barcodes = pyzbar.decode(gray)
        for barcode in barcodes:
            (x, y, w, h) = barcode.rect
            barcodeData = barcode.data.decode("utf-8")
            barcodeType = barcode.type
            return barcodeData, (x, y, w, h)
        return None, (0, 0, 0, 0)

    def pub_vel(self, x, y, z):
        twist = Twist()
        twist.linear.x = x
        twist.linear.y = y
        twist.angular.z = z
        self.pub_cmdVel.publish(twist)
    

    def robot_action(self,data):
        if data == "forward":
            self.pub_vel(0.3,0.0,0.0)

        elif data == "back":
            self.pub_vel(-0.3,0.0,0.0)
            
        elif data == "left":
            self.pub_vel(0.0,0.0,1.0)
            
        elif data == "right":
            self.pub_vel(0.0,0.0,-1.0)

        elif data == "turnright":
            self.pub_vel(0.3,0.0,-0.5)

        elif data == "turnleft":
            self.pub_vel(0.3,0.0,0.5)

        elif data == "stop":
            self.pub_vel(0.0,0.0,0.0)



class MY_Picture(Node):
    def __init__(self, name):
        super().__init__(name)
        self.bridge = CvBridge()
        self.sub_img = self.create_subscription(
            CompressedImage, '/espRos/esp32camera', self.handleTopic, 1) #获取esp32传来的图像

        self.QRdetect = QR_Tracker("QR_Tracker")
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
        frame = cv2.resize(frame, (640, 480))
        action = cv2.waitKey(10) & 0xFF

        payload, (x, y, w, h) = self.QRdetect.detect_qrcode(frame.copy())
        if payload != None:
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 225, 255), 2)
            cv2.putText(frame, payload, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 225, 255), 2)
            self.QRdetect.robot_action(payload)
        else:
            self.QRdetect.pub_vel(0.0,0.0,0.0)
        
        
        end = time.time()
        fps = 1 / ((end - start)+self.fps_seconds)
        
        text = "FPS : " + str(int(fps))
        cv2.putText(frame, text, (10,20), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,255), 2)
        cv2.imshow('frame', frame)

        #rclpy.spin(self.QRdetect)
        


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
    



















