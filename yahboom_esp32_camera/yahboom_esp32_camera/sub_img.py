import rclpy
import time
from rclpy.node import Node
import cv2 as cv
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage

from rclpy.time import Time
import datetime


class SubImg(Node):
    def __init__(self, name):
        super().__init__(name)
        self.bridge = CvBridge()
        self.sub_img = self.create_subscription(
            CompressedImage, '/espRos/esp32camera', self.handleTopic, 1)
        self.pub_img = self.create_publisher(Image, "/esp32_img", 1)

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

            #print(self.fps_seconds)
            
        start = time.time()
      

        frame = self.bridge.compressed_imgmsg_to_cv2(msg)
        # print(frame)
        frame = cv.resize(frame, (640, 480))
        cv.waitKey(10)
        end = time.time()
        #print(self.fps_seconds)
        fps = 1/((end - start)+self.fps_seconds) 
        text = "FPS : " + str(int(fps))
        cv.putText(frame, text, (30, 30), cv.FONT_HERSHEY_SIMPLEX,
                   0.6, (100, 200, 200), 1)
        # msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")

        cv.imshow("color_image", frame)
        img_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        self.pub_img.publish(img_msg)
        cv.waitKey(10)


def main():
    rclpy.init()
    esp_img = SubImg("sub_img")
    try:
        rclpy.spin(esp_img)
    except KeyboardInterrupt:
        pass
    finally:
        esp_img.destroy_node()
        rclpy.shutdown()
