#ros lib
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage, LaserScan, Image
from yahboomcar_msgs.msg import Position
from cv_bridge import CvBridge
#common lib
import os
import threading
import math
from yahboom_esp32ai_car.astra_common import *
from yahboomcar_msgs.msg import Position

from rclpy.time import Time
import datetime

print("import finish")
cv_edition = cv.__version__
print("cv_edition: ",cv_edition)
class Color_Identify(Node):
    def __init__(self,name):
        super().__init__(name)
        #create a publisher
        self.pub_position = self.create_publisher(Position,"/Current_point", 10)
        self.pub_cmdVel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_img = self.create_publisher(Image,'/image_raw',500)
        self.bridge = CvBridge()
        self.index = 2
        self.Roi_init = ()
        self.hsv_range = ()
        self.end = 0
        self.circle = (0, 0, 0)
        self.point_pose = (0, 0, 0)
        self.dyn_update = True
        self.Start_state = True
        self.select_flags = False
        self.gTracker_state = False
        self.windows_name = 'frame'
        self.Track_state = 'identify'
        self.color = color_follow()
        self.cols, self.rows = 0, 0
        self.Mouse_XY = (0, 0)
        self.declare_param()

        
        self.hsv_text = "/home/yahboom/yahboomcar_ws/src/yahboom_esp32ai_car/yahboom_esp32ai_car/colorHSV.text"
        #self.timer = self.create_timer(0.001, self.on_timer)
        
    def declare_param(self):
        #HSV
        self.declare_parameter("Hmin",0)
        self.Hmin = self.get_parameter('Hmin').get_parameter_value().integer_value
        self.declare_parameter("Smin",85)
        self.Smin = self.get_parameter('Smin').get_parameter_value().integer_value
        self.declare_parameter("Vmin",126)
        self.Vmin = self.get_parameter('Vmin').get_parameter_value().integer_value
        self.declare_parameter("Hmax",9)
        self.Hmax = self.get_parameter('Hmax').get_parameter_value().integer_value
        self.declare_parameter("Smax",253)
        self.Smax = self.get_parameter('Smax').get_parameter_value().integer_value
        self.declare_parameter("Vmax",253)
        self.Vmax = self.get_parameter('Vmax').get_parameter_value().integer_value
        self.declare_parameter('refresh',False)
        self.refresh = self.get_parameter('refresh').get_parameter_value().bool_value

    #改成my_picturc里面使用即可
    # def on_timer(self):
    #     ret, frame = self.capture.read()
    #     action = cv.waitKey(10) & 0xFF
    #     frame, binary =self.process(frame, action)
    #     start = time.time()
    #     fps = 1 / (start - self.end)
    #     text = "FPS : " + str(int(fps))
    #     self.end = start
    #     cv.putText(frame, text, (30, 30), cv.FONT_HERSHEY_SIMPLEX, 0.6, (100, 200, 200), 1)
    #     if len(binary) != 0: cv.imshow('frame', ManyImgs(1, ([frame, binary])))
    #     else:cv.imshow('frame', frame)
    #     msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
    #     self.pub_img.publish(msg)
    #     if action == ord('q') or action == 113:
    #         self.capture.release()
    #         cv.destroyAllWindows()
            
    def process(self, rgb_img, action):
        self.get_param()
        rgb_img = cv.resize(rgb_img, (640, 480))
        binary = []
        if action == 32: self.Track_state = 'tracking' #空格键值
        elif action == ord('i') or action == ord('I'): self.Track_state = "identify"
        elif action == ord('r') or action == ord('R'): self.Reset()
        elif action == ord('q') or action == ord('Q'): self.cancel()
        if self.Track_state == 'init':
            cv.namedWindow(self.windows_name, cv.WINDOW_AUTOSIZE)
            cv.setMouseCallback(self.windows_name, self.onMouse, 0)
            if self.select_flags == True:
                cv.line(rgb_img, self.cols, self.rows, (255, 0, 0), 2)
                cv.rectangle(rgb_img, self.cols, self.rows, (0, 255, 0), 2)
                if self.Roi_init[0] != self.Roi_init[2] and self.Roi_init[1] != self.Roi_init[3]:
                    rgb_img, self.hsv_range = self.color.Roi_hsv(rgb_img, self.Roi_init)
                    self.gTracker_state = True
                    self.dyn_update = True
                else: self.Track_state = 'init'
        elif self.Track_state == "identify":
            if os.path.exists(self.hsv_text): self.hsv_range = read_HSV(self.hsv_text)
            else: self.Track_state = 'init'
        if self.Track_state != 'init':
            if len(self.hsv_range) != 0:
                rgb_img, binary, self.circle = self.color.object_follow(rgb_img, self.hsv_range)
                if self.dyn_update == True:
                    write_HSV(self.hsv_text, self.hsv_range)
                    self.Hmin  = rclpy.parameter.Parameter('Hmin',rclpy.Parameter.Type.INTEGER,self.hsv_range[0][0])
                    self.Smin  = rclpy.parameter.Parameter('Smin',rclpy.Parameter.Type.INTEGER,self.hsv_range[0][1])
                    self.Vmin  = rclpy.parameter.Parameter('Vmin',rclpy.Parameter.Type.INTEGER,self.hsv_range[0][2])
                    self.Hmax  = rclpy.parameter.Parameter('Hmax',rclpy.Parameter.Type.INTEGER,self.hsv_range[1][0])
                    self.Smax  = rclpy.parameter.Parameter('Smax',rclpy.Parameter.Type.INTEGER,self.hsv_range[1][1])
                    self.Vmax  = rclpy.parameter.Parameter('Vmax',rclpy.Parameter.Type.INTEGER,self.hsv_range[1][2])
                    all_new_parameters = [self.Hmin,self.Smin,self.Vmin,self.Hmax,self.Smax,self.Vmax]
                    self.set_parameters(all_new_parameters)
                    
                    self.dyn_update = False     
        if self.Track_state == 'tracking':
            self.Start_state = True
            if self.circle[2] != 0: threading.Thread(
                target=self.execute, args=(self.circle[0], self.circle[1], self.circle[2])).start()
            if self.point_pose[0] != 0 and self.point_pose[1] != 0: threading.Thread(
                target=self.execute, args=(self.point_pose[0], self.point_pose[1], self.point_pose[2])).start()
        else:
            if self.Start_state == True:
                self.pub_cmdVel.publish(Twist())
                self.Start_state = False
        return rgb_img, binary
    
    def execute(self, x, y, z):
        position = Position()
        position.anglex = x * 1.0
        position.angley = y * 1.0
        position.distance = z * 1.0
        self.pub_position.publish(position)

    def get_param(self):
        #hsv
        self.Hmin = self.get_parameter('Hmin').get_parameter_value().integer_value
        self.Smin = self.get_parameter('Smin').get_parameter_value().integer_value
        self.Vmin = self.get_parameter('Vmin').get_parameter_value().integer_value
        self.Hmax = self.get_parameter('Hmax').get_parameter_value().integer_value
        self.Smax = self.get_parameter('Smax').get_parameter_value().integer_value
        self.Vmax = self.get_parameter('Vmax').get_parameter_value().integer_value
        self.refresh = self.get_parameter('refresh').get_parameter_value().bool_value
        
    def Reset(self):
        self.hsv_range = ()
        self.circle = (0, 0, 0)
        self.Mouse_XY = (0, 0)
        self.Track_state = 'init'
        for i in range(3): self.pub_position.publish(Position())
        print("succes!!!")
        
    def cancel(self):
        print("Shutting down this node.")
        cv.destroyAllWindows()
        
    def onMouse(self, event, x, y, flags, param):
        if event == 1:
            self.Track_state = 'init'
            self.select_flags = True
            self.Mouse_XY = (x, y)
        if event == 4:
            self.select_flags = False
            self.Track_state = 'mouse'
        if self.select_flags == True:
            self.cols = min(self.Mouse_XY[0], x), min(self.Mouse_XY[1], y)
            self.rows = max(self.Mouse_XY[0], x), max(self.Mouse_XY[1], y)
            self.Roi_init = (self.cols[0], self.cols[1], self.rows[0], self.rows[1])
        




class MY_Picture(Node):
    def __init__(self, name):
        super().__init__(name)
        self.bridge = CvBridge()
        self.sub_img = self.create_subscription(
            CompressedImage, '/espRos/esp32camera', self.handleTopic, 1) #获取esp32传来的图像

        self.color_identify = Color_Identify("ColorIdentify")
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

        action = cv.waitKey(10) & 0xFF
        frame, binary =self.color_identify.process(frame, action)
        if len(binary) != 0: cv.imshow('frame', ManyImgs(1, ([frame, binary])))
        else:cv.imshow('frame', frame)
        msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        self.color_identify.pub_img.publish(msg)       
        
        end = time.time()
        fps = 1/((end - start)+self.fps_seconds) 
        
        text = "FPS : " + str(int(fps))
        
        cv.putText(frame, text, (10,20), cv.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,255), 2)
        if len(binary) != 0: 
            cv.imshow('frame', ManyImgs(1, ([frame, binary])))
        else:
            cv.imshow('frame', frame)

        if action == ord('q') or action == 113:
            cv.destroyAllWindows()

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

