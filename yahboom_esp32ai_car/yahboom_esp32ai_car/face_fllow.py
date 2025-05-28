#ros lib
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage, LaserScan, Image
from yahboomcar_msgs.msg import Position
from std_msgs.msg import Int32, Bool,UInt16
#common lib
import os
import threading
import math
import cv2
from yahboom_esp32ai_car.astra_common import *
from yahboomcar_msgs.msg import Position
from cv_bridge import CvBridge
from std_msgs.msg import Int32, Bool,UInt16

from rclpy.time import Time
import datetime
print("import done")


class faceTracker(Node):
    def __init__(self,name):
        super().__init__(name)
        #create the publisher
        self.pub_Servo1 = self.create_publisher(Int32,"servo_s1" , 10)
        self.pub_Servo2 = self.create_publisher(Int32,"servo_s2" , 10)
        #create the subscriber
        #self.sub_depth = self.create_subscription(Image,"/image_raw", self.depth_img_Callback, 1)
        self.sub_JoyState = self.create_subscription(Bool,'/JoyState',  self.JoyStateCallback,1)
        #self.sub_position = self.create_subscription(Position,"/Current_point",self.positionCallback,1)
        self.bridge = CvBridge()
        self.minDist = 1500
        self.Center_x = 0
        self.Center_y = 0
        self.Center_r = 0
        self.Center_prevx = 0
        self.Center_prevr = 0
        self.prev_time = 0
        self.prev_dist = 0
        self.prev_angular = 0
        self.Joy_active = False
        self.Robot_Run = False
        self.img_flip = False
        self.dist = []
        self.encoding = ['8UC3']
        self.linear_PID = (20.0, 0.0, 1.0)
        self.angular_PID = (0.5, 0.0, 2.0)
        self.scale = 1000
        self.end = 0
        self.PWMServo_X = 0
        self.PWMServo_Y = 0
        self.s1_init_angle = Int32()
        self.s1_init_angle.data = self.PWMServo_X
        self.s2_init_angle = Int32()
        self.s2_init_angle.data = self.PWMServo_Y
        self.PID_init()
        self.declare_param()
        self.last_stamp = None
        self.new_seconds = 0
        self.fps_seconds = 1




        #self.capture = cv.VideoCapture(0)
        #self.timer = self.create_timer(0.001, self.on_timer)

        #ESP32_wifi
        self.bridge = CvBridge()
        self.sub_img = self.create_subscription(
            CompressedImage, '/espRos/esp32camera', self.handleTopic, 1) #获取esp32传来的图像

        #确保角度正常处于中间
        for i in range(10):
            self.pub_Servo1.publish(self.s1_init_angle)
            self.pub_Servo2.publish(self.s2_init_angle)
            time.sleep(0.1)

        print("init done")

    def declare_param(self):
        #PID
        self.declare_parameter("Kp",20)
        self.Kp = self.get_parameter('Kp').get_parameter_value().integer_value
        self.declare_parameter("Ki",0)
        self.Ki = self.get_parameter('Ki').get_parameter_value().integer_value
        self.declare_parameter("Kd",0.9)
        self.Kd = self.get_parameter('Kd').get_parameter_value().integer_value

    def get_param(self):
        self.Kd = self.get_parameter('Kd').get_parameter_value().integer_value
        self.Ki = self.get_parameter('Ki').get_parameter_value().integer_value
        self.Kp = self.get_parameter('Kp').get_parameter_value().integer_value
        self.linear_PID = (self.Kp,self.Ki,self.Kd)

    # def on_timer(self):
    #     self.get_param()
    #     global m 
    #     m=0
    #     global n
    #     n=0

    #     ret, frame = self.capture.read()
    #     action = cv.waitKey(10) & 0xFF
    #     start = time.time()
    #     fps = 1 / (start - self.end)
    #     text = "FPS : " + str(int(fps))
    #     self.end = start
    #     cv.putText(frame, text, (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 1)
    #     face_patterns = cv2.CascadeClassifier('/root/yahboomcar_ws/src/yahboomcar_astra/yahboomcar_astra/haarcascade_frontalface_default.xml')
    #     faces = face_patterns.detectMultiScale(frame , scaleFactor=1.1, minNeighbors=5, minSize=(100, 100))
    #     if len(faces)>0:
    #         for (x, y, w, h) in faces:
    #             m=x
    #             n=y
    #             cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
    #         self.execute(m,n)
    #     cv.imshow('frame', frame)
    #     if action == ord('q') or action == 113:
    #         self.capture.release()
    #         cv.destroyAllWindows()


    #ESP32_wifi
    def handleTopic(self, msg):
        self.get_param()
        global m 
        m=0
        global n
        n=0

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
        face_patterns = cv2.CascadeClassifier('/home/yahboom/yahboomcar_ws/src/yahboom_esp32ai_car/yahboom_esp32ai_car/haarcascade_frontalface_default.xml')
        faces = face_patterns.detectMultiScale(frame , scaleFactor=1.1, minNeighbors=5, minSize=(100, 100))
        if len(faces)>0:
            for (x, y, w, h) in faces:
                m=x
                n=y
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            self.execute(m,n)
        
        end = time.time()
        fps = 1/((end - start)+self.fps_seconds) 
        
        text = "FPS : " + str(int(fps))
        cv2.putText(frame, text, (10,20), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,255), 2)
        cv2.imshow('frame', frame)

        if action == ord('q') or action == 113:
            cv2.destroyAllWindows()



    def PID_init(self):
        self.PID_controller = simplePID(
            [0, 0],
            [self.linear_PID[0] / float(self.scale), self.linear_PID[0] / float(self.scale)],
            [self.linear_PID[1] / float(self.scale), self.linear_PID[1] / float(self.scale)],
            [self.linear_PID[2] / float(self.scale), self.linear_PID[2] / float(self.scale)])
        self.pub_Servo1.publish(self.s1_init_angle)
        self.pub_Servo2.publish(self.s2_init_angle)
            
    def JoyStateCallback(self, msg):
        if not isinstance(msg, Bool): return
        self.Joy_active = msg.data
        self.pub_cmdVel.publish(Twist())

    def positionCallback(self, msg):
        if not isinstance(msg, Position): return
        self.Center_x = msg.anglex
        self.Center_y = msg.angley
        self.Center_r = msg.distance

    def execute(self, point_x, point_y):
        [x_Pid, y_Pid] = self.PID_controller.update([point_x - 320, point_y - 240])

        if abs(x_Pid) <1 and abs(y_Pid)<0.5:  #在死区直接不动了
            return

        if self.img_flip == True:
            self.PWMServo_X += x_Pid
            self.PWMServo_Y += y_Pid
        else:
            self.PWMServo_X  -= x_Pid
            self.PWMServo_Y  += y_Pid

        if self.PWMServo_X  >= 40:
            self.PWMServo_X  = 40
        elif self.PWMServo_X  <= -40:
            self.PWMServo_X  = -40
        if self.PWMServo_Y >= 40:
            self.PWMServo_Y = 40
        elif self.PWMServo_Y <= -90:
            self.PWMServo_Y = -90

        # rospy.loginfo("target_servox: {}, target_servoy: {}".format(self.target_servox, self.target_servoy))
        print("servo1",self.PWMServo_X)
        servo1_angle = Int32()
        servo1_angle.data = int(self.PWMServo_X)
        servo2_angle = Int32()
        servo2_angle.data = int(self.PWMServo_Y)
        self.pub_Servo1.publish(servo1_angle)
        self.pub_Servo2.publish(servo2_angle)



class simplePID:
    '''very simple discrete PID controller'''

    def __init__(self, target, P, I, D):
        '''Create a discrete PID controller
        each of the parameters may be a vector if they have the same length
        Args:
        target (double) -- the target value(s)
        P, I, D (double)-- the PID parameter
        '''
        # check if parameter shapes are compatabile.
        if (not (np.size(P) == np.size(I) == np.size(D)) or ((np.size(target) == 1) and np.size(P) != 1) or (
                np.size(target) != 1 and (np.size(P) != np.size(target) and (np.size(P) != 1)))):
            raise TypeError('input parameters shape is not compatable')
        #rospy.loginfo('P:{}, I:{}, D:{}'.format(P, I, D))
        self.Kp = np.array(P)
        self.Ki = np.array(I)
        self.Kd = np.array(D)
        self.last_error = 0
        self.integrator = 0
        self.timeOfLastCall = None
        self.setPoint = np.array(target)
        self.integrator_max = float('inf')

    def update(self, current_value):
        '''Updates the PID controller.
        Args:
            current_value (double): vector/number of same legth as the target given in the constructor
        Returns:
            controll signal (double): vector of same length as the target
        '''
        current_value = np.array(current_value)
        if (np.size(current_value) != np.size(self.setPoint)):
            raise TypeError('current_value and target do not have the same shape')
        if (self.timeOfLastCall is None):
            # the PID was called for the first time. we don't know the deltaT yet
            # no controll signal is applied
            self.timeOfLastCall = time.perf_counter()
            return np.zeros(np.size(current_value))
        error = self.setPoint - current_value
        P = error
        currentTime = time.perf_counter()
        deltaT = (currentTime - self.timeOfLastCall)
        # integral of the error is current error * time since last update
        self.integrator = self.integrator + (error * deltaT)
        I = self.integrator
        # derivative is difference in error / time since last update
        D = (error - self.last_error) / deltaT
        self.last_error = error
        self.timeOfLastCall = currentTime
        # return controll signal
        return self.Kp * P + self.Ki * I + self.Kd * D



def main():
    rclpy.init()
    face_Tracker = faceTracker("FaceTracker")
    try:
        rclpy.spin(face_Tracker)
    except KeyboardInterrupt:
        pass
    finally:
        face_Tracker.destroy_node()
        rclpy.shutdown()








