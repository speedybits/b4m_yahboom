#ros lib
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage, LaserScan, Image
from yahboomcar_msgs.msg import Position
from std_msgs.msg import Int32, Bool,UInt16
from cv_bridge import CvBridge
#common lib
import os
import threading
import math
import time
from yahboom_esp32ai_car.astra_common import *
from yahboomcar_msgs.msg import Position

print("import done")

class color_Tracker(Node):
    def __init__(self,name):
        super().__init__(name)
        #create the publisher
        self.pub_cmdVel = self.create_publisher(Twist,'/cmd_vel',10)
        self.pub_Servo1 = self.create_publisher(Int32,"servo_s1" , 10)
        self.pub_Servo2 = self.create_publisher(Int32,"servo_s2" , 10)
        #create the subscriber
        self.sub_depth = self.create_subscription(Image,"/image_raw", self.depth_img_Callback, 1)
        self.sub_JoyState = self.create_subscription(Bool,'/JoyState',  self.JoyStateCallback,1)
        self.sub_position = self.create_subscription(Position,"/Current_point",self.positionCallback,1)
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
        self.linear_PID = (8.0, 0.0, 1.0)
        self.angular_PID = (0.5, 0.0, 2.0)
        self.scale = 1000
        self.PID_init()
        self.PWMServo_X = 0
        self.PWMServo_Y = 10
        self.s1_init_angle = Int32()
        self.s1_init_angle.data = self.PWMServo_X
        self.s2_init_angle = Int32()
        self.s2_init_angle.data = self.PWMServo_Y
        self.declare_param()

        #确保角度正常处于中间
        for i in range(10):
            self.pub_Servo1.publish(self.s1_init_angle)
            self.pub_Servo2.publish(self.s2_init_angle)
            time.sleep(0.1)
            
        print("init done")
        
    def declare_param(self):
        self.declare_parameter("linear_Kp",20.0)
        self.linear_Kp = self.get_parameter('linear_Kp').get_parameter_value().double_value
        self.declare_parameter("linear_Ki",0.0)
        self.linear_Ki = self.get_parameter('linear_Ki').get_parameter_value().double_value
        self.declare_parameter("linear_Kd",2.0)
        self.linear_Kd = self.get_parameter('linear_Kd').get_parameter_value().double_value
        self.declare_parameter("angular_Kp",0.5)
        self.angular_Kp = self.get_parameter('angular_Kp').get_parameter_value().double_value
        self.declare_parameter("angular_Ki",0.0)
        self.angular_Ki = self.get_parameter('angular_Ki').get_parameter_value().double_value
        self.declare_parameter("angular_Kd",2.0)
        self.angular_Kd = self.get_parameter('angular_Kd').get_parameter_value().double_value
        self.declare_parameter("scale",1000)
        self.scale = self.get_parameter('scale').get_parameter_value().integer_value
        self.declare_parameter("minDistance",1.0)
        self.minDistance = self.get_parameter('minDistance').get_parameter_value().double_value
        
        self.declare_parameter('angular_speed_limit',5.0)
        self.angular_speed_limit = self.get_parameter('angular_speed_limit').get_parameter_value().double_value
        self.pub_Servo1.publish(self.s1_init_angle)
        self.pub_Servo2.publish(self.s2_init_angle)

    def get_param(self):
        self.linear_Kp = self.get_parameter('linear_Kp').get_parameter_value().double_value
        self.linear_Ki = self.get_parameter('linear_Ki').get_parameter_value().double_value
        self.linear_Kd = self.get_parameter('linear_Kd').get_parameter_value().double_value
        self.angular_Kp = self.get_parameter('angular_Kp').get_parameter_value().double_value
        self.angular_Ki = self.get_parameter('angular_Ki').get_parameter_value().double_value
        self.angular_Kd = self.get_parameter('angular_Kd').get_parameter_value().double_value
        self.scale = self.get_parameter('scale').get_parameter_value().integer_value
        self.minDistance = self.get_parameter('minDistance').get_parameter_value().double_value
        
        self.linear_PID = (self.linear_Kp, self.linear_Ki, self.linear_Kd)
        self.angular_PID = (self.angular_Kp, self.angular_Ki, self.angular_Kd)
        self.minDist = self.minDistance * 1000
        

    def PID_init(self):
       # self.linear_pid = simplePID(self.linear_PID[0] / 1000.0, self.linear_PID[1] / 1000.0, self.linear_PID[2] / 1000.0)
        #self.angular_pid = simplePID(self.angular_PID[0] / 100.0, self.angular_PID[1] / 100.0, self.angular_PID[2] / 100.0)
        self.linear_pid = simplePID(
            [0, 0],
            [self.linear_PID[0]  / float(self.scale), self.linear_PID[0]  / float(self.scale)],
            [self.linear_PID[1]  / float(self.scale), self.linear_PID[1]  / float(self.scale)],
            [self.linear_PID[2]  / float(self.scale), self.linear_PID[2]  / float(self.scale)])
    def depth_img_Callback(self, msg):
        if not isinstance(msg, Image): return
        depthFrame = self.bridge.imgmsg_to_cv2(msg, desired_encoding=self.encoding[0])
        self.action = cv.waitKey(1)
        if self.Center_r != 0:
            now_time = time.time()
            if now_time - self.prev_time > 5:
                if self.Center_prevx == self.Center_x and self.Center_prevr == self.Center_r: self.Center_r = 0
                self.prev_time = now_time
            distance = [0, 0, 0, 0, 0]
            if 0 < int(self.Center_y - 3) and int(self.Center_y + 3) < 480 and 0 < int(
                self.Center_x - 3) and int(self.Center_x + 3) < 640:
                # print("depthFrame: ", len(depthFrame), len(depthFrame[0]))
                distance[0] = depthFrame[int(self.Center_y - 3)][int(self.Center_x - 3)]
                distance[1] = depthFrame[int(self.Center_y + 3)][int(self.Center_x - 3)]
                distance[2] = depthFrame[int(self.Center_y - 3)][int(self.Center_x + 3)]
                distance[3] = depthFrame[int(self.Center_y + 3)][int(self.Center_x + 3)]
                distance[4] = depthFrame[int(self.Center_y)][int(self.Center_x)]
                distance_ = 1000.0
                num_depth_points = 5
                for i in range(5):
                    if 40 < distance[i].all() < 80000: distance_ += distance[i]
                    else: num_depth_points -= 1
                if num_depth_points == 0: distance_ = self.minDist
                else: distance_ /= num_depth_points
                #print("Center_x: {}, Center_y: {}, distance_: {}".format(self.Center_x, self.Center_y, distance_))
                self.execute(self.Center_x, self.Center_y)
                self.Center_prevx = self.Center_x
                self.Center_prevr = self.Center_r
        else:
            if self.Robot_Run ==True:
                self.pub_cmdVel.publish(Twist())
                self.Robot_Run = False
        if self.action == ord('q') or self.action == 113: self.cleanup()
        
    def JoyStateCallback(self, msg):
        if not isinstance(msg, Bool): return
        self.Joy_active = msg.data
        self.pub_cmdVel.publish(Twist())
        
    def positionCallback(self, msg):
        if not isinstance(msg, Position): return
        self.Center_x = msg.anglex
        self.Center_y = msg.angley
        self.Center_r = msg.distance
        
    def cleanup(self):
        self.pub_cmdVel.publish(Twist())
        print ("Shutting down this node.")
        cv.destroyAllWindows()
        
    def execute(self, point_x, point_y):
        [x_Pid, y_Pid] = self.linear_pid .update([point_x - 320, point_y - 240])
        if self.img_flip == True:
            self.PWMServo_X += x_Pid
            self.PWMServo_Y += y_Pid
        else:
            self.PWMServo_X  -= x_Pid
            self.PWMServo_Y  += y_Pid

        if self.PWMServo_X  >= 45:
            self.PWMServo_X  = 45
        elif self.PWMServo_X  <= -45:
            self.PWMServo_X  = -45
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
       # rospy.loginfo('P:{}, I:{}, D:{}'.format(P, I, D))
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
    color_tracker = color_Tracker("ColorTracker")
    print("start it")
    rclpy.spin(color_tracker)
