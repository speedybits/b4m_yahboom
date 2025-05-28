#!/usr/bin/env python3
# encoding: utf-8
import getpass
import threading
from yahboom_esp32ai_car.astra_common import *
from sensor_msgs.msg import CompressedImage,Image
from std_msgs.msg import Int32, Bool,UInt16
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge

from rclpy.time import Time
import datetime


class mono_Tracker(Node):
    def __init__(self,name):
        super().__init__(name)
        self.pub_Servo1 = self.create_publisher(Int32,"servo_s1" , 10)
        self.pub_Servo2 = self.create_publisher(Int32,"servo_s2" , 10) 
        self.declare_param()
        self.target_servox = 0
        self.target_servoy = 10
        self.point_pose = (0, 0, 0)
        self.circle = (0, 0, 0)
        self.hsv_range = ()
        self.dyn_update = True
        self.select_flags = False
        self.gTracker_state = False
        self.windows_name = 'frame'
        self.cols, self.rows = 0, 0
        self.Mouse_XY = (0, 0)
        self.index = 2
        self.end = 0
        self.color = color_follow()
    
        self.tracker_types = ['BOOSTING', 'MIL', 'KCF']
        self.tracker_type = ['KCF'] 
        self.VideoSwitch = True
        self.img_flip = False

        self.last_stamp = None
        self.new_seconds = 0
        self.fps_seconds = 1

        ser1_angle = Int32()
        ser1_angle.data = int(self.target_servox)
        ser2_angle = Int32()
        ser2_angle.data = int(self.target_servoy)

        #确保角度正常处于中间
        for i in range(10):
            self.pub_Servo1.publish(ser1_angle)
            self.pub_Servo2.publish(ser2_angle)
            time.sleep(0.1)
        


        self.hsv_text ="/home/yahboom/yahboomcar_ws/src/yahboom_esp32ai_car/yahboom_esp32ai_car/colorHSV.text"
        self.mono_PID = (12, 0, 0.9)
        self.scale = 1000
        self.PID_init()

        print("OpenCV Version: ",cv.__version__)
        self.gTracker = Tracker(tracker_type=self.tracker_type)
        self.tracker_type = self.tracker_types[self.index]
        self.Track_state = 'init'

        #USB
        #self.capture = cv.VideoCapture(0)
        #self.timer = self.create_timer(0.001, self.on_timer)

        #ESP32_wifi
        self.bridge = CvBridge()
        self.sub_img = self.create_subscription(
            CompressedImage, '/espRos/esp32camera', self.handleTopic, 1) #获取esp32传来的图像


    def declare_param(self):
        #PID
        self.declare_parameter("Kp",12)
        self.Kp = self.get_parameter('Kp').get_parameter_value().integer_value
        self.declare_parameter("Ki",0)
        self.Ki = self.get_parameter('Ki').get_parameter_value().integer_value
        self.declare_parameter("Kd",0.9)
        self.Kd = self.get_parameter('Kd').get_parameter_value().integer_value
    
    def get_param(self):
        self.Kd = self.get_parameter('Kd').get_parameter_value().integer_value
        self.Ki = self.get_parameter('Ki').get_parameter_value().integer_value
        self.Kp = self.get_parameter('Kp').get_parameter_value().integer_value
        self.mono_PID = (self.Kp,self.Ki,self.Kd)
        

    def cancel(self):     
        self.Reset()
        if self.VideoSwitch==False: self.__sub_img.unregister()
        cv.destroyAllWindows()

    # USB
    # def on_timer(self):
    #     self.get_param()
    #     ret, frame = self.capture.read()
    #     action = cv.waitKey(10) & 0xFF
    #     frame, binary =self.process(frame, action)
    #     start = time.time()
    #     fps = 1 / (start - self.end)
    #     text = "FPS : " + str(int(fps))
    #     self.end = start
    #     cv.putText(frame, text, (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 1)
    #     if len(binary) != 0: cv.imshow('frame', ManyImgs(1, ([frame, binary])))
    #     else:cv.imshow('frame', frame)
    #     if action == ord('q') or action == 113:
    #         self.capture.release()
    #         cv.destroyAllWindows()

    #ESP32_wifi
    def handleTopic(self, msg):
        self.last_stamp = msg.header.stamp  
        if self.last_stamp:
            total_secs = Time(nanoseconds=self.last_stamp.nanosec, seconds=self.last_stamp.sec).nanoseconds
            delta = datetime.timedelta(seconds=total_secs * 1e-9)
            seconds = delta.total_seconds()*100

            if self.new_seconds != 0:
                self.fps_seconds = seconds - self.new_seconds

            self.new_seconds = seconds#保留这次的值

        self.get_param()
        start = time.time()
        frame = self.bridge.compressed_imgmsg_to_cv2(msg)
        frame = cv.resize(frame, (640, 480))

        action = cv.waitKey(10) & 0xFF
        frame, binary =self.process(frame, action)
        
        
        end = time.time()
        fps = 1 / ((end - start)+self.fps_seconds)
        
        text = "FPS : " + str(int(fps))
        cv.putText(frame, text, (10,20), cv.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,255), 2)
        cv.imshow('frame', frame)

        if action == ord('q') or action == 113:
            cv.destroyAllWindows()




    def Reset(self):
        self.hsv_range = ()
        self.circle = (0, 0, 0)
        self.Mouse_XY = (0, 0)
        self.Track_state = 'init'
        self.target_servox = 0
        self.target_servoy = 10
    

    def execute(self, point_x, point_y):
        # rospy.loginfo("point_x: {}, point_y: {}".format(point_x, point_y))
        [x_Pid, y_Pid] = self.PID_controller.update([point_x - 320, point_y - 240])
        if self.img_flip == True:
            self.target_servox -= x_Pid
            self.target_servoy += y_Pid
        else:
            self.target_servox -= x_Pid
            self.target_servoy += y_Pid
        if self.target_servox >= 45:
            self.target_servox = 45
        elif self.target_servox <= -45:
            self.target_servox = -45
        if self.target_servoy >= 40:
            self.target_servoy = 40
        elif self.target_servoy <= -90:
            self.target_servoy = -90
        print("servo1",self.target_servox)
        servo1_angle = Int32()
        servo1_angle.data = int(self.target_servox)
        servo2_angle = Int32()
        servo2_angle.data = int(self.target_servoy)
        self.pub_Servo1.publish(servo1_angle)
        self.pub_Servo2.publish(servo2_angle)

    def dynamic_reconfigure_callback(self, config, level):
        self.scale = config['scale']
        self.mono_PID = (config['Kp'], config['Ki'], config['Kd'])
        self.hsv_range = ((config['Hmin'], config['Smin'], config['Vmin']),
                          (config['Hmax'], config['Smax'], config['Vmax']))
        self.PID_init()
        return config

    def PID_init(self):
        self.PID_controller = simplePID(
            [0, 0],
            [self.mono_PID[0] / float(self.scale), self.mono_PID[0] / float(self.scale)],
            [self.mono_PID[1] / float(self.scale), self.mono_PID[1] / float(self.scale)],
            [self.mono_PID[2] / float(self.scale), self.mono_PID[2] / float(self.scale)])

    def onMouse(self, event, x, y, flags, param):
        if event == 1:
            self.Track_state = 'init'
            self.select_flags = True
            self.Mouse_XY = (x,y)
        if event == 4:
            self.select_flags = False
            self.Track_state = 'identify'
        if self.select_flags == True:
            self.cols = min(self.Mouse_XY[0], x), min(self.Mouse_XY[1], y)
            self.rows = max(self.Mouse_XY[0], x), max(self.Mouse_XY[1], y)
            self.Roi_init = (self.cols[0], self.cols[1], self.rows[0], self.rows[1])

    def process(self, rgb_img, action):
        # param action: [113 or 'q':退出]，[114 or 'r':重置]，[105 or 'i'：识别]，[32：开始追踪]
        rgb_img = cv.resize(rgb_img, (640, 480))
        binary = []
        if self.img_flip == True: rgb_img = cv.flip(rgb_img, 1)
        if action == 32: self.Track_state = 'tracking'
        elif action == ord('i') or action == 105: self.Track_state = "identify"
        elif action == ord('r') or action == 114: self.Reset()
        elif action == ord('q') or action == 113: self.cancel()
        if self.Track_state == 'init':
            cv.namedWindow(self.windows_name, cv.WINDOW_AUTOSIZE)
            cv.setMouseCallback(self.windows_name, self.onMouse, 0)
            if self.select_flags == True:
                cv.line(rgb_img, self.cols, self.rows, (255, 0, 0), 2)
                cv.rectangle(rgb_img, self.cols, self.rows, (0, 255, 0), 2)
                if self.Roi_init[0] != self.Roi_init[2] and self.Roi_init[1] != self.Roi_init[3]:
                    if self.tracker_type == "color": rgb_img, self.hsv_range = self.color.Roi_hsv(rgb_img, self.Roi_init)
                    self.gTracker_state = True
                    self.dyn_update = True
                else: self.Track_state = 'init'
        if self.Track_state != 'init':
            if self.tracker_type == "color" and len(self.hsv_range) != 0:
                rgb_img, binary, self.circle = self.color.object_follow(rgb_img, self.hsv_range)
                if self.dyn_update == True:
                    params = {'Hmin': self.hsv_range[0][0], 'Hmax': self.hsv_range[1][0],
                              'Smin': self.hsv_range[0][1], 'Smax': self.hsv_range[1][1],
                              'Vmin': self.hsv_range[0][2], 'Vmax': self.hsv_range[1][2]}
                    self.dyn_client.update_configuration(params)
                    self.dyn_update = False
            if self.tracker_type != "color":
                if self.gTracker_state == True:
                    Roi = (self.Roi_init[0], self.Roi_init[1], self.Roi_init[2] - self.Roi_init[0], self.Roi_init[3] - self.Roi_init[1])
                    self.gTracker = Tracker(tracker_type=self.tracker_type)
                    self.gTracker.initWorking(rgb_img, Roi)
                    self.gTracker_state = False
                rgb_img, (targBegin_x, targBegin_y), (targEnd_x, targEnd_y) = self.gTracker.track(rgb_img)
                center_x = targEnd_x / 2 + targBegin_x / 2
                center_y = targEnd_y / 2 + targBegin_y / 2
                width = targEnd_x - targBegin_x
                high = targEnd_y - targBegin_y
                self.point_pose = (center_x, center_y, min(width, high))
        if self.Track_state == 'tracking':
            if self.circle[2] != 0: threading.Thread(target=self.execute, args=(self.circle[0], self.circle[1])).start()
            if self.point_pose[0] != 0 and self.point_pose[1] != 0: threading.Thread(target=self.execute, args=(self.point_pose[0], self.point_pose[1])).start()
        if self.tracker_type != "color": cv.putText(rgb_img, " Tracker", (260, 20), cv.FONT_HERSHEY_SIMPLEX, 0.75, (50, 170, 50), 2)
        return rgb_img, binary

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
    mono_tracker = mono_Tracker("monoIdentify")
    try:
        rclpy.spin(mono_tracker)
    except KeyboardInterrupt:
        pass
    finally:
        mono_tracker.destroy_node()
        rclpy.shutdown()
    
    

