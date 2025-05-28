#ros lib
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool,UInt16
#commom lib
import os
import sys
import math
import numpy as np
import time
from time import sleep
from yahboomcar_laser.common import *
print ("improt done")
RAD2DEG = 180 / math.pi

class laserWarning(Node):
    def __init__(self,name):
        super().__init__(name)
        #create a sub
        self.sub_laser = self.create_subscription(LaserScan,"/scan",self.registerScan,1)
        self.sub_JoyState = self.create_subscription(Bool,'/JoyState', self.JoyStateCallback,1)
        self.sub_JoyState = self.create_subscription(Bool,'/JoyState', self.JoyStateCallback,1)
        #create a pub
        self.pub_vel = self.create_publisher(Twist,'/cmd_vel',1)
        self.pub_Buzzer = self.create_publisher(UInt16,'/beep',1)
        
        
        
        
        #declareparam
        self.declare_parameter("LaserAngle",10.0)
        self.LaserAngle = self.get_parameter('LaserAngle').get_parameter_value().double_value
        self.declare_parameter("ResponseDist",0.3)
        self.ResponseDist = self.get_parameter('ResponseDist').get_parameter_value().double_value
        self.declare_parameter("Switch",False)
        self.Switch = self.get_parameter('Switch').get_parameter_value().bool_value
        
        self.Right_warning = 0
        self.Left_warning = 0
        self.front_warning = 0
        self.Joy_active = False
        self.ros_ctrl = SinglePID()
        self.ang_pid = SinglePID(3.0, 0.0, 5.0)
        self.Buzzer_state = False
        self.Moving = False
        
        self.timer = self.create_timer(0.01,self.on_timer)
        
    def on_timer(self):
        self.Switch = self.get_parameter('Switch').get_parameter_value().bool_value
        self.LaserAngle = self.get_parameter('LaserAngle').get_parameter_value().double_value
        self.ResponseDist = self.get_parameter('ResponseDist').get_parameter_value().double_value

    def JoyStateCallback(self, msg):
        if not isinstance(msg, Bool): return
        self.Joy_active = msg.data
        
    def exit_pro(self):
        cmd1 = "ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "
        cmd2 = '''"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"'''
        cmd = cmd1 +cmd2
        os.system(cmd)
        cmd3 = "ros2 topic pub --once /beep std_msgs/msg/UInt16 "
        cmd4 = '''"data: 0"'''
        cmd = cmd1 +cmd2
        os.system(cmd)

    def registerScan(self, scan_data):
        if not isinstance(scan_data, LaserScan): return
        if self.Joy_active  or self.Switch == True:
            if self.Moving == True:
                print("stop")
                self.pub_vel.publish(Twist())
                self.Moving = not self.Moving
            return
        self.Moving = True
        ranges = np.array(scan_data.ranges)
        minDistList = []
        minDistIDList = []
        
        for i in range(len(ranges)):
            angle = (scan_data.angle_min + scan_data.angle_increment * i) * RAD2DEG
            if angle > 180: angle = angle - 360
            if  abs(angle) < self.LaserAngle and ranges[i] > 0:
            	minDistList.append(ranges[i])
            	minDistIDList.append(angle)
        if len(minDistList) != 0: 
        	minDist = min(minDistList)
        	minDistID = minDistIDList[minDistList.index(minDist)]
        else:
        	return
        	
        velocity = Twist()
        angle_pid_compute = self.ang_pid.pid_compute(minDistID/48, 0)
        if abs(angle_pid_compute) < 0.1:
            velocity.angular.z = 0.0
        else:
            velocity.angular.z = angle_pid_compute
        self.pub_vel.publish(velocity)
        

        print("minDist: ",minDist)
        print("minDistID: ",minDistID)
        if minDist <= self.ResponseDist:
        	print("---------------")
        	b = UInt16()
        	b.data = 1
        	self.pub_Buzzer.publish(b)

        else:
        	print("no obstacles@")
        	b = UInt16()
        	b.data = 0
        	self.pub_Buzzer.publish(UInt16())
        	
        

def main():
    rclpy.init()
    laser_warn = laserWarning("laser_Warnning")
    print ("start it")
    try:
        rclpy.spin(laser_warn)
    except KeyboardInterrupt:
        pass
    finally:
        laser_warn.exit_pro()
        laser_warn.destroy_node()
        rclpy.shutdown()
