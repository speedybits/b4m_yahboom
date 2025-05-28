#ros lib
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import os
import sys
#commom lib
import math
import numpy as np
import time
from time import sleep
from yahboomcar_laser.common import *
print ("improt done")
RAD2DEG = 180 / math.pi

class laserTracker(Node):
    def __init__(self,name):
        super().__init__(name)
        #create a sub
        self.sub_laser = self.create_subscription(LaserScan,"/scan",self.registerScan,1)
        self.sub_JoyState = self.create_subscription(Bool,'/JoyState', self.JoyStateCallback,1)
        #create a pub
        self.pub_vel = self.create_publisher(Twist,'/cmd_vel',1)
        
        
        
        
        #declareparam
        self.declare_parameter("priorityAngle",10.0)
        self.priorityAngle = self.get_parameter('priorityAngle').get_parameter_value().double_value
        self.declare_parameter("LaserAngle",15.0)
        self.LaserAngle = self.get_parameter('LaserAngle').get_parameter_value().double_value
        self.declare_parameter("ResponseDist",0.55)
        self.ResponseDist = self.get_parameter('ResponseDist').get_parameter_value().double_value
        self.declare_parameter("Switch",False)
        self.Switch = self.get_parameter('Switch').get_parameter_value().bool_value
        
        self.Right_warning = 0
        self.Left_warning = 0
        self.front_warning = 0
        self.Joy_active = False
        self.ros_ctrl = SinglePID()
        self.Moving = False
        self.lin_pid = SinglePID(2.0, 0.0, 2.0)
        self.ang_pid = SinglePID(3.0, 0.0, 5.0)
        
        self.timer = self.create_timer(0.01,self.on_timer)
        
    def on_timer(self):
        self.Switch = self.get_parameter('Switch').get_parameter_value().bool_value
        self.priorityAngle = self.get_parameter('priorityAngle').get_parameter_value().double_value
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


    def registerScan(self, scan_data):
        if not isinstance(scan_data, LaserScan): return
        ranges = np.array(scan_data.ranges)
        offset = 0.5
        frontDistList = []
        frontDistIDList = []
        minDistList = []
        minDistIDList = []

        
        for i in range(len(ranges)):
            angle = (scan_data.angle_min + scan_data.angle_increment * i) * RAD2DEG
            if angle > 180: angle = angle - 360
            if abs(angle) < self.priorityAngle:
            	if 0 < ranges[i] < (self.ResponseDist + offset):
            		frontDistList.append(ranges[i])
            		frontDistIDList.append(angle)
            elif abs(angle) < self.LaserAngle and ranges[i] > 0:
            	minDistList.append(ranges[i])
            	minDistIDList.append(angle)
            
            	
        if len(frontDistIDList) != 0:
        	minDist = min(frontDistList)
        	minDistID = frontDistIDList[frontDistList.index(minDist)]
        else:
        	minDist = min(minDistList)
        	minDistID = minDistIDList[minDistList.index(minDist)]
        if self.Joy_active or self.Switch == True:
        	if self.Moving == True:
        	    self.pub_vel.publish(Twist())
        	    self.Moving = not self.Moving
        	return
        self.Moving = True
        velocity = Twist()
        print("minDist: ",minDist)
        if abs(minDist - self.ResponseDist) < 0.1: minDist = self.ResponseDist
        velocity.linear.x = -self.lin_pid.pid_compute(self.ResponseDist, minDist)
        ang_pid_compute = self.ang_pid.pid_compute(minDistID/48, 0)
        if minDistID > 0: velocity.angular.z = ang_pid_compute
        else: velocity.angular.z = ang_pid_compute
        velocity.angular.z = ang_pid_compute
        if abs(ang_pid_compute) < 0.1: velocity.angular.z = 0.0
        
        self.pub_vel.publish(velocity)

def main():
    rclpy.init()
    laser_tracker = laserTracker("laser_Tracker")
    print ("start it")
    try:
        rclpy.spin(laser_tracker)
    except KeyboardInterrupt:
        pass
    finally:
        laser_tracker.exit_pro()
        laser_tracker.destroy_node()
        rclpy.shutdown()
