#for patrol
#math
from math import radians, copysign, sqrt, pow
from math import pi
import numpy as np
#rclpy
import rclpy
from rclpy.node import Node
#tf
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
#msg
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
#others
import PyKDL

from yahboomcar_multi.singlePID import *

from time import sleep

print("import finish")
RAD2DEG = 180 / pi

class ListenTF(Node):
    def __init__(self,name):
        super().__init__(name)
        self.min_dist = 0.5
        self.minX = 0.0
        self.minY = 0.0
        self.source_frame = "point1"
        self.robot_frame = "robot2/base_footprint"
        self.start_status = True
        self.lin_pid = SinglePID()
        self.ang_pid = SinglePID()
        self.linPIDparam = [1.0, 0, 1.0]
        self.angPIDparam = [0.8, 0, 1.0]
        self.lin_pid.Set_pid(self.linPIDparam[0], self.linPIDparam[1], self.linPIDparam[2])
        self.ang_pid.Set_pid(self.angPIDparam[0], self.angPIDparam[1], self.angPIDparam[2])
        #create publisher
        self.pub_cmdVel = self.create_publisher(Twist,"/robot2/cmd_vel",5)
        #create subscriber
        self.sub_joy = self.create_subscription(Bool,"/JoyState",self.JoyStateCallback,1)
        #create TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer,self)
        #declare param
        self.declare_parameter('Switch',True)
        self.Switch = self.get_parameter('Switch').get_parameter_value().bool_value
        #create timer
        self.timer = self.create_timer(0.1,self.on_timer)
        self.index = 0
        
        
    def on_timer(self):
        self.Switch = self.get_parameter('Switch').get_parameter_value().bool_value
        
        if self.Switch == True:
            x = self.listen_tf_xy().transform.translation.x
            y = self.listen_tf_xy().transform.translation.y
            theta = self.listen_tf_z()
            distance = sqrt(pow(x,2)+pow(x,2))
            print("distance: ",distance)
            if self.start_status:
                if abs(self.min_dist - distance)<0.01 and abs(self.minX - x)<0.1 and abs(self.minY -y)<0.1:
                    self.start_status = False
                else:
                    self.min_dist = distance
                    self.minX = x
                    self.minY = y
            else:
                move_cmd = Twist()
                move_cmd.linear.x = self.lin_pid.pid_compute(x,0.0)/4.0
                move_cmd.linear.y = self.lin_pid.pid_compute(y,0.0)/4.0
                move_cmd.angular.z = self.lin_pid.pid_compute(np.interp(theta*RAD2DEG,[-180,180],[-5,5]),0.0)/4.0
                self.pub_cmdVel.publish(move_cmd)
            
            
    def listen_tf_xy(self):
        try:
            now = rclpy.time.Time()
            #now = self.get_clock().now().to_msg()
            trans = self.tf_buffer.lookup_transform(self.source_frame,self.robot_frame,now)
            return trans
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().info('transform not ready')
            raise
            return
            
    def listen_tf_z(self):
        try:
            now = rclpy.time.Time()
            #now = self.get_clock().now().to_msg()
            rot = self.tf_buffer.lookup_transform(self.robot_frame,self.source_frame,now)
            cacl_rot = PyKDL.Rotation.Quaternion(rot.transform.rotation.x, rot.transform.rotation.y, rot.transform.rotation.z, rot.transform.rotation.w)
            angle_rot = cacl_rot.GetRPY()[2]
            
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().info('transform not ready')
            return
        return angle_rot       
            
    def normalize_angle(self,angle):
        res = angle
        #print("res: ",res)
        while res > pi:
            res -= 2.0 * pi
        while res < -pi:
            res += 2.0 * pi
        return res
    
    def JoyStateCallback(self, msg):
        if not isinstance(msg, Bool): return
        self.Joy_active = msg.data
 
    
def main():
    rclpy.init()
    listen_robot1 = ListenTF("listen_robot1")
    print("create done")
    rclpy.spin(listen_robot1)
    
