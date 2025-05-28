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
from tf2_ros import TransformBroadcaster
#msg
from geometry_msgs.msg import Twist, Point,TransformStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool,String
#others
import PyKDL
from time import sleep

print("import finish")

class Queue_Broadcaster(Node):
    def __init__(self,name):
        super().__init__(name)
        #create TFBroadcaster
        self.tf_br = TransformBroadcaster(self)
        #declare param
        self.declare_parameter('Switch',False)
        self.Switch = self.get_parameter('Switch').get_parameter_value().bool_value
        self.declare_parameter('Team',"vertical")
        self.Team = self.get_parameter('Team').get_parameter_value().string_value
        self.declare_parameter('Distance',0.5)
        self.Distance = self.get_parameter('Distance').get_parameter_value().double_value
        self.robot_name = 'robot1/base_footprint'
        '''self.declare_parameter('Command',"Square")
        self.Command = self.get_parameter('Command').get_parameter_value().string_value
        self.command_src = self.Command
        self.declare_parameter('Length',1.0)
        self.Length = self.get_parameter('Length').get_parameter_value().double_value
        self.declare_parameter('Linear',0.2)
        self.Linear = self.get_parameter('Linear').get_parameter_value().double_value
        self.declare_parameter('Angular',2.0)
        self.Angular = self.get_parameter('Angular').get_parameter_value().double_value
        self.declare_parameter('ResponseDist',0.6)
        self.ResponseDist = self.get_parameter('ResponseDist').get_parameter_value().double_value
        self.declare_parameter('LaserAngle',20.0)
        self.LaserAngle = self.get_parameter('LaserAngle').get_parameter_value().double_value
        self.declare_parameter('RotationScaling',1.25)
        self.RotationScaling = self.get_parameter('RotationScaling').get_parameter_value().double_value
        self.declare_parameter('RotationTolerance',0.3)
        self.RotationTolerance = self.get_parameter('RotationTolerance').get_parameter_value().double_value'''
        #create sub
        self.sub_scan = self.create_subscription(String,"/team",self.Callback,1)
        #create timer
        #self.timer = self.create_timer(0.1,self.on_timer)
        self.index = 0
        


    def Callback(self,msg):
        self.Team = msg.data
        print("Team: ",self.Team)
        if self.Team == 'vertical':
            x1,y1,x2,y2 = -self.Distance,0.0,-self.Distance*2,0.0
        elif self.Team == 'horizontal':
            x1,y1,x2,y2 =0,self.Distance,0.0,-self.Distance
        elif self.Team == 'convoyl':
            x1,y1,x2,y2 == -self.Distance,self.Distance,-self.Distance,-self.Distance
        #point1
        t1 = TransformStamped()
        t1.header.stamp = self.get_clock().now().to_msg()
        t1.header.frame_id = self.robot_name
        t1.child_frame_id = "point1"
        t1.transform.translation.x = x1
        t1.transform.translation.y = y1
        self.tf_br.sendTransform(t1)
        
        #point2
        t2 = TransformStamped()
        t2.header.stamp = self.get_clock().now().to_msg()
        t2.header.frame_id = self.robot_name
        t2.child_frame_id = "point2"
        t2.transform.translation.x = x2
        t2.transform.translation.y = y2
        self.tf_br.sendTransform(t2)

    
def main():
    rclpy.init()
    queue_br = Queue_Broadcaster("queue")
    print("create done")
    rclpy.spin(queue_br)
    
