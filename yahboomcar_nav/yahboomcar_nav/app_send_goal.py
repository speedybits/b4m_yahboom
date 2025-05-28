#!/usr/bin/env python
# coding:utf-8
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
from rclpy.clock import Clock
import os
import sys
from geometry_msgs.msg import PoseStamped

class Send_Goal(Node):
    def __init__(self,name):
        super().__init__(name)
        self.sub_goal = self.create_subscription(PoseStamped,"/goal_pose",self.TopicCallback,1)
        self.sub_goal = self.create_subscription(PoseStamped,"/goal_pose",self.TopicCallback,1)
        self.pub_goal = self.create_publisher(PoseStamped,"/goal_pose",1)
        
    def TopicCallback(self, msg):
        if not isinstance(msg, PoseStamped): return
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.pose.position.x = msg.pose.position.x
        goal.pose.position.y = msg.pose.position.y
        goal.pose.position.z = msg.pose.position.z
        goal.pose.orientation.x = msg.pose.orientation.x
        goal.pose.orientation.y = msg.pose.orientation.y
        goal.pose.orientation.z = msg.pose.orientation.z
        goal.pose.orientation.w = msg.pose.orientation.w
        self.pub_goal.publish(goal)
        
        '''print("frame_id : ",msg.header.frame_id)
        print("px : ",msg.pose.position.x)'''
        

        

def main():
    rclpy.init()
    send_goal = Send_Goal("AppSendGaol_Node")
    rclpy.spin(send_goal)
