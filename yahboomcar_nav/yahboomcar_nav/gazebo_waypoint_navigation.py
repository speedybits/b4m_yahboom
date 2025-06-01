#!/usr/bin/env python3
# coding:utf-8
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import threading
import numpy as np
import os
import sys
import math
import time
import termios
import tty
import select
import tf2_ros
from tf2_ros import TransformException
from tf2_geometry_msgs import do_transform_pose

class GazeboWaypointNavigation(Node):
    def __init__(self, name):
        super().__init__(name)
        
        # Declare parameters
        self.declare_parameter('use_sim_time', True)
        
        # Create callback group for the action client
        self.callback_group = ReentrantCallbackGroup()
        
        # Create publisher for cmd_vel to stop the robot when needed
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Create action client for navigation
        self.nav_to_pose_client = ActionClient(
            self, 
            NavigateToPose, 
            'navigate_to_pose',
            callback_group=self.callback_group
        )
        
        # Setup TF2 listener for getting robot position
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Wait for the action server to be available
        self.get_logger().info('Waiting for navigation action server...')
        self.nav_to_pose_client.wait_for_server()
        self.get_logger().info('Navigation action server connected!')
        
        # Initialize waypoints dictionary (name -> pose)
        self.waypoints = {}
        
        # Current navigation goal handle
        self.current_goal_handle = None
        
        # Flag to track if navigation is in progress
        self.navigating = False
        
        # Create a timer for keyboard input checking
        self.keyboard_timer = self.create_timer(0.1, self.check_keyboard_input)
        
        # Print instructions
        self.print_instructions()
    
    def print_instructions(self):
        """Print the instructions for using the waypoint navigation system."""
        self.get_logger().info("\n" + "="*50)
        self.get_logger().info("GAZEBO WAYPOINT NAVIGATION SYSTEM")
        self.get_logger().info("="*50)
        self.get_logger().info("Commands:")
        self.get_logger().info("  s - Save current position as a waypoint")
        self.get_logger().info("  l - List all saved waypoints")
        self.get_logger().info("  g - Go to a waypoint (you'll be prompted for the number)")
        self.get_logger().info("  c - Cancel current navigation")
        self.get_logger().info("  q - Quit the program")
        self.get_logger().info("="*50)
    
    def check_keyboard_input(self):
        """Check for keyboard input and process commands."""
        if select.select([sys.stdin], [], [], 0)[0]:
            key = sys.stdin.read(1)
            
            if key == 's':
                self.save_current_position()
            elif key == 'l':
                self.list_waypoints()
            elif key == 'g':
                self.select_and_navigate_to_waypoint()
            elif key == 'c':
                self.cancel_navigation()
            elif key == 'q':
                self.quit_program()
    
    def get_current_position(self):
        """
        Get the current position of the robot using TF2.
        """
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        
        try:
            # Get the transform from map to base_link
            transform = self.tf_buffer.lookup_transform(
                "map",
                "base_link",
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=1.0)
            )
            
            # Set the position from the transform
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            
            # Set the orientation from the transform
            pose.pose.orientation.x = transform.transform.rotation.x
            pose.pose.orientation.y = transform.transform.rotation.y
            pose.pose.orientation.z = transform.transform.rotation.z
            pose.pose.orientation.w = transform.transform.rotation.w
            
            return pose
            
        except TransformException as ex:
            self.get_logger().error(f"Could not get transform: {ex}")
            
            # Return a default pose if transform fails
            pose.pose.position.x = 0.0
            pose.pose.position.y = 0.0
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            return pose
    
    def save_current_position(self):
        """Save the current position as a waypoint."""
        # Get the current position
        current_pose = self.get_current_position()
        
        # Ask for a name for this waypoint
        self.get_logger().info("Enter a name for this waypoint: ")
        waypoint_name = input().strip()
        
        if not waypoint_name:
            self.get_logger().warn("Waypoint name cannot be empty. Waypoint not saved.")
            return
        
        # Save the waypoint
        self.waypoints[waypoint_name] = current_pose
        self.get_logger().info(f"Waypoint '{waypoint_name}' saved at position: "
                              f"({current_pose.pose.position.x:.2f}, {current_pose.pose.position.y:.2f})")
    
    def list_waypoints(self):
        """List all saved waypoints."""
        if not self.waypoints:
            self.get_logger().info("No waypoints saved yet.")
            return
        
        self.get_logger().info("\nSaved Waypoints:")
        for i, (name, pose) in enumerate(self.waypoints.items(), 1):
            self.get_logger().info(f"{i}. {name}: ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})")
    
    def select_and_navigate_to_waypoint(self):
        """Prompt the user to select a waypoint and navigate to it."""
        if not self.waypoints:
            self.get_logger().info("No waypoints saved yet. Save some waypoints first.")
            return
        
        # List the waypoints
        self.list_waypoints()
        
        # Ask the user to select a waypoint
        self.get_logger().info("Enter the number of the waypoint to navigate to: ")
        try:
            selection = int(input().strip())
            if selection < 1 or selection > len(self.waypoints):
                self.get_logger().warn(f"Invalid selection. Please enter a number between 1 and {len(self.waypoints)}.")
                return
            
            # Get the selected waypoint
            waypoint_name = list(self.waypoints.keys())[selection - 1]
            waypoint_pose = self.waypoints[waypoint_name]
            
            # Navigate to the selected waypoint
            self.navigate_to_pose(waypoint_pose, waypoint_name)
            
        except ValueError:
            self.get_logger().warn("Invalid input. Please enter a number.")
    
    def navigate_to_pose(self, pose, waypoint_name):
        """Navigate to the given pose using the NavigateToPose action."""
        if self.navigating:
            self.get_logger().warn("Already navigating. Cancel current navigation first.")
            return
        
        self.get_logger().info(f"Navigating to waypoint '{waypoint_name}'...")
        
        # Create the goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        
        # Set the flag
        self.navigating = True
        
        # Send the goal and get a future for the result
        send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback
        )
        
        # Add a callback for when the goal is accepted
        send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """Callback for when the goal is accepted or rejected."""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().warn('Goal was rejected!')
            self.navigating = False
            return
        
        self.get_logger().info('Goal accepted!')
        self.current_goal_handle = goal_handle
        
        # Get the result future
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        """Callback for when the navigation action is completed."""
        result = future.result().result
        status = future.result().status
        
        if status == 4:  # SUCCEEDED
            self.get_logger().info('Navigation succeeded!')
        else:
            self.get_logger().warn(f'Navigation failed with status: {status}')
        
        self.navigating = False
        self.current_goal_handle = None
    
    def feedback_callback(self, feedback_msg):
        """Callback for navigation feedback."""
        feedback = feedback_msg.feedback
        # You can process feedback here if needed
        # For example, you could log the current position or distance to goal
    
    def cancel_navigation(self):
        """Cancel the current navigation goal."""
        if not self.navigating or self.current_goal_handle is None:
            self.get_logger().warn("No navigation in progress to cancel.")
            return
        
        self.get_logger().info("Cancelling current navigation...")
        
        # Cancel the goal
        cancel_future = self.current_goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self.cancel_done_callback)
        
        # Stop the robot immediately
        self.stop_robot()
    
    def cancel_done_callback(self, future):
        """Callback for when the cancel request is completed."""
        self.get_logger().info("Navigation cancelled.")
        self.navigating = False
        self.current_goal_handle = None
    
    def stop_robot(self):
        """Stop the robot by publishing zero velocity."""
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        
        # Publish multiple times to ensure the robot stops
        for _ in range(5):
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)
    
    def quit_program(self):
        """Stop the robot and quit the program."""
        self.get_logger().info("Quitting...")
        
        # Cancel navigation if in progress
        if self.navigating:
            self.cancel_navigation()
        
        # Stop the robot
        self.stop_robot()
        
        # Shutdown the node
        self.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

def main(args=None):
    # Initialize the ROS 2 context
    rclpy.init(args=args)
    
    # Set terminal to raw mode to get input without waiting for Enter key
    old_attr = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        
        # Create the node
        waypoint_nav = GazeboWaypointNavigation("gazebo_waypoint_navigation_node")
        
        # Create a MultiThreadedExecutor
        executor = MultiThreadedExecutor()
        
        # Add the node to the executor
        executor.add_node(waypoint_nav)
        
        try:
            # Spin the executor
            executor.spin()
        except KeyboardInterrupt:
            pass
        finally:
            # Cleanup
            executor.shutdown()
            waypoint_nav.destroy_node()
            
    finally:
        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)
    
    # Shutdown ROS 2
    rclpy.shutdown()

if __name__ == '__main__':
    main()
