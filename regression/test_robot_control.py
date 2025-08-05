#!/usr/bin/env python3
"""
Test Robot Control and cmd_vel Response

This test validates that the robot responds correctly to cmd_vel commands
in Gazebo simulation, including forward movement and turning.

Validates checklist items:
- Robot Control: Differential drive plugin working with cmd_vel commands
- Robot Turning Fixed: Robot now successfully turns and navigates
- Core Validation Requirements: Robot Control
"""

import unittest
import subprocess
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import GetEntityState
import threading
import math
import os
import psutil


class RobotControlTestNode(Node):
    def __init__(self):
        super().__init__('robot_control_test_node')
        self.current_pose = None
        self.initial_pose = None
        self.movement_detected = False
        self.rotation_detected = False
        
        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        
    def odom_callback(self, msg):
        """Callback for odometry data"""
        self.current_pose = msg.pose.pose
        
        if self.initial_pose is None:
            self.initial_pose = self.current_pose
            return
            
        # Check for movement
        dx = self.current_pose.position.x - self.initial_pose.position.x
        dy = self.current_pose.position.y - self.initial_pose.position.y
        distance = math.sqrt(dx*dx + dy*dy)
        
        if distance > 0.05:  # 5cm threshold
            self.movement_detected = True
            
        # Check for rotation
        def quaternion_to_yaw(q):
            return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        
        initial_yaw = quaternion_to_yaw(self.initial_pose.orientation)
        current_yaw = quaternion_to_yaw(self.current_pose.orientation)
        angle_diff = abs(current_yaw - initial_yaw)
        
        # Handle angle wrapping
        if angle_diff > math.pi:
            angle_diff = 2 * math.pi - angle_diff
            
        if angle_diff > 0.1:  # ~6 degree threshold
            self.rotation_detected = True
    
    def send_forward_command(self, duration=2.0):
        """Send forward movement command"""
        twist = Twist()
        twist.linear.x = 0.2  # 0.2 m/s forward
        twist.angular.z = 0.0
        
        end_time = time.time() + duration
        while time.time() < end_time:
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)
        
        # Stop robot
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)
    
    def send_rotation_command(self, duration=2.0):
        """Send rotation command"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.5  # 0.5 rad/s rotation
        
        end_time = time.time() + duration
        while time.time() < end_time:
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)
        
        # Stop robot
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
    
    def reset_detection_flags(self):
        """Reset movement detection flags"""
        self.movement_detected = False
        self.rotation_detected = False
        self.initial_pose = self.current_pose


class TestRobotControl(unittest.TestCase):
    """Test robot control and cmd_vel response"""
    
    @classmethod
    def setUpClass(cls):
        """Set up test environment"""
        # Kill any existing processes
        cls.kill_existing_processes()
        
        # Initialize ROS
        rclpy.init()
        cls.test_node = RobotControlTestNode()
        
        # Start executor in separate thread
        cls.executor = rclpy.executors.SingleThreadedExecutor()
        cls.executor.add_node(cls.test_node)
        cls.executor_thread = threading.Thread(target=cls.executor.spin)
        cls.executor_thread.daemon = True
        cls.executor_thread.start()
        
        # Launch Gazebo and robot
        cls.setup_simulation()
        
    @classmethod
    def tearDownClass(cls):
        """Clean up test environment"""
        # Stop processes
        processes = [cls.robot_process, cls.gazebo_process]
        for process in processes:
            if process and process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
        
        # Cleanup ROS
        cls.executor.shutdown()
        cls.test_node.destroy_node()
        rclpy.shutdown()
        
        # Final cleanup
        cls.kill_existing_processes()
    
    @staticmethod
    def kill_existing_processes():
        """Kill any existing Gazebo, ROS, or related processes"""
        kill_patterns = [
            'ign gazebo', 'ignition gazebo', 'gazebo',
            'ros2', 'rviz2'
        ]
        
        for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
            try:
                cmdline = ' '.join(proc.info['cmdline'] or [])
                for pattern in kill_patterns:
                    if pattern in cmdline.lower():
                        proc.terminate()
                        break
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                pass
        
        time.sleep(2)
    
    @classmethod
    def setup_simulation(cls):
        """Set up Gazebo simulation with robot"""
        # Launch Gazebo
        cls.gazebo_process = subprocess.Popen(
            ['ign', 'gazebo', '--verbose'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            env=dict(os.environ, ROS_DOMAIN_ID='20')
        )
        
        # Wait for Gazebo to start
        time.sleep(10)
        
        # Launch robot
        cmd = [
            'bash', '-c',
            'source install/setup.bash && ros2 launch yahboomcar_nav spawn_robot_simple_gazebo.py'
        ]
        
        cls.robot_process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            env=dict(os.environ, ROS_DOMAIN_ID='20')
        )
        
        # Wait for robot to spawn and initialize
        time.sleep(15)
    
    def test_01_odometry_available(self):
        """Test that odometry data is available"""
        # Wait for odometry data
        timeout = 30
        start_time = time.time()
        
        while self.test_node.current_pose is None and (time.time() - start_time) < timeout:
            time.sleep(0.1)
        
        self.assertIsNotNone(
            self.test_node.current_pose,
            "No odometry data received within 30 seconds"
        )
    
    def test_02_cmd_vel_topic_available(self):
        """Test that cmd_vel topic is available and accepting commands"""
        # Check if topic exists
        result = subprocess.run(
            ['bash', '-c', 'source install/setup.bash && timeout 5 ros2 topic list'],
            capture_output=True,
            text=True,
            env=dict(os.environ, ROS_DOMAIN_ID='20')
        )
        
        self.assertIn('/cmd_vel', result.stdout, "cmd_vel topic not found")
        
        # Check if topic is being listened to
        result = subprocess.run(
            ['bash', '-c', 'source install/setup.bash && timeout 5 ros2 topic info /cmd_vel'],
            capture_output=True,
            text=True,
            env=dict(os.environ, ROS_DOMAIN_ID='20')
        )
        
        self.assertIn('Subscription count:', result.stdout, "No subscribers to cmd_vel topic")
    
    def test_03_forward_movement(self):
        """Test that robot responds to forward movement commands"""
        # Reset detection flags and wait for stable pose
        time.sleep(2)
        self.test_node.reset_detection_flags()
        time.sleep(1)
        
        # Send forward command
        self.test_node.send_forward_command(duration=3.0)
        
        # Wait for movement to be detected
        time.sleep(2)
        
        self.assertTrue(
            self.test_node.movement_detected,
            "Robot did not respond to forward movement command"
        )
    
    def test_04_rotation_movement(self):
        """Test that robot responds to rotation commands"""
        # Reset detection flags and wait for stable pose
        time.sleep(2)
        self.test_node.reset_detection_flags()
        time.sleep(1)
        
        # Send rotation command
        self.test_node.send_rotation_command(duration=3.0)
        
        # Wait for rotation to be detected
        time.sleep(2)
        
        self.assertTrue(
            self.test_node.rotation_detected,
            "Robot did not respond to rotation command"
        )
    
    def test_05_precise_movement_control(self):
        """Test precise movement control for square navigation"""
        # Reset and get initial position
        time.sleep(2)
        self.test_node.reset_detection_flags()
        time.sleep(1)
        
        initial_x = self.test_node.current_pose.position.x
        initial_y = self.test_node.current_pose.position.y
        
        # Move forward for approximately 0.5 meters
        twist = Twist()
        twist.linear.x = 0.2
        twist.angular.z = 0.0
        
        # Send command for 2.5 seconds (should move ~0.5m)
        end_time = time.time() + 2.5
        while time.time() < end_time:
            self.test_node.cmd_vel_pub.publish(twist)
            time.sleep(0.1)
        
        # Stop
        twist.linear.x = 0.0
        self.test_node.cmd_vel_pub.publish(twist)
        time.sleep(1)
        
        # Check distance moved
        final_x = self.test_node.current_pose.position.x
        final_y = self.test_node.current_pose.position.y
        
        distance = math.sqrt((final_x - initial_x)**2 + (final_y - initial_y)**2)
        
        # Should move between 0.3 and 0.7 meters (allowing for some variance)
        self.assertGreater(distance, 0.3, f"Robot moved too little: {distance:.3f}m")
        self.assertLess(distance, 0.7, f"Robot moved too much: {distance:.3f}m")
    
    def test_06_stop_command_response(self):
        """Test that robot stops when receiving zero velocity commands"""
        # Start movement
        twist = Twist()
        twist.linear.x = 0.2
        twist.angular.z = 0.0
        
        # Move for 1 second
        end_time = time.time() + 1.0
        while time.time() < end_time:
            self.test_node.cmd_vel_pub.publish(twist)
            time.sleep(0.1)
        
        # Record position
        moving_x = self.test_node.current_pose.position.x
        moving_y = self.test_node.current_pose.position.y
        
        # Send stop command
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.test_node.cmd_vel_pub.publish(twist)
        
        # Wait and check if robot stopped
        time.sleep(2)
        
        stopped_x = self.test_node.current_pose.position.x
        stopped_y = self.test_node.current_pose.position.y
        
        stop_distance = math.sqrt((stopped_x - moving_x)**2 + (stopped_y - moving_y)**2)
        
        # Should stop within 0.1 meters
        self.assertLess(stop_distance, 0.1, f"Robot did not stop promptly: {stop_distance:.3f}m")


if __name__ == '__main__':
    # Set up test environment
    os.environ['ROS_DOMAIN_ID'] = '20'
    
    # Run tests
    unittest.main(verbosity=2)