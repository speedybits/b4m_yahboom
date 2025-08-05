#!/usr/bin/env python3
"""
Test SLAM Toolbox Launch and Initialization

This test validates that SLAM toolbox launches correctly in Gazebo simulation
and that all required SLAM services and topics are available.

Validates checklist items:
- SLAM Integration: SLAM toolbox running and ready for mapping
- Core Validation Requirements: SLAM Ready
"""

import unittest
import subprocess
import time
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
import threading
import signal
import os
import psutil


class SLAMTestNode(Node):
    def __init__(self):
        super().__init__('slam_test_node')
        self.map_received = False
        self.slam_services_available = False
        
    def check_slam_services(self):
        """Check if SLAM toolbox services are available"""
        required_services = [
            '/slam_toolbox/save_map',
            '/slam_toolbox/serialize_map',
            '/slam_toolbox/deserialize_map'
        ]
        
        available_services = self.get_service_names_and_types()
        service_names = [name for name, _ in available_services]
        
        for service in required_services:
            if service not in service_names:
                return False
        return True
        
    def map_callback(self, msg):
        """Callback for map topic"""
        self.map_received = True
        self.get_logger().info(f"Map received: {msg.info.width}x{msg.info.height}")


class TestSLAMLaunch(unittest.TestCase):
    """Test SLAM toolbox launch and initialization"""
    
    @classmethod
    def setUpClass(cls):
        """Set up test environment"""
        # Kill any existing processes
        cls.kill_existing_processes()
        
        # Initialize ROS
        rclpy.init()
        cls.test_node = SLAMTestNode()
        
        # Start executor in separate thread
        cls.executor = rclpy.executors.SingleThreadedExecutor()
        cls.executor.add_node(cls.test_node)
        cls.executor_thread = threading.Thread(target=cls.executor.spin)
        cls.executor_thread.daemon = True
        cls.executor_thread.start()
        
        # Launch processes
        cls.gazebo_process = None
        cls.robot_process = None
        cls.slam_process = None
        
    @classmethod
    def tearDownClass(cls):
        """Clean up test environment"""
        # Stop processes
        processes = [cls.slam_process, cls.robot_process, cls.gazebo_process]
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
            'slam_toolbox', 'ros2', 'rviz2'
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
        
        # Wait for processes to terminate
        time.sleep(2)
    
    def test_01_gazebo_launch(self):
        """Test that Ignition Gazebo launches successfully"""
        self.gazebo_process = subprocess.Popen(
            ['ign', 'gazebo', '--verbose'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            env=dict(os.environ, ROS_DOMAIN_ID='20')
        )
        
        # Wait for Gazebo to start
        time.sleep(10)
        
        # Check if Gazebo is running
        self.assertIsNone(self.gazebo_process.poll(), "Gazebo process terminated unexpectedly")
        
        # Check for Gazebo GUI process
        gazebo_running = False
        for proc in psutil.process_iter(['name', 'cmdline']):
            try:
                cmdline = ' '.join(proc.info['cmdline'] or [])
                if 'ign gazebo' in cmdline or 'ignition gazebo' in cmdline:
                    gazebo_running = True
                    break
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                pass
        
        self.assertTrue(gazebo_running, "Ignition Gazebo is not running")
    
    def test_02_robot_spawn(self):
        """Test that robot spawns successfully in Gazebo"""
        # Source ROS environment and launch robot
        cmd = [
            'bash', '-c',
            'source install/setup.bash && ros2 launch yahboomcar_nav spawn_robot_simple_gazebo.py'
        ]
        
        self.robot_process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            env=dict(os.environ, ROS_DOMAIN_ID='20')
        )
        
        # Wait for robot to spawn
        time.sleep(15)
        
        # Check if process is still running
        self.assertIsNone(self.robot_process.poll(), "Robot spawn process terminated unexpectedly")
        
        # Check for robot topics (basic validation)
        result = subprocess.run(
            ['bash', '-c', 'source install/setup.bash && timeout 5 ros2 topic list'],
            capture_output=True,
            text=True,
            env=dict(os.environ, ROS_DOMAIN_ID='20')
        )
        
        topics = result.stdout
        required_topics = ['/cmd_vel', '/odom', '/tf']
        
        for topic in required_topics:
            self.assertIn(topic, topics, f"Required topic {topic} not found")
    
    def test_03_slam_toolbox_launch(self):
        """Test that SLAM toolbox launches successfully"""
        # Launch SLAM mapping
        cmd = [
            'bash', '-c',
            'source install/setup.bash && ros2 launch yahboomcar_nav slam_mapping_gazebo.py'
        ]
        
        self.slam_process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            env=dict(os.environ, ROS_DOMAIN_ID='20')
        )
        
        # Wait for SLAM to start
        time.sleep(20)
        
        # Check if process is still running
        self.assertIsNone(self.slam_process.poll(), "SLAM process terminated unexpectedly")
    
    def test_04_slam_services_available(self):
        """Test that SLAM toolbox services are available"""
        # Wait for services to be available
        max_attempts = 30
        attempt = 0
        
        while attempt < max_attempts:
            if self.test_node.check_slam_services():
                break
            time.sleep(1)
            attempt += 1
        
        self.assertTrue(
            self.test_node.check_slam_services(),
            "SLAM toolbox services not available after 30 seconds"
        )
    
    def test_05_slam_topics_active(self):
        """Test that SLAM-related topics are publishing"""
        # Check for SLAM topics
        result = subprocess.run(
            ['bash', '-c', 'source install/setup.bash && timeout 10 ros2 topic list'],
            capture_output=True,
            text=True,
            env=dict(os.environ, ROS_DOMAIN_ID='20')
        )
        
        topics = result.stdout
        slam_topics = ['/map', '/tf', '/slam_toolbox/feedback']
        
        for topic in slam_topics:
            self.assertIn(topic, topics, f"SLAM topic {topic} not found")
        
        # Test map topic publishing
        result = subprocess.run(
            ['bash', '-c', 'source install/setup.bash && timeout 5 ros2 topic hz /map'],
            capture_output=True,
            text=True,
            env=dict(os.environ, ROS_DOMAIN_ID='20')
        )
        
        # Should get some output indicating topic is publishing
        self.assertNotEqual(len(result.stdout.strip()), 0, "Map topic not publishing data")
    
    def test_06_slam_ready_for_mapping(self):
        """Test that SLAM is ready to build maps"""
        # Subscribe to map topic briefly to verify it's working
        map_subscription = self.test_node.create_subscription(
            OccupancyGrid,
            '/map',
            self.test_node.map_callback,
            10
        )
        
        # Wait for map message
        timeout = 30
        start_time = time.time()
        
        while not self.test_node.map_received and (time.time() - start_time) < timeout:
            time.sleep(0.1)
        
        self.assertTrue(
            self.test_node.map_received,
            "No map data received from SLAM toolbox within 30 seconds"
        )
        
        # Clean up subscription
        self.test_node.destroy_subscription(map_subscription)


if __name__ == '__main__':
    # Set up test environment
    os.environ['ROS_DOMAIN_ID'] = '20'
    
    # Run tests
    unittest.main(verbosity=2)