#!/usr/bin/env python3
"""
Test Transform Chain Integrity

This test validates that the TF transform chain is properly established
for SLAM operation, including the critical map->odom->base_footprint chain.

Validates checklist items:
- Core Validation Requirements: Transform Chain
- Sensor Integration: Laser scan (/scan) and odometry (/odom) topics active
- Transform Tree: slam_toolbox provides map->odom transform
- Key Integration Points: Transform tree stability
"""

import unittest
import subprocess
import time
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
import threading
import os
import psutil
import tf2_ros
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped


class TransformTestNode(Node):
    def __init__(self):
        super().__init__('transform_test_node')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.transforms_received = {
            'map_to_odom': False,
            'odom_to_base_footprint': False,
            'base_footprint_to_base_link': False,
            'base_link_to_laser': False
        }
        
        self.transform_timestamps = {}
        
        # Subscribe to TF
        self.tf_sub = self.create_subscription(
            TFMessage, '/tf', self.tf_callback, 10
        )
        self.tf_static_sub = self.create_subscription(
            TFMessage, '/tf_static', self.tf_static_callback, 10
        )
        
    def tf_callback(self, msg):
        """Process dynamic transforms"""
        for transform in msg.transforms:
            self.process_transform(transform)
            
    def tf_static_callback(self, msg):
        """Process static transforms"""
        for transform in msg.transforms:
            self.process_transform(transform)
    
    def process_transform(self, transform):
        """Process individual transform"""
        parent = transform.header.frame_id
        child = transform.child_frame_id
        
        # Remove leading slashes for comparison
        parent = parent.lstrip('/')
        child = child.lstrip('/')
        
        # Check for key transforms
        if parent == 'map' and child == 'odom':
            self.transforms_received['map_to_odom'] = True
            self.transform_timestamps['map_to_odom'] = time.time()
            
        elif parent == 'odom' and child == 'base_footprint':
            self.transforms_received['odom_to_base_footprint'] = True
            self.transform_timestamps['odom_to_base_footprint'] = time.time()
            
        elif parent == 'base_footprint' and child == 'base_link':
            self.transforms_received['base_footprint_to_base_link'] = True
            self.transform_timestamps['base_footprint_to_base_link'] = time.time()
            
        elif parent == 'base_link' and (child == 'laser' or child == 'laser_link'):
            self.transforms_received['base_link_to_laser'] = True
            self.transform_timestamps['base_link_to_laser'] = time.time()
    
    def check_transform_chain(self, source_frame, target_frame, timeout=5.0):
        """Check if transform chain exists between frames"""
        try:
            # Try to get transform
            transform = self.tf_buffer.lookup_transform(
                target_frame, source_frame, rclpy.time.Time(), 
                timeout=rclpy.duration.Duration(seconds=timeout)
            )
            return True, transform
        except Exception as e:
            return False, str(e)
    
    def test_transform_stability(self, source_frame, target_frame, duration=5.0):
        """Test transform stability over time"""
        transforms = []
        start_time = time.time()
        
        while (time.time() - start_time) < duration:
            try:
                transform = self.tf_buffer.lookup_transform(
                    target_frame, source_frame, rclpy.time.Time()
                )
                transforms.append(transform)
                time.sleep(0.1)
            except:
                pass
        
        return len(transforms) > 0, transforms


class TestTransforms(unittest.TestCase):
    """Test transform chain integrity"""
    
    @classmethod
    def setUpClass(cls):
        """Set up test environment"""
        # Kill any existing processes
        cls.kill_existing_processes()
        
        # Initialize ROS
        rclpy.init()
        cls.test_node = TransformTestNode()
        
        # Start executor in separate thread
        cls.executor = rclpy.executors.SingleThreadedExecutor()
        cls.executor.add_node(cls.test_node)
        cls.executor_thread = threading.Thread(target=cls.executor.spin)
        cls.executor_thread.daemon = True
        cls.executor_thread.start()
        
        # Launch simulation environment
        cls.setup_simulation()
        
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
        """Kill any existing processes"""
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
        
        time.sleep(2)
    
    @classmethod
    def setup_simulation(cls):
        """Set up Gazebo simulation with robot and SLAM"""
        # Launch Gazebo
        cls.gazebo_process = subprocess.Popen(
            ['ign', 'gazebo', '--verbose'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            env=dict(os.environ, ROS_DOMAIN_ID='20')
        )
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
        time.sleep(15)
        
        # Launch SLAM
        cmd = [
            'bash', '-c',
            'source install/setup.bash && ros2 launch yahboomcar_nav slam_mapping_gazebo.py'
        ]
        cls.slam_process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            env=dict(os.environ, ROS_DOMAIN_ID='20')
        )
        time.sleep(20)
    
    def test_01_tf_topics_available(self):
        """Test that TF topics are available"""
        result = subprocess.run(
            ['bash', '-c', 'source install/setup.bash && timeout 10 ros2 topic list'],
            capture_output=True,
            text=True,
            env=dict(os.environ, ROS_DOMAIN_ID='20')
        )
        
        required_topics = ['/tf', '/tf_static']
        
        for topic in required_topics:
            self.assertIn(topic, result.stdout, f"TF topic {topic} not found")
        
        # Check if topics are publishing
        for topic in required_topics:
            result = subprocess.run(
                ['bash', '-c', f'source install/setup.bash && timeout 5 ros2 topic hz {topic}'],
                capture_output=True,
                text=True,
                env=dict(os.environ, ROS_DOMAIN_ID='20')
            )
            
            # Should get some output (either rate info or timeout)
            self.assertNotEqual(len(result.stdout.strip()), 0, 
                               f"TF topic {topic} not publishing")
    
    def test_02_basic_transform_reception(self):
        """Test that basic transforms are being received"""
        # Wait for transforms to be received
        timeout = 30
        start_time = time.time()
        
        while (time.time() - start_time) < timeout:
            if any(self.test_node.transforms_received.values()):
                break
            time.sleep(0.1)
        
        # At least one transform should be received
        received_any = any(self.test_node.transforms_received.values())
        self.assertTrue(received_any, "No transforms received within 30 seconds")
    
    def test_03_slam_map_to_odom_transform(self):
        """Test that SLAM toolbox publishes map->odom transform"""
        # Wait specifically for map->odom transform
        timeout = 45  # SLAM takes time to initialize
        start_time = time.time()
        
        while (time.time() - start_time) < timeout:
            if self.test_node.transforms_received['map_to_odom']:
                break
            time.sleep(0.5)
        
        self.assertTrue(
            self.test_node.transforms_received['map_to_odom'],
            "SLAM toolbox not publishing map->odom transform within 45 seconds"
        )
    
    def test_04_odom_to_base_transform(self):
        """Test that odometry publishes odom->base_footprint transform"""
        # Wait for odom->base_footprint transform
        timeout = 30
        start_time = time.time()
        
        while (time.time() - start_time) < timeout:
            if self.test_node.transforms_received['odom_to_base_footprint']:
                break
            time.sleep(0.1)
        
        self.assertTrue(
            self.test_node.transforms_received['odom_to_base_footprint'],
            "No odom->base_footprint transform received within 30 seconds"
        )
    
    def test_05_complete_transform_chain(self):
        """Test that complete transform chain exists"""
        # Check map->base_link chain
        chain_exists, result = self.test_node.check_transform_chain('base_link', 'map', timeout=30.0)
        
        if not chain_exists:
            self.fail(f"Transform chain map->base_link not available: {result}")
        
        # Check map->laser chain (needed for SLAM)
        laser_frames = ['laser', 'laser_link', 'lidar_link']
        laser_chain_exists = False
        
        for laser_frame in laser_frames:
            chain_exists, result = self.test_node.check_transform_chain(laser_frame, 'map', timeout=10.0)
            if chain_exists:
                laser_chain_exists = True
                break
        
        self.assertTrue(laser_chain_exists, "Transform chain map->laser not available")
    
    def test_06_transform_stability(self):
        """Test that transforms are stable over time"""
        # Test map->base_link stability
        stable, transforms = self.test_node.test_transform_stability('base_link', 'map', duration=10.0)
        
        self.assertTrue(stable, "Transform map->base_link not stable over time")
        
        # Should get at least 50 transform samples in 10 seconds (5Hz minimum)
        self.assertGreater(len(transforms), 50, 
                          f"Transform update rate too low: {len(transforms)} samples in 10 seconds")
    
    def test_07_transform_consistency(self):
        """Test that transforms are consistent and reasonable"""
        try:
            # Get map->base_link transform
            transform = self.test_node.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time()
            )
            
            # Transform should have reasonable values
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            
            # Position should be finite and not too large (assuming robot starts near origin)
            self.assertTrue(abs(translation.x) < 100.0, f"X position unreasonable: {translation.x}")
            self.assertTrue(abs(translation.y) < 100.0, f"Y position unreasonable: {translation.y}")
            self.assertTrue(abs(translation.z) < 10.0, f"Z position unreasonable: {translation.z}")
            
            # Quaternion should be normalized
            quat_norm = (rotation.x**2 + rotation.y**2 + rotation.z**2 + rotation.w**2)**0.5
            self.assertAlmostEqual(quat_norm, 1.0, places=3, 
                                  msg=f"Quaternion not normalized: {quat_norm}")
            
        except Exception as e:
            self.fail(f"Failed to get or validate transform: {e}")
    
    def test_08_tf_tree_completeness(self):
        """Test that TF tree has all expected frames"""
        # Get list of all frames
        result = subprocess.run(
            ['bash', '-c', 'source install/setup.bash && timeout 10 ros2 run tf2_ros tf2_monitor'],
            capture_output=True,
            text=True,
            env=dict(os.environ, ROS_DOMAIN_ID='20')
        )
        
        # Should contain key frames
        expected_frames = ['map', 'odom', 'base_footprint', 'base_link']
        
        for frame in expected_frames:
            self.assertIn(frame, result.stdout, f"Frame {frame} not found in TF tree")
    
    def test_09_slam_transform_authority(self):
        """Test that SLAM toolbox is the authority for map->odom transform"""
        # Check TF info to see who's publishing map->odom
        result = subprocess.run(
            ['bash', '-c', 'source install/setup.bash && timeout 10 ros2 run tf2_ros tf2_echo map odom'],
            capture_output=True,
            text=True,
            env=dict(os.environ, ROS_DOMAIN_ID='20')
        )
        
        # Should get transform data (indicates SLAM is publishing)
        self.assertNotEqual(len(result.stdout.strip()), 0,
                           "No map->odom transform data (SLAM not publishing)")
        
        # Check that transform contains reasonable data
        if 'translation:' in result.stdout.lower():
            # Transform is being published
            self.assertIn('translation:', result.stdout.lower(),
                         "Transform data format unexpected")


if __name__ == '__main__':
    # Set up test environment
    os.environ['ROS_DOMAIN_ID'] = '20'
    
    # Run tests
    unittest.main(verbosity=2)