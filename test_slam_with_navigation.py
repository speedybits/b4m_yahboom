#!/usr/bin/env python3

"""
Test SLAM mapping with the working square navigation system
This combines the working test_square_corners.py approach with SLAM toolbox
"""

import sys
import subprocess
import time
import signal
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
import math

class SLAMNavigationTest(Node):
    def __init__(self):
        super().__init__('slam_navigation_test')
        
        # Test state
        self.map_received = False
        self.scan_received = False
        self.map_data = None
        self.scan_data = None
        
        # Subscribe to SLAM outputs
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        self.get_logger().info("SLAM Navigation Test node started")
        
    def map_callback(self, msg):
        if not self.map_received:
            self.map_received = True
            self.get_logger().info(f"Map received! Size: {msg.info.width}x{msg.info.height}")
        self.map_data = msg
        
    def scan_callback(self, msg):
        if not self.scan_received:
            self.scan_received = True
            self.get_logger().info(f"Laser scan received! {len(msg.ranges)} points")
        self.scan_data = msg

def test_slam_with_navigation():
    """Test SLAM functionality with the working navigation system"""
    
    print("=== SLAM Navigation Test ===")
    print("Testing SLAM mapping with working robot navigation")
    print()
    
    sim_proc = None
    gazebo_proc = None
    slam_proc = None
    
    try:
        # Initialize ROS2
        rclpy.init()
        
        # Start the test node
        test_node = SLAMNavigationTest()
        
        print("Step 1: Starting Ignition Gazebo...")
        sim_proc = subprocess.Popen([
            'ros2', 'launch', 'yahboomcar_nav', 'ignition_gazebo_launch.py'
        ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        
        time.sleep(8)  # Wait for Gazebo to start
        
        print("Step 2: Spawning robot with controllers...")
        gazebo_proc = subprocess.Popen([
            'ros2', 'launch', 'yahboomcar_nav', 'spawn_robot_with_controllers_ignition.py'
        ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        
        time.sleep(8)  # Wait for robot to spawn
        
        print("Step 3: Starting SLAM toolbox...")
        slam_proc = subprocess.Popen([
            'ros2', 'launch', 'yahboomcar_nav', 'slam_toolbox_launch.py'
        ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        
        time.sleep(5)  # Wait for SLAM to start
        
        print("Step 4: Testing SLAM functionality...")
        
        # Test for 30 seconds
        start_time = time.time()
        test_duration = 30
        
        while time.time() - start_time < test_duration:
            rclpy.spin_once(test_node, timeout_sec=0.1)
            
            # Check if we have both scan and map data
            if test_node.scan_received and test_node.map_received:
                print("✅ SUCCESS: Both laser scan and map data received!")
                print(f"  - Laser scan: {len(test_node.scan_data.ranges) if test_node.scan_data else 0} points")
                print(f"  - Map size: {test_node.map_data.info.width}x{test_node.map_data.info.height} cells")
                return 0
        
        # Check partial success
        if test_node.scan_received:
            print("⚠️  PARTIAL: Laser scan working but no map data")
            return 1
        elif test_node.map_received:
            print("⚠️  PARTIAL: Map data received but no laser scan")
            return 1
        else:
            print("❌ FAILED: No laser scan or map data received")
            return 1
            
    except KeyboardInterrupt:
        print("\nTest interrupted")
        return 1
        
    except Exception as e:
        print(f"Test error: {e}")
        return 1
        
    finally:
        print("\nCleaning up...")
        
        # Cleanup processes
        for proc in [slam_proc, gazebo_proc, sim_proc]:
            if proc and proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill()
        
        # Force cleanup
        subprocess.run(['pkill', '-f', 'slam_toolbox'], capture_output=True)
        subprocess.run(['pkill', '-f', 'ign gazebo'], capture_output=True)
        subprocess.run(['pkill', '-f', 'robot_state_publisher'], capture_output=True)
        time.sleep(2)
        
        # Shutdown ROS
        try:
            rclpy.shutdown()
        except:
            pass

if __name__ == "__main__":
    exit_code = test_slam_with_navigation()
    sys.exit(exit_code)