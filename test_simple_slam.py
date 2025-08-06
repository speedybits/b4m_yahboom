#!/usr/bin/env python3

"""
Simple test to check if SLAM toolbox can start and publish map topic
"""

import sys
import subprocess
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid

class SimpleSLAMTest(Node):
    def __init__(self):
        super().__init__('simple_slam_test')
        
        self.map_received = False
        
        # Subscribe to map topic
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        
        self.get_logger().info("Simple SLAM test started")
        
    def map_callback(self, msg):
        if not self.map_received:
            self.map_received = True
            self.get_logger().info(f"Map received! Size: {msg.info.width}x{msg.info.height}")

def test_simple_slam():
    """Test if SLAM toolbox can start and publish map"""
    
    print("=== Simple SLAM Test ===")
    print("Testing if SLAM toolbox can start and publish a map topic")
    print()
    
    slam_proc = None
    
    try:
        # Initialize ROS2
        rclpy.init()
        test_node = SimpleSLAMTest()
        
        print("Starting SLAM toolbox...")
        slam_proc = subprocess.Popen([
            'ros2', 'launch', 'yahboomcar_nav', 'slam_toolbox_launch.py'
        ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        
        time.sleep(8)  # Wait for SLAM to start
        
        print("Testing for map topic...")
        
        # Test for 20 seconds
        start_time = time.time()
        test_duration = 20
        
        while time.time() - start_time < test_duration:
            rclpy.spin_once(test_node, timeout_sec=0.1)
            
            if test_node.map_received:
                print("✅ SUCCESS: Map topic is being published!")
                return 0
        
        print("❌ FAILED: No map topic received within 20 seconds")
        return 1
            
    except Exception as e:
        print(f"Test error: {e}")
        return 1
        
    finally:
        print("Cleaning up...")
        
        if slam_proc and slam_proc.poll() is None:
            slam_proc.terminate()
            try:
                slam_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                slam_proc.kill()
        
        subprocess.run(['pkill', '-f', 'slam_toolbox'], capture_output=True)
        time.sleep(1)
        
        try:
            rclpy.shutdown()
        except:
            pass

if __name__ == "__main__":
    exit_code = test_simple_slam()
    sys.exit(exit_code)