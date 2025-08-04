#!/usr/bin/env python3
"""
Diagnostic script to test gazebo_ros2_control controller manager service
Based on Grok AI analysis - tests for executor context and service responsiveness
"""

import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import ListControllers
import time
import subprocess
import sys

class ControllerManagerTester(Node):
    def __init__(self):
        super().__init__('controller_manager_tester')
        self.get_logger().info('Controller Manager Tester starting...')
        
        # Create service client
        self.client = self.create_client(ListControllers, '/controller_manager/list_controllers')
        
        # Wait for service with timeout
        self.get_logger().info('Waiting for /controller_manager/list_controllers service...')
        start_time = time.time()
        timeout = 20.0
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            elapsed = time.time() - start_time
            if elapsed > timeout:
                self.get_logger().error(f'Service not available after {timeout}s timeout')
                return
            self.get_logger().info(f'Still waiting... ({elapsed:.1f}s)')
        
        self.get_logger().info('Service is available! Testing service call...')
        
        # Test service call
        request = ListControllers.Request()
        future = self.client.call_async(request)
        
        # Wait for response with timeout
        start_time = time.time()
        while not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            elapsed = time.time() - start_time
            if elapsed > 5.0:
                self.get_logger().error('Service call timed out after 5s - executor may be blocked')
                self.diagnose_issue()
                return
                
        try:
            response = future.result()
            self.get_logger().info(f'Service responded! Controllers: {[c.name for c in response.controller]}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')
            
    def diagnose_issue(self):
        """Run diagnostic commands to help identify the issue"""
        self.get_logger().info('\n=== DIAGNOSTICS ===')
        
        # Check node list
        result = subprocess.run(['ros2', 'node', 'list'], capture_output=True, text=True)
        self.get_logger().info(f'Active nodes:\n{result.stdout}')
        
        # Check gazebo_ros2_control node
        if '/gazebo_ros2_control' in result.stdout:
            result = subprocess.run(['ros2', 'node', 'info', '/gazebo_ros2_control'], 
                                  capture_output=True, text=True)
            self.get_logger().info(f'gazebo_ros2_control node info:\n{result.stdout}')
            
        # Check hardware interfaces
        result = subprocess.run(['ros2', 'control', 'list_hardware_interfaces'], 
                              capture_output=True, text=True)
        self.get_logger().info(f'Hardware interfaces:\n{result.stdout}')
        
        self.get_logger().info('\nPossible causes based on Grok AI analysis:')
        self.get_logger().info('1. Executor blocked - service advertised but callbacks not processed')
        self.get_logger().info('2. Hardware interface initialization failed')
        self.get_logger().info('3. Race condition - service advertised before executor spinning')
        self.get_logger().info('4. Gazebo Classic + ROS2 Humble compatibility issue')

def main():
    rclpy.init()
    
    tester = ControllerManagerTester()
    
    try:
        rclpy.spin_once(tester, timeout_sec=30.0)
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()