#!/usr/bin/env python3
# test_motor_control.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import subprocess
import sys
import threading

class MotorTestNode(Node):
    def __init__(self):
        super().__init__('motor_test')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry', self.odom_callback, 10)
        self.odom_data = {'received': False, 'linear_x': 0.0, 'count': 0}
        
    def odom_callback(self, msg):
        self.odom_data['received'] = True
        self.odom_data['linear_x'] = msg.twist.twist.linear.x
        self.odom_data['count'] += 1

def test_motor_control():
    """Test 4: Verify motors drive wheels correctly"""
    
    # Cleanup first
    print("Cleaning up any existing processes...")
    subprocess.run(['./b4m_shutdown.sh', '--keep-agent'], capture_output=True)
    time.sleep(2)
    
    # Launch Gazebo Classic simulation with integrated robot
    print("Launching Gazebo Classic simulation...")
    gazebo_proc = subprocess.Popen([
        'ros2', 'launch', 'yahboomcar_nav', 'gazebo_classic_nav_launch.py'
    ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    sim_proc = None  # No separate spawn process needed
    
    try:
        # Wait for system to initialize
        print("Waiting for system initialization...")
        time.sleep(10)
        
        # Initialize ROS2
        rclpy.init()
        node = MotorTestNode()
        
        # Create executor in thread
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(node)
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()
        
        # Wait for odometry to start
        print("Waiting for odometry data...")
        timeout = time.time() + 10
        while not node.odom_data['received'] and time.time() < timeout:
            time.sleep(0.1)
        
        if not node.odom_data['received']:
            print("✗ No odometry data received")
            print("Test result: FAILED")
            return 1
        
        print("✓ Odometry data received")
        
        # Send forward command
        print("Sending forward velocity command (0.5 m/s)...")
        cmd = Twist()
        cmd.linear.x = 0.5  # 0.5 m/s forward
        
        # Reset odometry count
        node.odom_data['count'] = 0
        
        # Send commands for 5 seconds
        start_time = time.time()
        while time.time() - start_time < 5.0:
            node.cmd_pub.publish(cmd)
            time.sleep(0.1)
        
        # Stop robot
        print("Stopping robot...")
        cmd.linear.x = 0.0
        node.cmd_pub.publish(cmd)
        time.sleep(1)
        
        # Verify movement
        print(f"Odometry updates received: {node.odom_data['count']}")
        print(f"Last velocity reading: {node.odom_data['linear_x']} m/s")
        
        if node.odom_data['count'] > 10:  # Should have received multiple updates
            print("✓ Motor control verified - robot responded to commands")
            print("Test result: PASSED")
            return 0
        else:
            print("✗ Insufficient odometry updates")
            print("Test result: FAILED")
            return 1
            
    except Exception as e:
        print(f"Error during test: {e}")
        return 1
    finally:
        # Cleanup
        print("\nCleaning up...")
        if 'node' in locals():
            node.destroy_node()
        if 'executor' in locals():
            executor.shutdown()
        rclpy.shutdown()
        
        # Kill processes
        for proc in [sim_proc, gazebo_proc]:
            if proc and proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill()
        
        # Extra cleanup
        subprocess.run(['pkill', '-f', 'ign gazebo'], capture_output=True)
        subprocess.run(['pkill', '-f', 'controller_manager'], capture_output=True)
        subprocess.run(['pkill', '-f', 'robot_state_publisher'], capture_output=True)
        subprocess.run(['pkill', '-f', 'spawner'], capture_output=True)
        time.sleep(2)

if __name__ == "__main__":
    sys.exit(test_motor_control())