#!/usr/bin/env python3
# test_turning.py - Simple test to diagnose turning issues

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import math
import subprocess
import sys
import threading

class TurningTestNode(Node):
    def __init__(self):
        super().__init__('turning_test')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry', self.odom_callback, 10)
        
        self.position = {'x': 0.0, 'y': 0.0}
        self.orientation = 0.0  # yaw in radians
        self.angular_velocity = 0.0
        self.linear_velocity = 0.0
        
    def odom_callback(self, msg):
        self.position['x'] = msg.pose.pose.position.x
        self.position['y'] = msg.pose.pose.position.y
        
        # Extract yaw from quaternion (correct formula)
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        # Convert quaternion to yaw angle
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        self.orientation = math.atan2(siny_cosp, cosy_cosp)
        
        # Get velocities
        self.linear_velocity = msg.twist.twist.linear.x
        self.angular_velocity = msg.twist.twist.angular.z

def test_turning():
    """Diagnose turning issues"""
    
    # Cleanup first
    print("Cleaning up any existing processes...")
    subprocess.run(['./b4m_shutdown.sh', '--keep-agent'], capture_output=True)
    time.sleep(2)
    
    # Launch simulation with GUI
    print("Launching simulation with GUI...")
    gazebo_proc = subprocess.Popen([
        'ros2', 'launch', 'yahboomcar_nav', 'ignition_gazebo_launch.py'
    ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    
    sim_proc = subprocess.Popen([
        'ros2', 'launch', 'yahboomcar_nav', 'spawn_robot_with_controllers_ignition.py'
    ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    
    try:
        # Wait for system initialization
        print("Waiting for system initialization...")
        time.sleep(10)
        
        # Initialize ROS2
        rclpy.init()
        node = TurningTestNode()
        
        # Create executor in thread
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(node)
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()
        
        # Wait for odometry
        time.sleep(2)
        
        print(f"\nInitial state:")
        print(f"Position: ({node.position['x']:.3f}, {node.position['y']:.3f})")
        print(f"Orientation: {math.degrees(node.orientation):.1f}°")
        print(f"Linear vel: {node.linear_velocity:.3f} m/s")
        print(f"Angular vel: {node.angular_velocity:.3f} rad/s")
        
        # Test 1: Pure rotation (no linear motion)
        print(f"\n=== Test 1: Pure Rotation ===")
        cmd = Twist()
        cmd.angular.z = 0.5  # 0.5 rad/s clockwise
        
        print("Sending angular.z = 0.5 rad/s for 5 seconds...")
        start_orientation = node.orientation
        
        for i in range(50):  # 5 seconds at 10Hz
            node.cmd_pub.publish(cmd)
            time.sleep(0.1)
            if i % 10 == 0:  # Print every second
                angle_change = node.orientation - start_orientation
                print(f"  t={i/10:.1f}s: angle_change={math.degrees(angle_change):.1f}°, "
                      f"reported_angular_vel={node.angular_velocity:.3f} rad/s")
        
        # Stop
        cmd.angular.z = 0.0
        node.cmd_pub.publish(cmd)
        time.sleep(1)
        
        total_angle_change = node.orientation - start_orientation
        expected_change = 0.5 * 5  # 0.5 rad/s * 5 seconds = 2.5 radians
        
        print(f"\nTest 1 Results:")
        print(f"Expected angle change: {math.degrees(expected_change):.1f}°")
        print(f"Actual angle change: {math.degrees(total_angle_change):.1f}°")
        print(f"Final position: ({node.position['x']:.3f}, {node.position['y']:.3f})")
        
        if abs(total_angle_change) < 0.1:
            print("❌ PROBLEM: Robot did not rotate!")
            print("   This suggests angular velocity commands are not working.")
        else:
            print("✅ Robot rotated successfully")
        
        # Test 2: Forward motion
        print(f"\n=== Test 2: Forward Motion ===")
        cmd = Twist()
        cmd.linear.x = 0.2  # 0.2 m/s forward
        
        print("Sending linear.x = 0.2 m/s for 3 seconds...")
        start_position = {'x': node.position['x'], 'y': node.position['y']}
        
        for i in range(30):  # 3 seconds at 10Hz
            node.cmd_pub.publish(cmd)
            time.sleep(0.1)
            if i % 10 == 0:  # Print every second
                dx = node.position['x'] - start_position['x']
                dy = node.position['y'] - start_position['y']
                distance = math.sqrt(dx**2 + dy**2)
                print(f"  t={i/10:.1f}s: distance_moved={distance:.3f}m, "
                      f"reported_linear_vel={node.linear_velocity:.3f} m/s")
        
        # Stop
        cmd.linear.x = 0.0
        node.cmd_pub.publish(cmd)
        
        dx = node.position['x'] - start_position['x']
        dy = node.position['y'] - start_position['y']
        total_distance = math.sqrt(dx**2 + dy**2)
        expected_distance = 0.2 * 3  # 0.2 m/s * 3 seconds = 0.6 meters
        
        print(f"\nTest 2 Results:")
        print(f"Expected distance: {expected_distance:.3f}m")
        print(f"Actual distance: {total_distance:.3f}m")
        
        if total_distance < 0.1:
            print("❌ PROBLEM: Robot did not move forward!")
        else:
            print("✅ Robot moved forward successfully")
        
        print(f"\n=== DIAGNOSIS ===")
        if abs(total_angle_change) < 0.1 and total_distance > 0.1:
            print("🔍 ISSUE FOUND: Robot can move forward but cannot turn")
            print("   Possible causes:")
            print("   1. DiffDrive plugin not receiving angular velocity commands")
            print("   2. Wheel separation distance is wrong")
            print("   3. Only using front wheels instead of all 4 wheels")
            print("   4. Joint names in DiffDrive plugin are incorrect")
        elif abs(total_angle_change) < 0.1 and total_distance < 0.1:
            print("🔍 ISSUE FOUND: Robot not responding to any commands")
            print("   Possible causes:")
            print("   1. cmd_vel topic not connected")
            print("   2. DiffDrive plugin not loaded")
            print("   3. Controllers not properly initialized")
        else:
            print("✅ Both forward motion and rotation working normally")
        
        return 0
            
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
        
        for proc in [sim_proc, gazebo_proc]:
            if proc and proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill()
        
        subprocess.run(['pkill', '-f', 'ign gazebo'], capture_output=True)
        subprocess.run(['pkill', '-f', 'controller_manager'], capture_output=True)
        subprocess.run(['pkill', '-f', 'robot_state_publisher'], capture_output=True)
        time.sleep(2)

if __name__ == "__main__":
    sys.exit(test_turning())