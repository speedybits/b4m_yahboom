#!/usr/bin/env python3
# test_square_navigation.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import math
import subprocess
import sys
import threading

class SquareNavigationNode(Node):
    def __init__(self):
        super().__init__('square_navigation_test')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry', self.odom_callback, 10)
        
        # Position tracking
        self.position = {'x': 0.0, 'y': 0.0}
        self.orientation = 0.0  # yaw in radians
        self.path_points = []
        
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
        
    def move_forward(self, distance, speed=0.2):
        """Move forward a specified distance"""
        start_pos = {'x': self.position['x'], 'y': self.position['y']}
        
        cmd = Twist()
        cmd.linear.x = speed
        
        while True:
            self.cmd_pub.publish(cmd)
            
            # Calculate distance traveled
            dx = self.position['x'] - start_pos['x']
            dy = self.position['y'] - start_pos['y']
            traveled = math.sqrt(dx**2 + dy**2)
            
            if traveled >= distance:
                break
            
            time.sleep(0.1)
        
        # Stop
        cmd.linear.x = 0.0
        self.cmd_pub.publish(cmd)
        time.sleep(0.5)
        
        # Record position
        self.path_points.append({'x': self.position['x'], 'y': self.position['y']})
        print(f"Position: ({self.position['x']:.2f}, {self.position['y']:.2f})")
        
    def turn_left_90(self, angular_speed=0.5):
        """Turn left 90 degrees"""
        start_orientation = self.orientation
        target_change = math.pi / 2  # 90 degrees in radians
        
        cmd = Twist()
        cmd.angular.z = angular_speed
        
        while True:
            self.cmd_pub.publish(cmd)
            
            # Calculate angle turned (handle wrap-around)
            angle_diff = self.orientation - start_orientation
            if angle_diff < -math.pi:
                angle_diff += 2 * math.pi
            elif angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            
            if angle_diff >= target_change:
                break
            
            time.sleep(0.1)
        
        # Stop
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)
        time.sleep(0.5)
        print(f"Orientation: {math.degrees(self.orientation):.1f}°")

def test_square_navigation():
    """Test 6: Navigate robot in 1-meter square"""
    
    # Cleanup first
    print("Cleaning up any existing processes...")
    subprocess.run(['./b4m_shutdown.sh', '--keep-agent'], capture_output=True)
    time.sleep(2)
    
    # Launch simulation with GUI for visual demonstration
    print("Launching simulation with GUI (auto-start)...")
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
        node = SquareNavigationNode()
        
        # Create executor in thread
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(node)
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()
        
        # Wait for odometry
        time.sleep(2)
        
        print("\nStarting 1-meter square navigation:")
        print("Initial position:", f"({node.position['x']:.2f}, {node.position['y']:.2f})")
        
        # Navigate in a square: 4 sides of 1 meter each (slower for visual demonstration)
        for side in range(4):
            print(f"\n--- Side {side + 1} ---")
            node.move_forward(1.0, speed=0.15)  # Slower for better visibility
            if side < 3:  # Don't turn after the last side
                node.turn_left_90(angular_speed=0.3)  # Slower turning
        
        print("\nNavigation complete!")
        
        # Verify we returned close to start
        final_x = node.position['x']
        final_y = node.position['y']
        distance_from_start = math.sqrt(final_x**2 + final_y**2)
        
        print(f"\nFinal position: ({final_x:.2f}, {final_y:.2f})")
        print(f"Distance from start: {distance_from_start:.3f}m")
        
        # Verify path forms a square
        if len(node.path_points) >= 4:
            # Calculate side lengths
            side_lengths = []
            for i in range(4):
                p1 = node.path_points[i]
                p2 = node.path_points[(i + 1) % 4]
                length = math.sqrt((p2['x'] - p1['x'])**2 + (p2['y'] - p1['y'])**2)
                side_lengths.append(length)
                print(f"Side {i+1} length: {length:.3f}m")
            
            # Check if all sides are close to 1 meter
            avg_length = sum(side_lengths) / 4
            max_deviation = max(abs(l - 1.0) for l in side_lengths)
            
            print(f"\nAverage side length: {avg_length:.3f}m")
            print(f"Maximum deviation from 1m: {max_deviation:.3f}m")
            
            # Success criteria
            if distance_from_start < 0.2 and max_deviation < 0.15:
                print("\n✓ Square navigation successful!")
                print("Test result: PASSED")
                return 0
            else:
                print("\n✗ Square navigation failed accuracy requirements")
                print("Test result: FAILED")
                return 1
        else:
            print("\n✗ Insufficient path points recorded")
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
    sys.exit(test_square_navigation())