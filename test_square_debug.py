#!/usr/bin/env python3
# test_square_debug.py - Detailed debugging version

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import math
import subprocess
import sys
import threading

class SquareDebugNode(Node):
    def __init__(self):
        super().__init__('square_debug_test')
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
        
    def move_forward(self, distance, speed=0.15):
        """Move forward a specified distance with detailed logging"""
        print(f"    Starting forward motion: {distance}m at {speed} m/s")
        start_pos = {'x': self.position['x'], 'y': self.position['y']}
        
        cmd = Twist()
        cmd.linear.x = speed
        
        step_count = 0
        while True:
            self.cmd_pub.publish(cmd)
            
            # Calculate distance traveled
            dx = self.position['x'] - start_pos['x']
            dy = self.position['y'] - start_pos['y']
            traveled = math.sqrt(dx**2 + dy**2)
            
            step_count += 1
            if step_count % 10 == 0:  # Print every second
                print(f"      t={step_count/10:.1f}s: traveled={traveled:.3f}m, "
                      f"pos=({self.position['x']:.3f}, {self.position['y']:.3f})")
            
            if traveled >= distance:
                break
            
            time.sleep(0.1)
        
        # Stop
        cmd.linear.x = 0.0
        self.cmd_pub.publish(cmd)
        time.sleep(0.5)
        
        # Record position
        self.path_points.append({'x': self.position['x'], 'y': self.position['y']})
        print(f"    Forward motion complete: final pos=({self.position['x']:.3f}, {self.position['y']:.3f})")
        
    def turn_left_90_debug(self, angular_speed=0.3):
        """Turn left 90 degrees with detailed debugging"""
        print(f"    Starting 90° left turn at {angular_speed} rad/s")
        start_orientation = self.orientation
        target_change = math.pi / 2  # 90 degrees in radians
        
        print(f"      Initial orientation: {math.degrees(start_orientation):.1f}°")
        print(f"      Target change: {math.degrees(target_change):.1f}°")
        
        cmd = Twist()
        cmd.angular.z = angular_speed
        
        step_count = 0
        max_steps = 200  # Safety limit (20 seconds at 0.1s per step)
        
        while step_count < max_steps:
            self.cmd_pub.publish(cmd)
            
            # Calculate angle turned (handle wrap-around)
            angle_diff = self.orientation - start_orientation
            if angle_diff < -math.pi:
                angle_diff += 2 * math.pi
            elif angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            
            step_count += 1
            if step_count % 10 == 0:  # Print every second
                print(f"      t={step_count/10:.1f}s: turned={math.degrees(angle_diff):.1f}°, "
                      f"current_orientation={math.degrees(self.orientation):.1f}°")
            
            if angle_diff >= target_change:
                print(f"      Turn complete! Final turn: {math.degrees(angle_diff):.1f}°")
                break
            
            time.sleep(0.1)
        
        if step_count >= max_steps:
            print(f"      WARNING: Turn timeout! Only turned {math.degrees(angle_diff):.1f}°")
        
        # Stop
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)
        time.sleep(0.5)
        
        final_orientation = self.orientation
        total_turn = final_orientation - start_orientation
        if total_turn < -math.pi:
            total_turn += 2 * math.pi
        elif total_turn > math.pi:
            total_turn -= 2 * math.pi
            
        print(f"    Turn complete: final orientation={math.degrees(final_orientation):.1f}°, "
              f"total_turn={math.degrees(total_turn):.1f}°")

def test_square_debug():
    """Debug version of square navigation"""
    
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
        node = SquareDebugNode()
        
        # Create executor in thread
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(node)
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()
        
        # Wait for odometry
        time.sleep(2)
        
        print(f"\n=== Starting Square Navigation Debug ===")
        print(f"Initial position: ({node.position['x']:.3f}, {node.position['y']:.3f})")
        print(f"Initial orientation: {math.degrees(node.orientation):.1f}°")
        
        # Navigate in a square: 4 sides of 1 meter each
        for side in range(4):
            print(f"\n--- Side {side + 1} ---")
            node.move_forward(1.0, speed=0.15)
            
            if side < 3:  # Don't turn after the last side
                print(f"\n--- Turn {side + 1} ---")
                node.turn_left_90_debug(angular_speed=0.3)
        
        print(f"\n=== Navigation Complete ===")
        final_x = node.position['x']
        final_y = node.position['y']
        distance_from_start = math.sqrt(final_x**2 + final_y**2)
        
        print(f"Final position: ({final_x:.3f}, {final_y:.3f})")
        print(f"Distance from start: {distance_from_start:.3f}m")
        print(f"Final orientation: {math.degrees(node.orientation):.1f}°")
        
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
    sys.exit(test_square_debug())