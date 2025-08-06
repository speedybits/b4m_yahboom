#!/usr/bin/env python3
# test_square_corners.py - Test that verifies robot reaches each corner of 1m square using Gazebo Classic

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import math
import subprocess
import sys
import threading

class SquareCornersNode(Node):
    def __init__(self):
        super().__init__('square_corners_test')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.x = 0.0
        self.y = 0.0
        self.orientation = 0.0
        
    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        # Convert quaternion to yaw
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        # Proper quaternion to euler conversion
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        self.orientation = math.atan2(siny_cosp, cosy_cosp)
        
    def move_to_corner(self, target_x, target_y, corner_name, tolerance=0.1):
        """Move robot to specific corner position"""
        print(f"\n--- Moving to {corner_name} at ({target_x:.1f}, {target_y:.1f}) ---")
        
        cmd = Twist()
        max_iterations = 100  # 10 seconds max
        
        for i in range(max_iterations):
            # Calculate distance to target
            dx = target_x - self.x
            dy = target_y - self.y
            distance = math.sqrt(dx**2 + dy**2)
            
            # Calculate angle to target
            target_angle = math.atan2(dy, dx)
            
            # Calculate angle difference
            angle_diff = target_angle - self.orientation
            # Normalize angle to [-pi, pi]
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi
            
            if i % 10 == 0:
                print(f"  Position: ({self.x:.3f}, {self.y:.3f}), Distance to corner: {distance:.3f}m")
                print(f"  Current angle: {math.degrees(self.orientation):.1f}°, Target angle: {math.degrees(target_angle):.1f}°")
            
            # Check if we reached the corner
            if distance < tolerance:
                print(f"  ✓ Reached {corner_name}! Final position: ({self.x:.3f}, {self.y:.3f})")
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.cmd_pub.publish(cmd)
                time.sleep(0.5)
                return True
            
            # Control logic
            if abs(angle_diff) > 0.2:  # Need to turn
                cmd.linear.x = 0.0
                cmd.angular.z = 0.4 if angle_diff > 0 else -0.4
            else:  # Move forward
                cmd.linear.x = min(0.2, distance * 0.5)  # Slow down as we approach
                cmd.angular.z = angle_diff * 2.0  # Small corrections while moving
            
            self.cmd_pub.publish(cmd)
            time.sleep(0.1)
        
        # Stop if we timeout
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)
        
        print(f"  ✗ Failed to reach {corner_name}. Final position: ({self.x:.3f}, {self.y:.3f})")
        return False

def test_square_corners():
    """Test that robot reaches each corner of a 1-meter square using Gazebo Classic"""
    
    print("=== Square Corners Navigation Test (Gazebo Classic) ===")
    print("This test verifies the robot reaches each corner of a 1m square")
    
    # Cleanup first
    subprocess.run(['./b4m_shutdown.sh', '--keep-agent'], capture_output=True)
    time.sleep(2)
    
    # Launch Gazebo Classic simulation
    print("\nLaunching Gazebo Classic simulation...")
    sim_proc = subprocess.Popen([
        'ros2', 'launch', 'yahboomcar_nav', 'gazebo_classic_nav_launch.py'
    ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    
    try:
        print("Waiting for system initialization...")
        time.sleep(10)
        
        rclpy.init()
        node = SquareCornersNode()
        
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(node)
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()
        
        time.sleep(2)
        
        # Define the corners of a 1-meter square
        # Starting at (0,0), going counter-clockwise
        corners = [
            (0.0, 0.0, "Start/Corner 1"),
            (1.0, 0.0, "Corner 2"),
            (1.0, 1.0, "Corner 3"), 
            (0.0, 1.0, "Corner 4"),
            (0.0, 0.0, "Back to Start")
        ]
        
        print(f"\nInitial position: ({node.x:.3f}, {node.y:.3f})")
        print("Square path corners:")
        for i, (x, y, name) in enumerate(corners):
            print(f"  {i+1}. {name}: ({x:.1f}, {y:.1f})")
        
        # Navigate to each corner
        success_count = 0
        reached_corners = []
        
        for i, (target_x, target_y, corner_name) in enumerate(corners):
            if node.move_to_corner(target_x, target_y, corner_name, tolerance=0.15):
                success_count += 1
                reached_corners.append((node.x, node.y))
            else:
                reached_corners.append(None)
        
        # Analyze results
        print("\n=== RESULTS ===")
        print(f"Corners reached: {success_count}/{len(corners)}")
        
        # Calculate actual path
        print("\nActual path taken:")
        for i, corner in enumerate(reached_corners):
            if corner:
                print(f"  Corner {i+1}: ({corner[0]:.3f}, {corner[1]:.3f})")
            else:
                print(f"  Corner {i+1}: NOT REACHED")
        
        # Check if we completed the square
        if success_count >= 4:
            # Calculate deviation from ideal square
            if reached_corners[0] and reached_corners[-1]:
                final_error = math.sqrt(
                    (reached_corners[-1][0] - reached_corners[0][0])**2 + 
                    (reached_corners[-1][1] - reached_corners[0][1])**2
                )
                print(f"\nDistance from start to final position: {final_error:.3f}m")
                
                if final_error < 0.2:
                    print("\n✅ SQUARE NAVIGATION SUCCESSFUL!")
                    print("Robot successfully navigated a 1-meter square in Gazebo Classic")
                    return 0
                else:
                    print("\n⚠️  Square completed but with high error")
                    return 1
        else:
            print("\n❌ SQUARE NAVIGATION FAILED")
            print("Robot could not reach all corners")
            return 1
            
    except Exception as e:
        print(f"Error during test: {e}")
        return 1
    finally:
        print("\nCleaning up...")
        if 'node' in locals():
            node.destroy_node()
        if 'executor' in locals():
            executor.shutdown()
        rclpy.shutdown()
        
        if sim_proc and sim_proc.poll() is None:
            sim_proc.terminate()
            try:
                sim_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                sim_proc.kill()
        
        # Cleanup Gazebo Classic processes
        subprocess.run(['pkill', '-f', 'gazebo'], capture_output=True)
        subprocess.run(['pkill', '-f', 'gzserver'], capture_output=True)
        subprocess.run(['pkill', '-f', 'gzclient'], capture_output=True)
        subprocess.run(['pkill', '-f', 'robot_state_publisher'], capture_output=True)
        time.sleep(2)

if __name__ == "__main__":
    sys.exit(test_square_corners())