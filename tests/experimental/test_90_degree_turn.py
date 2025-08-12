#!/usr/bin/env python3
# test_90_degree_turn.py - Test to calibrate 90-degree turns

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import math
import subprocess
import sys
import threading

class TurnCalibrationNode(Node):
    def __init__(self):
        super().__init__('turn_calibration_test')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odometry', self.odom_callback, 10)
        
        self.orientation = 0.0
        self.initial_orientation = None
        
    def odom_callback(self, msg):
        # Convert quaternion to yaw
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        # Proper quaternion to euler conversion
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        self.orientation = math.atan2(siny_cosp, cosy_cosp)
        
        if self.initial_orientation is None:
            self.initial_orientation = self.orientation
        
    def turn_90_degrees(self, direction='left', angular_speed=0.3):
        """Turn exactly 90 degrees and measure actual turn"""
        if self.initial_orientation is None:
            print("Waiting for odometry...")
            time.sleep(1)
            
        start_angle = self.orientation
        target_angle = start_angle + (math.pi/2 if direction == 'left' else -math.pi/2)
        
        print(f"\n--- 90° {direction} turn test ---")
        print(f"Start angle: {math.degrees(start_angle):.1f}°")
        print(f"Target angle: {math.degrees(target_angle):.1f}°")
        print(f"Angular speed: {angular_speed} rad/s")
        
        cmd = Twist()
        cmd.angular.z = angular_speed if direction == 'left' else -angular_speed
        
        # Track turning progress
        turn_start_time = time.time()
        last_print_time = 0
        
        while True:
            # Calculate how much we've turned
            current_turn = self.orientation - start_angle
            
            # Normalize angle difference
            while current_turn > math.pi:
                current_turn -= 2 * math.pi
            while current_turn < -math.pi:
                current_turn += 2 * math.pi
                
            # Adjust for direction
            if direction == 'right':
                current_turn = -current_turn
            
            # Print progress every 0.5 seconds
            current_time = time.time()
            if current_time - last_print_time > 0.5:
                print(f"  Turned: {math.degrees(current_turn):.1f}° / 90°")
                last_print_time = current_time
            
            # Check if we've turned enough
            if current_turn >= math.pi/2 - 0.05:  # Within 3 degrees
                break
                
            # Safety timeout
            if current_time - turn_start_time > 10:
                print("  ⚠️  Turn timeout!")
                break
                
            self.cmd_pub.publish(cmd)
            time.sleep(0.01)
        
        # Stop turning
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)
        
        # Wait for robot to settle
        time.sleep(0.5)
        
        # Calculate actual turn
        final_turn = self.orientation - start_angle
        while final_turn > math.pi:
            final_turn -= 2 * math.pi
        while final_turn < -math.pi:
            final_turn += 2 * math.pi
        
        if direction == 'right':
            final_turn = -final_turn
            
        turn_duration = time.time() - turn_start_time
        
        print(f"\n✓ Turn complete!")
        print(f"  Actual turn: {math.degrees(final_turn):.1f}°")
        print(f"  Error: {math.degrees(final_turn - math.pi/2):.1f}°")
        print(f"  Duration: {turn_duration:.2f} seconds")
        print(f"  Final angle: {math.degrees(self.orientation):.1f}°")
        
        return final_turn, turn_duration

def test_90_degree_turns():
    """Test and calibrate 90-degree turns"""
    
    print("=== 90-Degree Turn Calibration Test ===")
    print("This test calibrates the robot's turning to achieve accurate 90° turns")
    
    # Cleanup first
    subprocess.run(['./b4m_shutdown.sh', '--keep-agent'], capture_output=True)
    time.sleep(2)
    
    # Launch simulation
    print("\nLaunching simulation...")
    gazebo_proc = subprocess.Popen([
        'ros2', 'launch', 'yahboomcar_nav', 'gazebo_classic_nav_launch.py'
    ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    
    # Robot spawning is now integrated into gazebo_classic_nav_launch.py
    sim_proc = None
    
    try:
        print("Waiting for system initialization...")
        time.sleep(10)
        
        rclpy.init()
        node = TurnCalibrationNode()
        
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(node)
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()
        
        time.sleep(2)
        
        # Test different angular speeds
        test_speeds = [0.2, 0.3, 0.4, 0.5]
        results = []
        
        for speed in test_speeds:
            # Test left turn
            actual_turn, duration = node.turn_90_degrees('left', speed)
            results.append({
                'direction': 'left',
                'speed': speed,
                'actual_turn': actual_turn,
                'duration': duration,
                'error': math.degrees(actual_turn - math.pi/2)
            })
            
            time.sleep(2)
            
            # Test right turn (to return to original orientation)
            actual_turn, duration = node.turn_90_degrees('right', speed)
            results.append({
                'direction': 'right',
                'speed': speed,
                'actual_turn': actual_turn,
                'duration': duration,
                'error': math.degrees(actual_turn - math.pi/2)
            })
            
            time.sleep(2)
        
        # Analyze results
        print("\n=== CALIBRATION RESULTS ===")
        print("Speed (rad/s) | Direction | Actual Turn | Error    | Duration")
        print("-------------|-----------|-------------|----------|----------")
        
        best_speed = None
        best_error = float('inf')
        
        for r in results:
            print(f"{r['speed']:12.1f} | {r['direction']:9s} | "
                  f"{math.degrees(r['actual_turn']):10.1f}° | "
                  f"{r['error']:7.1f}° | {r['duration']:7.2f}s")
            
            if abs(r['error']) < abs(best_error):
                best_error = r['error']
                best_speed = r['speed']
        
        print(f"\n✓ Best angular speed: {best_speed} rad/s (error: {best_error:.1f}°)")
        
        # Calculate timing for 90-degree turn
        avg_duration = sum(r['duration'] for r in results if r['speed'] == best_speed) / 2
        print(f"✓ Recommended turn duration for 90°: {avg_duration:.2f} seconds")
        
        return 0
        
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
        
        for proc in [gazebo_proc]:
            if proc and proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill()
        
        subprocess.run(['pkill', '-f', 'gazebo'], capture_output=True)
        subprocess.run(['pkill', '-f', 'gzserver'], capture_output=True)
        subprocess.run(['pkill', '-f', 'gzclient'], capture_output=True)
        subprocess.run(['pkill', '-f', 'controller_manager'], capture_output=True)
        subprocess.run(['pkill', '-f', 'robot_state_publisher'], capture_output=True)
        time.sleep(2)

if __name__ == "__main__":
    sys.exit(test_90_degree_turns())