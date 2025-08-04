#!/usr/bin/env python3
# test_gazebo_position.py - Test using actual Gazebo robot position, not fake odometry

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math
import subprocess
import sys
import threading
import json

class GazeboPositionTestNode(Node):
    def __init__(self):
        super().__init__('gazebo_position_test')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
    def get_gazebo_pose(self):
        """Get robot pose directly from Ignition Gazebo"""
        try:
            # Get pose from Gazebo service
            result = subprocess.run([
                'ign', 'service', '-s', '/world/empty_world/entity/system/add',
                '--reqtype', 'ignition.msgs.EntityPlugin_V',
                '--reptype', 'ignition.msgs.Boolean',
                '--timeout', '1000'
            ], capture_output=True, text=True, timeout=3)
            
            # Alternative: Get model info 
            model_info = subprocess.run([
                'ign', 'model', '-m', 'yahboomcar', '-i'
            ], capture_output=True, text=True, timeout=3)
            
            if model_info.returncode == 0:
                # Parse pose from model info output
                lines = model_info.stdout.split('\n')
                for line in lines:
                    if 'Pose:' in line:
                        # Extract position from pose line
                        pose_str = line.split('Pose:')[1].strip()
                        # Parse "x y z roll pitch yaw" format
                        values = pose_str.split()
                        if len(values) >= 6:
                            return {
                                'x': float(values[0]),
                                'y': float(values[1]),
                                'z': float(values[2]),
                                'yaw': float(values[5])
                            }
        except:
            pass
            
        return None
        
    def move_forward(self, distance, speed=0.15):
        """Move forward and track using Gazebo position"""
        print(f"    Moving forward {distance}m at {speed} m/s")
        
        start_pose = self.get_gazebo_pose()
        if not start_pose:
            print("    ❌ Could not get initial Gazebo pose")
            return False
            
        print(f"    Start pose: ({start_pose['x']:.3f}, {start_pose['y']:.3f})")
        
        cmd = Twist()
        cmd.linear.x = speed
        
        step_count = 0
        while step_count < 80:  # Max 8 seconds
            self.cmd_pub.publish(cmd)
            
            current_pose = self.get_gazebo_pose()
            if current_pose:
                dx = current_pose['x'] - start_pose['x']
                dy = current_pose['y'] - start_pose['y']
                traveled = math.sqrt(dx**2 + dy**2)
                
                if step_count % 10 == 0:  # Print every second
                    print(f"      t={step_count/10:.1f}s: traveled={traveled:.3f}m, "
                          f"pos=({current_pose['x']:.3f}, {current_pose['y']:.3f})")
                
                if traveled >= distance:
                    break
            
            step_count += 1
            time.sleep(0.1)
        
        # Stop
        cmd.linear.x = 0.0
        self.cmd_pub.publish(cmd)
        time.sleep(0.5)
        
        final_pose = self.get_gazebo_pose()
        if final_pose:
            print(f"    Final pose: ({final_pose['x']:.3f}, {final_pose['y']:.3f})")
            return True
        return False
        
    def turn_left_90(self, angular_speed=0.3):
        """Turn left 90 degrees using Gazebo position"""
        print(f"    Turning left 90° at {angular_speed} rad/s")
        
        start_pose = self.get_gazebo_pose()
        if not start_pose:
            print("    ❌ Could not get initial Gazebo pose")
            return False
            
        start_yaw = start_pose['yaw']
        target_yaw = start_yaw + math.pi/2  # 90 degrees
        
        print(f"    Start yaw: {math.degrees(start_yaw):.1f}°")
        print(f"    Target yaw: {math.degrees(target_yaw):.1f}°")
        
        cmd = Twist()
        cmd.angular.z = angular_speed
        
        step_count = 0
        while step_count < 200:  # Max 20 seconds
            self.cmd_pub.publish(cmd)
            
            current_pose = self.get_gazebo_pose()
            if current_pose:
                current_yaw = current_pose['yaw']
                
                # Handle angle wrapping
                yaw_diff = current_yaw - start_yaw
                if yaw_diff < -math.pi:
                    yaw_diff += 2*math.pi
                elif yaw_diff > math.pi:
                    yaw_diff -= 2*math.pi
                
                if step_count % 10 == 0:  # Print every second
                    print(f"      t={step_count/10:.1f}s: turned={math.degrees(yaw_diff):.1f}°, "
                          f"current_yaw={math.degrees(current_yaw):.1f}°, "
                          f"pos=({current_pose['x']:.3f}, {current_pose['y']:.3f})")
                
                if abs(yaw_diff) >= math.pi/2 - 0.1:  # Close to 90 degrees
                    print(f"    ✅ Turn complete: {math.degrees(yaw_diff):.1f}°")
                    break
            
            step_count += 1
            time.sleep(0.1)
        
        if step_count >= 200:
            print("    ❌ Turn timeout - robot may not be turning")
            return False
        
        # Stop
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)
        time.sleep(0.5)
        
        return True

def test_gazebo_square_navigation():
    """Test square navigation using actual Gazebo poses"""
    
    print("=== Gazebo Position Square Navigation Test ===")
    print("This test uses ACTUAL robot poses from Gazebo, not odometry")
    
    # Cleanup first
    subprocess.run(['./b4m_shutdown.sh', '--keep-agent'], capture_output=True)
    time.sleep(2)
    
    # Launch simulation
    print("Launching simulation...")
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
        node = GazeboPositionTestNode()
        
        # Create executor in thread
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(node)
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()
        
        time.sleep(2)
        
        # Get initial pose from Gazebo
        initial_pose = node.get_gazebo_pose()
        if not initial_pose:
            print("❌ Could not get robot pose from Gazebo")
            return 1
            
        print(f"\nInitial Gazebo pose: ({initial_pose['x']:.3f}, {initial_pose['y']:.3f}, yaw={math.degrees(initial_pose['yaw']):.1f}°)")
        
        path_poses = [initial_pose]
        
        # Navigate in a square: 4 sides of 1 meter each
        for side in range(4):
            print(f"\n--- Side {side + 1} ---")
            if not node.move_forward(1.0, speed=0.15):
                print("❌ Forward motion failed")
                return 1
                
            current_pose = node.get_gazebo_pose()
            if current_pose:
                path_poses.append(current_pose)
            
            if side < 3:  # Don't turn after the last side
                print(f"\n--- Turn {side + 1} ---")
                if not node.turn_left_90(angular_speed=0.3):
                    print("❌ Turn failed")
                    return 1
        
        # Analyze the actual path from Gazebo
        print(f"\n=== GAZEBO PATH ANALYSIS ===")
        for i, pose in enumerate(path_poses):
            print(f"Point {i}: ({pose['x']:.3f}, {pose['y']:.3f}) yaw={math.degrees(pose['yaw']):.1f}°")
        
        if len(path_poses) >= 5:
            # Calculate side lengths from actual Gazebo positions
            side_lengths = []
            for i in range(4):
                p1 = path_poses[i]
                p2 = path_poses[i+1] 
                length = math.sqrt((p2['x'] - p1['x'])**2 + (p2['y'] - p1['y'])**2)
                side_lengths.append(length)
                print(f"Side {i+1} actual length: {length:.3f}m")
            
            # Check if we returned to start
            final_pose = path_poses[-1]
            distance_from_start = math.sqrt(final_pose['x']**2 + final_pose['y']**2)
            
            print(f"\nFinal Gazebo pose: ({final_pose['x']:.3f}, {final_pose['y']:.3f})")
            print(f"Distance from start: {distance_from_start:.3f}m")
            
            # Test criteria
            avg_length = sum(side_lengths) / 4
            max_deviation = max(abs(l - 1.0) for l in side_lengths)
            
            print(f"Average side length: {avg_length:.3f}m")
            print(f"Maximum deviation from 1m: {max_deviation:.3f}m")
            
            if distance_from_start < 0.2 and max_deviation < 0.15:
                print("\n✅ GAZEBO SQUARE NAVIGATION SUCCESSFUL!")
                print("Test result: PASSED")
                return 0
            else:
                print("\n❌ Square navigation failed accuracy requirements")
                print("Test result: FAILED")
                return 1
        else:
            print("❌ Insufficient pose data")
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
    sys.exit(test_gazebo_square_navigation())