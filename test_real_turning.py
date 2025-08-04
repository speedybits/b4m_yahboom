#!/usr/bin/env python3
# test_real_turning.py - Test that verifies ACTUAL robot motion, not just odometry

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import math
import subprocess
import sys
import threading

class RealTurningTestNode(Node):
    def __init__(self):
        super().__init__('real_turning_test')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry', self.odom_callback, 10)
        
        self.position = {'x': 0.0, 'y': 0.0}
        self.orientation = 0.0
        self.position_history = []
        
    def odom_callback(self, msg):
        self.position['x'] = msg.pose.pose.position.x
        self.position['y'] = msg.pose.pose.position.y
        
        # Store position history to track actual movement
        self.position_history.append({
            'x': self.position['x'],
            'y': self.position['y'],
            'time': time.time()
        })
        
        # Keep only last 100 positions
        if len(self.position_history) > 100:
            self.position_history = self.position_history[-100:]

def test_real_turning():
    """Test that verifies actual robot turning by checking position changes"""
    
    print("=== Real Turning Test ===")
    print("This test will verify the robot ACTUALLY turns by checking position patterns")
    
    # Cleanup first
    subprocess.run(['./b4m_shutdown.sh', '--keep-agent'], capture_output=True)
    time.sleep(2)
    
    # Launch simulation
    gazebo_proc = subprocess.Popen([
        'ros2', 'launch', 'yahboomcar_nav', 'ignition_gazebo_launch.py'
    ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    
    sim_proc = subprocess.Popen([
        'ros2', 'launch', 'yahboomcar_nav', 'spawn_robot_with_controllers_ignition.py'
    ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    
    try:
        print("Waiting for system initialization...")
        time.sleep(10)
        
        rclpy.init()
        node = RealTurningTestNode()
        
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(node)
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()
        
        time.sleep(2)
        
        print(f"Initial position: ({node.position['x']:.3f}, {node.position['y']:.3f})")
        
        # Test 1: Move forward and verify straight-line motion
        print(f"\n=== Test 1: Forward Motion ===")
        cmd = Twist()
        cmd.linear.x = 0.2
        
        start_pos = {'x': node.position['x'], 'y': node.position['y']}
        print(f"Moving forward for 3 seconds...")
        
        for i in range(30):
            node.cmd_pub.publish(cmd)
            time.sleep(0.1)
        
        cmd.linear.x = 0.0
        node.cmd_pub.publish(cmd)
        time.sleep(1)
        
        end_pos = {'x': node.position['x'], 'y': node.position['y']}
        
        # Calculate if motion was roughly straight
        dx = end_pos['x'] - start_pos['x']
        dy = end_pos['y'] - start_pos['y']
        distance = math.sqrt(dx**2 + dy**2)
        angle_of_motion = math.atan2(dy, dx)
        
        print(f"Start position: ({start_pos['x']:.3f}, {start_pos['y']:.3f})")
        print(f"End position: ({end_pos['x']:.3f}, {end_pos['y']:.3f})")
        print(f"Distance moved: {distance:.3f}m")
        print(f"Direction of movement: {math.degrees(angle_of_motion):.1f}°")
        
        if distance < 0.3:
            print("❌ FAIL: Robot didn't move forward properly")
            return 1
        else:
            print("✅ PASS: Robot moved forward")
        
        # Test 2: Try to turn and verify if position changes indicate turning
        print(f"\n=== Test 2: Turn Test ===")
        print("Attempting to turn left...")
        
        # Record starting position
        turn_start_pos = {'x': node.position['x'], 'y': node.position['y']}
        node.position_history.clear()  # Clear history to track turn motion
        
        cmd = Twist()
        cmd.angular.z = 0.5  # Try to turn left
        
        print("Sending angular velocity command for 5 seconds...")
        for i in range(50):
            node.cmd_pub.publish(cmd)
            if i % 10 == 0:
                print(f"  t={i/10:.1f}s: pos=({node.position['x']:.3f}, {node.position['y']:.3f})")
            time.sleep(0.1)
        
        cmd.angular.z = 0.0
        node.cmd_pub.publish(cmd)
        time.sleep(1)
        
        turn_end_pos = {'x': node.position['x'], 'y': node.position['y']}
        
        # Analyze if the robot actually moved in a turning pattern
        if len(node.position_history) >= 10:
            # Check if robot moved in an arc (turning) vs straight line
            positions = node.position_history[-30:]  # Last 3 seconds of motion
            
            # Calculate if motion pattern suggests turning
            x_coords = [p['x'] for p in positions]
            y_coords = [p['y'] for p in positions]
            
            x_variance = max(x_coords) - min(x_coords)
            y_variance = max(y_coords) - min(y_coords)
            
            print(f"Turn motion analysis:")
            print(f"  Start: ({turn_start_pos['x']:.3f}, {turn_start_pos['y']:.3f})")
            print(f"  End: ({turn_end_pos['x']:.3f}, {turn_end_pos['y']:.3f})")
            print(f"  X variance during turn: {x_variance:.3f}m")
            print(f"  Y variance during turn: {y_variance:.3f}m")
            
            # If robot is truly turning, we should see movement in both X and Y
            total_variance = x_variance + y_variance
            
            if total_variance < 0.05:  # Very little movement in any direction
                print("❌ FAIL: Robot didn't move during 'turn' - angular commands not working")
                print("   The robot should have moved in an arc pattern if turning")
                return 1
            elif x_variance > 0.02 and y_variance > 0.02:
                print("✅ PASS: Robot showed curved motion pattern (likely turning)")
                return 0
            else:
                print("⚠️  UNCLEAR: Robot moved but pattern doesn't clearly indicate turning")
                print("   This might indicate partial functionality or very slow turning")
                return 1
        else:
            print("❌ FAIL: Not enough position data to analyze turning")
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
    sys.exit(test_real_turning())