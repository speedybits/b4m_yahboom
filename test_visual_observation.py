#!/usr/bin/env python3
# test_visual_observation.py - Manual observation test

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import subprocess
import sys
import threading

class VisualTestNode(Node):
    def __init__(self):
        super().__init__('visual_test')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

def test_visual_observation():
    """Test that requires human observation of actual robot behavior"""
    
    print("=== VISUAL OBSERVATION TEST ===")
    print("This test requires you to observe the robot in Gazebo GUI")
    print("and confirm if it actually turns or just moves forward")
    
    # Cleanup first
    subprocess.run(['./b4m_shutdown.sh', '--keep-agent'], capture_output=True)
    time.sleep(2)
    
    # Launch simulation
    print("\nLaunching Gazebo GUI...")
    gazebo_proc = subprocess.Popen([
        'ros2', 'launch', 'yahboomcar_nav', 'gazebo_classic_nav_launch.py'
    ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    
    # Robot spawning is now integrated into gazebo_classic_nav_launch.py
    sim_proc = None
    
    try:
        print("Waiting for system initialization...")
        time.sleep(10)
        
        rclpy.init()
        node = VisualTestNode()
        
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(node)
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()
        
        time.sleep(2)
        
        print("\n" + "="*60)
        print("WATCH THE GAZEBO GUI CAREFULLY!")
        print("="*60)
        
        # Test 1: Forward motion
        print("\n1. FORWARD MOTION TEST (5 seconds)")
        print("   → Robot should move forward in a straight line")
        print("   → WATCH: Does the robot move forward? (Should be YES)")
        
        cmd = Twist()
        cmd.linear.x = 0.2
        
        for i in range(50):  # 5 seconds
            node.cmd_pub.publish(cmd)
            time.sleep(0.1)
        
        cmd.linear.x = 0.0
        node.cmd_pub.publish(cmd)
        time.sleep(2)
        
        print("   ✓ Forward motion complete")
        
        # Test 2: Pure rotation
        print("\n2. ROTATION TEST (10 seconds)")
        print("   → Robot should rotate in place (turn left)")
        print("   → WATCH: Does the robot body actually rotate? (Currently NO)")
        print("   → Note: Wheels may spin but robot body stays facing same direction")
        
        cmd = Twist()
        cmd.angular.z = 0.5  # Turn left
        
        for i in range(100):  # 10 seconds
            node.cmd_pub.publish(cmd)
            if i % 10 == 0:
                print(f"     {i//10 + 1}/10 - Is robot rotating?")
            time.sleep(0.1)
        
        cmd.angular.z = 0.0
        node.cmd_pub.publish(cmd)
        time.sleep(2)
        
        print("   ✓ Rotation test complete")
        
        # Test 3: Square path attempt
        print("\n3. SQUARE PATH TEST")
        print("   → Attempting 1-meter square navigation")
        print("   → WATCH: Does robot trace square path or just go straight?")
        
        for side in range(4):
            print(f"\n   Side {side + 1}: Moving forward 1 meter...")
            
            # Move forward
            cmd = Twist()
            cmd.linear.x = 0.2
            for i in range(50):  # ~5 seconds = ~1 meter
                node.cmd_pub.publish(cmd)
                time.sleep(0.1)
            
            cmd.linear.x = 0.0
            node.cmd_pub.publish(cmd)
            time.sleep(1)
            
            if side < 3:
                print(f"   Turn {side + 1}: Attempting 90° left turn...")
                
                # Attempt turn
                cmd = Twist()
                cmd.angular.z = 0.5
                for i in range(30):  # 3 seconds
                    node.cmd_pub.publish(cmd)
                    time.sleep(0.1)
                
                cmd.angular.z = 0.0
                node.cmd_pub.publish(cmd)
                time.sleep(1)
        
        print("\n" + "="*60)
        print("OBSERVATION QUESTIONS:")
        print("="*60)
        print("1. Did the robot move forward during forward commands? (Expected: YES)")
        print("2. Did the robot body rotate during rotation commands? (Should be: YES)")
        print("3. Did the robot trace a square path? (Should be: YES - complete square)")
        print("4. Do you see the wheels spinning during rotation? (Expected: YES)")
        print()
        print("DIAGNOSIS:")
        print("- If wheels spin but robot doesn't turn → Wheel-ground friction issue")
        print("- If robot only goes straight → Differential drive broken")
        print("- If robot traces perfect square → Everything working!")
        print("="*60)
        
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
    sys.exit(test_visual_observation())