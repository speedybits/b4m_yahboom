#!/usr/bin/env python3
# test_simple_turn.py - Simple test to check if robot turns

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import subprocess
import sys
import threading

class SimpleTurnNode(Node):
    def __init__(self):
        super().__init__('simple_turn_test')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

def test_simple_turn():
    print("=== Simple Turn Test ===")
    print("Testing if robot can turn with new 4-wheel configuration")
    
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
        print("Waiting for initialization...")
        time.sleep(12)
        
        rclpy.init()
        node = SimpleTurnNode()
        
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(node)
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()
        
        time.sleep(2)
        
        print("\n=== Test 1: Forward Motion ===")
        cmd = Twist()
        cmd.linear.x = 0.2
        
        for i in range(30):  # 3 seconds
            node.cmd_pub.publish(cmd)
            if i % 10 == 0:
                print(f"Forward motion: {i//10 + 1}/3 seconds")
            time.sleep(0.1)
        
        cmd.linear.x = 0.0
        node.cmd_pub.publish(cmd)
        time.sleep(2)
        print("✓ Forward motion complete")
        
        print("\n=== Test 2: Turning Motion ===")
        print("Watch Gazebo GUI: Does robot actually rotate?")
        
        cmd = Twist()
        cmd.angular.z = 0.4  # Turn left
        
        for i in range(50):  # 5 seconds
            node.cmd_pub.publish(cmd)
            if i % 10 == 0:
                print(f"Turning: {i//10 + 1}/5 seconds - Is robot body rotating?")
            time.sleep(0.1)
        
        cmd.angular.z = 0.0
        node.cmd_pub.publish(cmd)
        time.sleep(1)
        print("✓ Turn command complete")
        
        print("\n=== Result ===")
        print("If robot body rotated during turning test:")
        print("  → SUCCESS: 4-wheel differential drive is working!")
        print("  → Square navigation should now work")
        print()
        print("If robot did NOT rotate:")
        print("  → FAILED: Still have physics/friction issues")
        print("  → Need to investigate further")
        
        return 0
        
    except Exception as e:
        print(f"Error: {e}")
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
        time.sleep(2)

if __name__ =="__main__":
    sys.exit(test_simple_turn())