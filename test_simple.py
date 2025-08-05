#!/usr/bin/env python3
# test_simple.py - Simplified integrated test

import subprocess
import time
import sys
import os

def run_simple_test():
    """Simple integrated test that launches and verifies the robot model"""
    
    print("=== Simplified Ignition Gazebo Model Test ===")
    
    # Clean up first
    print("\n1. Cleaning up existing processes...")
    subprocess.run(['./b4m_shutdown.sh', '--keep-agent'], capture_output=True)
    subprocess.run(['pkill', '-f', 'ign gazebo'], capture_output=True)
    subprocess.run(['pkill', '-f', 'controller_manager'], capture_output=True)
    time.sleep(2)
    
    gazebo_proc = None
    launch_proc = None
    
    try:
        # Launch Ignition Gazebo with auto-start
        print("\n2. Launching Ignition Gazebo with auto-start...")
        gazebo_proc = subprocess.Popen([
            'ros2', 'launch', 'yahboomcar_nav', 'ignition_gazebo_launch.py'
        ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        print("   (Simulation should start automatically - no play button needed)")
        time.sleep(5)
        
        # Check if Gazebo is running
        gazebo_check = subprocess.run(['pgrep', '-f', 'ign gazebo'], capture_output=True)
        if gazebo_check.returncode != 0:
            print("✗ Failed to launch Ignition Gazebo")
            return 1
        print("✓ Ignition Gazebo launched successfully")
        
        # Launch robot with controllers
        print("\n3. Spawning robot with controllers...")
        launch_proc = subprocess.Popen([
            'ros2', 'launch', 'yahboomcar_nav', 'spawn_robot_with_controllers_ignition.py'
        ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        
        # Wait for everything to initialize
        print("   Waiting for initialization...")
        time.sleep(15)
        
        # Check for robot state publisher
        print("\n4. Verifying robot components...")
        robot_state_check = subprocess.run(['pgrep', '-f', 'robot_state_publisher'], capture_output=True)
        if robot_state_check.returncode == 0:
            print("✓ Robot state publisher running")
        else:
            print("✗ Robot state publisher not found")
            return 1
        
        # Check for controllers
        controller_check = subprocess.run(['pgrep', '-f', 'controller_manager'], capture_output=True)
        if controller_check.returncode == 0:
            print("✓ Controller manager running")
        else:
            print("✗ Controller manager not found")
            return 1
        
        # Check ROS2 topics
        print("\n5. Checking ROS2 topics...")
        topics = subprocess.run(['ros2', 'topic', 'list'], capture_output=True, text=True).stdout
        
        required_topics = ['/cmd_vel', '/robot_description']
        missing_topics = []
        for topic in required_topics:
            if topic in topics:
                print(f"✓ Found topic: {topic}")
            else:
                missing_topics.append(topic)
                print(f"✗ Missing topic: {topic}")
        
        if missing_topics:
            print("\nMissing required topics:", missing_topics)
            return 1
        
        print("\n=== All checks passed! ===")
        print("The robot model is loaded correctly in Ignition Gazebo")
        print("- Simulation is running")
        print("- Robot components are active")
        print("- Required topics are available")
        return 0
        
    except Exception as e:
        print(f"\nError during test: {e}")
        return 1
    finally:
        # Cleanup
        print("\nCleaning up...")
        for proc in [launch_proc, gazebo_proc]:
            if proc and proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill()
        
        subprocess.run(['pkill', '-f', 'ign gazebo'], capture_output=True)
        subprocess.run(['pkill', '-f', 'controller_manager'], capture_output=True)
        subprocess.run(['pkill', '-f', 'robot_state_publisher'], capture_output=True)
        subprocess.run(['pkill', '-f', 'spawner'], capture_output=True)
        time.sleep(2)

if __name__ == "__main__":
    sys.exit(run_simple_test())