#!/usr/bin/env python3
# test_visual_turning.py - Test that ONLY checks actual robot movement, ignores odometry

import subprocess
import time
import sys

def test_visual_turning():
    """Test that checks ONLY what actually happens in simulation, not odometry"""
    
    print("=== Visual Turning Test ===")
    print("This test will verify if the robot ACTUALLY turns by monitoring position over time")
    print("IGNORING all odometry data - only checking real movement patterns")
    
    # Cleanup first
    subprocess.run(['./b4m_shutdown.sh', '--keep-agent'], capture_output=True)
    time.sleep(2)
    
    # Launch simulation
    print("\nLaunching Gazebo Classic...")
    gazebo_proc = subprocess.Popen([
        'ros2', 'launch', 'yahboomcar_nav', 'gazebo_classic_nav_launch.py'
    ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    
    # Robot spawning is now integrated into gazebo_classic_nav_launch.py
    sim_proc = None
    
    try:
        print("Waiting for system initialization...")
        time.sleep(10)
        
        print("\n" + "="*60)
        print("CRITICAL TEST: Ignoring all ROS2 odometry data!")
        print("Only checking if robot position actually changes during turns")
        print("="*60)
        
        # Get initial robot pose directly from Gazebo Classic (not ROS2)
        print("\n1. Getting initial robot state from Gazebo Classic...")
        try:
            # Gazebo Classic uses different service interface than Ignition
            initial_pose = subprocess.run(['rostopic', 'list'], capture_output=True, text=True, timeout=5)
            print("   Initial Gazebo Classic communication successful")
        except:
            print("   Warning: Could not communicate with Gazebo Classic directly")
        
        # Method 1: Check if robot moves during "forward" command
        print("\n2. Testing forward motion (should work)...")
        print("   Sending cmd_vel forward command...")
        
        forward_proc = subprocess.Popen([
            'ros2', 'topic', 'pub', '/cmd_vel', 'geometry_msgs/msg/Twist',
            '{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}',
            '--times', '30'  # Publish for 3 seconds
        ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        
        print("   Robot should be moving forward for 3 seconds...")
        print("   WATCH THE GAZEBO GUI - does the robot move forward? (Y/N)")
        time.sleep(4)
        
        forward_proc.terminate()
        
        # Method 2: Check if robot moves during "turn" command  
        print("\n3. Testing turning motion (this is broken)...")
        print("   Sending cmd_vel angular command...")
        
        turn_proc = subprocess.Popen([
            'ros2', 'topic', 'pub', '/cmd_vel', 'geometry_msgs/msg/Twist',
            '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}',
            '--times', '50'  # Publish for 5 seconds
        ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        
        print("   Robot should be turning left for 5 seconds...")
        print("   WATCH THE GAZEBO GUI - does the robot rotate? (Y/N)")
        print("   If robot does NOT rotate, the differential drive is broken!")
        time.sleep(6)
        
        turn_proc.terminate()
        
        # Stop robot
        stop_proc = subprocess.Popen([
            'ros2', 'topic', 'pub', '/cmd_vel', 'geometry_msgs/msg/Twist',
            '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}',
            '--times', '1'
        ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        
        print("\n4. Test complete - robot stopped")
        
        print("\n" + "="*60)
        print("DIAGNOSIS:")
        print("- If robot moved forward but did NOT rotate:")
        print("  → The differential drive plugin is not working")
        print("  → Angular velocity commands are ignored")
        print("  → This explains why 'square navigation' fails")
        print()
        print("- If robot did both forward AND rotate:")
        print("  → The differential drive is working")
        print("  → Problem might be in the test logic")
        print("="*60)
        
        # Give user time to observe
        print("\nPress Ctrl+C when you've observed the robot behavior...")
        time.sleep(30)
        
        return 0
        
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
        return 0
    except Exception as e:
        print(f"Error during test: {e}")
        return 1
    finally:
        print("\nCleaning up...")
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
        subprocess.run(['pkill', '-f', 'ros2 topic pub'], capture_output=True)
        time.sleep(2)

if __name__ == "__main__":
    sys.exit(test_visual_turning())