#!/usr/bin/env python3
# test_minimal_turn.py - Minimal test to see robot turning behavior

import subprocess
import time
import sys

def test_minimal_turn():
    print("=== Minimal Turn Test ===")
    print("Launching Gazebo with robot, then sending single turn command")
    
    # Cleanup
    subprocess.run(['./b4m_shutdown.sh', '--keep-agent'], capture_output=True)
    time.sleep(2)
    
    # Launch Gazebo GUI
    print("1. Launching Gazebo GUI (auto-starts)...")
    gazebo_proc = subprocess.Popen([
        'ros2', 'launch', 'yahboomcar_nav', 'ignition_gazebo_launch.py'
    ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    
    time.sleep(8)
    
    # Launch robot spawner 
    print("2. Spawning robot in Gazebo...")
    sim_proc = subprocess.Popen([
        'ros2', 'launch', 'yahboomcar_nav', 'spawn_robot_with_controllers_ignition.py'
    ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    
    time.sleep(10)
    
    try:
        print("3. WATCH GAZEBO CAREFULLY!")
        print("   Robot should be visible in Gazebo")
        
        # Test forward
        print("\n4. Sending FORWARD command (3 seconds)...")
        print("   → Should see robot move forward")
        forward_cmd = subprocess.run([
            'ros2', 'topic', 'pub', '/cmd_vel', 'geometry_msgs/msg/Twist',
            '{linear: {x: 0.3}}', '--times', '30'
        ], capture_output=True, timeout=5)
        time.sleep(1)
        
        # Test turn  
        print("\n5. Sending TURN command (5 seconds)...")
        print("   → Should see robot ROTATE (not just wheels spinning)")
        print("   → If robot body doesn't rotate, differential drive is broken")
        turn_cmd = subprocess.run([
            'ros2', 'topic', 'pub', '/cmd_vel', 'geometry_msgs/msg/Twist',
            '{angular: {z: 0.5}}', '--times', '50'
        ], capture_output=True, timeout=7)
        time.sleep(1)
        
        # Stop
        print("\n6. Stopping robot...")
        stop_cmd = subprocess.run([
            'ros2', 'topic', 'pub', '/cmd_vel', 'geometry_msgs/msg/Twist',
            '{}', '--times', '1'
        ], capture_output=True, timeout=2)
        
        print("\n=== OBSERVATION ===")
        print("Did the robot body actually rotate during step 5? (Y/N)")
        print("If NO: Differential drive plugin is not working properly")
        print("If YES: Plugin works, problem is elsewhere")
        
        return 0
        
    except Exception as e:
        print(f"Error: {e}")
        return 1
    finally:
        print("\nCleaning up...")
        for proc in [sim_proc, gazebo_proc]:
            if proc and proc.poll() is None:
                proc.terminate()
        time.sleep(3)
        subprocess.run(['pkill', '-f', 'ign gazebo'], capture_output=True)

if __name__ == "__main__":
    sys.exit(test_minimal_turn())