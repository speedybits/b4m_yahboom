#!/usr/bin/env python3
# test_ignition_direct.py - Test direct Ignition Gazebo commands

import subprocess
import time
import sys

def test_ignition_direct():
    """Test sending commands directly to Ignition Gazebo"""
    
    print("=== Direct Ignition Gazebo Test ===")
    
    # Cleanup first
    subprocess.run(['./b4m_shutdown.sh', '--keep-agent'], capture_output=True)
    time.sleep(2)
    
    # Launch just Gazebo with robot
    print("Launching Ignition Gazebo...")
    gazebo_proc = subprocess.Popen([
        'ros2', 'launch', 'yahboomcar_nav', 'ignition_gazebo_launch.py'
    ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    time.sleep(5)
    
    print("Spawning robot...")
    spawn_proc = subprocess.Popen([
        'ros2', 'run', 'ros_gz_sim', 'create',
        '-name', 'yahboomcar',
        '-file', '/home/mike/projects/b4m_yahboom/yahboomcar_description/urdf/yahboomcar_robot2_gazebo.urdf',
        '-x', '0', '-y', '0', '-z', '0'
    ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    
    time.sleep(5)
    
    try:
        # Check what topics are available in Ignition
        print("\nChecking Ignition topics...")
        topics_result = subprocess.run(['ign', 'topic', '-l'], 
                                     capture_output=True, text=True, timeout=10)
        print("Available Ignition topics:")
        print(topics_result.stdout)
        
        # Try to send a command directly to Ignition
        print("\nTrying to send direct Ignition command...")
        
        # First, let's see if there's a cmd_vel topic in Ignition
        if '/cmd_vel' in topics_result.stdout:
            print("Found /cmd_vel topic in Ignition!")
            
            # Try to send a twist message directly
            cmd_result = subprocess.run([
                'ign', 'topic', '-t', '/cmd_vel', '-m', 'ignition.msgs.Twist',
                '-p', 'linear: {x: 0.2}, angular: {z: 0.5}'
            ], capture_output=True, text=True, timeout=5)
            
            print(f"Command result: {cmd_result.stdout}")
            print("Sent command - check if robot moves in Gazebo GUI!")
            time.sleep(5)
            
        elif '/model/yahboomcar/cmd_vel' in topics_result.stdout:
            print("Found model-specific cmd_vel topic!")
            
            cmd_result = subprocess.run([
                'ign', 'topic', '-t', '/model/yahboomcar/cmd_vel', '-m', 'ignition.msgs.Twist',
                '-p', 'linear: {x: 0.2}, angular: {z: 0.5}'
            ], capture_output=True, text=True, timeout=5)
            
            print(f"Command result: {cmd_result.stdout}")
            print("Sent command - check if robot moves in Gazebo GUI!")
            time.sleep(5)
            
        else:
            print("❌ No cmd_vel topic found in Ignition!")
            print("This means the DiffDrive plugin is not loaded or configured properly")
            
        # Check if there are any model-specific topics
        model_topics = [line for line in topics_result.stdout.split('\n') if 'yahboomcar' in line]
        if model_topics:
            print(f"\nYahboomcar-specific topics found:")
            for topic in model_topics:
                print(f"  {topic}")
        else:
            print("❌ No yahboomcar-specific topics found - robot might not be loaded properly")
        
        return 0
        
    except subprocess.TimeoutExpired:
        print("❌ Command timed out")
        return 1
    except Exception as e:
        print(f"❌ Error: {e}")
        return 1
    finally:
        print("\nCleaning up...")
        if spawn_proc and spawn_proc.poll() is None:
            spawn_proc.terminate()
        if gazebo_proc and gazebo_proc.poll() is None:
            gazebo_proc.terminate()
        
        subprocess.run(['pkill', '-f', 'ign gazebo'], capture_output=True)
        time.sleep(2)

if __name__ == "__main__":
    sys.exit(test_ignition_direct())