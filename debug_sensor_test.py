#!/usr/bin/env python3
"""
Debug script to test laser sensor configuration in Ignition Gazebo
"""
import subprocess
import time
import sys

def run_command(cmd, description):
    """Run a command and print results"""
    print(f"\n=== {description} ===")
    try:
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=10)
        print(f"Exit code: {result.returncode}")
        if result.stdout:
            print(f"STDOUT:\n{result.stdout}")
        if result.stderr:
            print(f"STDERR:\n{result.stderr}")
        return result
    except subprocess.TimeoutExpired:
        print("Command timed out after 10 seconds")
        return None
    except Exception as e:
        print(f"Error running command: {e}")
        return None

def main():
    print("=== Laser Sensor Debug Test ===")
    
    # Step 1: Start Ignition Gazebo
    print("Starting Ignition Gazebo...")
    gazebo_proc = subprocess.Popen([
        'ign', 'gazebo', '/usr/share/ignition/ignition-gazebo6/worlds/empty.sdf'
    ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    
    time.sleep(15)  # Wait for Gazebo to start
    
    # Step 2: Check initial topics
    run_command('ign topic -l', 'Initial Ignition topics')
    
    # Step 3: Spawn robot
    print("\nSpawning robot...")
    spawn_result = run_command(
        'ros2 run ros_gz_sim create -file /home/mike/projects/b4m_yahboom/install/yahboomcar_description/share/yahboomcar_description/urdf/yahboomcar_robot2_gazebo.urdf -name yahboomcar -x 0 -y 0 -z 0',
        'Robot spawn'
    )
    
    time.sleep(10)  # Wait for robot to spawn
    
    # Step 4: Check topics after spawn
    run_command('ign topic -l | grep -E "(scan|lidar|sensor|yahboom)"', 'Topics after robot spawn')
    
    # Step 5: Check all topics
    run_command('ign topic -l', 'All topics after robot spawn')
    
    # Step 6: Check if there are any sensor-related messages
    run_command('ign topic -l | grep -i sensor', 'Sensor topics')
    
    # Step 7: Try to echo any scan topic that might exist
    scan_topics = ['/scan', '/yahboomcar/scan', '/model/yahboomcar/scan', '/world/empty/model/yahboomcar/scan']
    
    for topic in scan_topics:
        print(f"\nTrying to echo topic: {topic}")
        result = run_command(f'timeout 3 ign topic -e {topic}', f'Echo {topic}')
    
    # Cleanup
    print("\nCleaning up...")
    gazebo_proc.terminate()
    try:
        gazebo_proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        gazebo_proc.kill()
    
    print("Debug test complete!")

if __name__ == '__main__':
    main()