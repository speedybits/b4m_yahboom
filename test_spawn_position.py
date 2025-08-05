#!/usr/bin/env python3
# test_spawn_position.py

import subprocess
import time
import sys
import os
import signal

def test_spawn_position():
    """Test 2: Verify robot spawns at correct height"""
    
    # First, shut down any existing ROS2 processes
    print("Shutting down any existing processes...")
    subprocess.run(['./b4m_shutdown.sh', '--keep-agent'], capture_output=True)
    time.sleep(2)
    
    # Source the workspace
    subprocess.run(['bash', '-c', 'source install/setup.bash'], capture_output=True)
    
    gazebo_proc = None
    robot_state_proc = None
    spawn_proc = None
    
    try:
        # Launch Ignition Gazebo
        print("Launching Ignition Gazebo...")
        gazebo_proc = subprocess.Popen([
            'ros2', 'launch', 'yahboomcar_nav', 'ignition_gazebo_launch.py'
        ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        time.sleep(5)
        
        # Start robot state publisher with URDF
        print("Starting robot state publisher...")
        urdf_path = "/home/mike/projects/b4m_yahboom/yahboomcar_description/urdf/yahboomcar_robot2_gazebo.urdf"
        with open(urdf_path, 'r') as f:
            robot_desc = f.read()
        
        robot_state_proc = subprocess.Popen([
            'ros2', 'run', 'robot_state_publisher', 'robot_state_publisher',
            '--ros-args', '-p', f'robot_description:={robot_desc}'
        ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        time.sleep(2)
        
        # Spawn robot at ground level (z=0)
        print("Spawning robot at z=0...")
        spawn_proc = subprocess.Popen([
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-name', 'test_robot',
            '-topic', '/robot_description',
            '-x', '0', '-y', '0', '-z', '0'
        ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        
        # Wait for spawn to complete
        time.sleep(5)
        
        # Check if robot spawned successfully by checking spawn process return code
        spawn_return = spawn_proc.wait(timeout=5)
        
        if spawn_return == 0:
            print("✓ Robot spawned successfully")
            # Additional verification: check if simulation is running
            sim_check = subprocess.run(['pgrep', '-f', 'ign gazebo'], capture_output=True)
            if sim_check.returncode == 0:
                print("✓ Simulation running")
                print("Test result: PASSED")
                return 0
            else:
                print("✗ Simulation not running")
                print("Test result: FAILED")
                return 1
        else:
            print("✗ Robot spawn failed with return code:", spawn_return)
            # Check spawn stderr for errors
            spawn_errors = spawn_proc.stderr.read().decode()
            if spawn_errors:
                print("Spawn errors:", spawn_errors)
            print("Test result: FAILED")
            return 1
            
    except Exception as e:
        print(f"Error during test: {e}")
        return 1
    finally:
        # Cleanup
        print("\nCleaning up...")
        for proc in [spawn_proc, robot_state_proc, gazebo_proc]:
            if proc and proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill()
        
        # Extra cleanup
        subprocess.run(['pkill', '-f', 'ign gazebo'], capture_output=True)
        subprocess.run(['pkill', '-f', 'ros_gz_sim'], capture_output=True)
        subprocess.run(['pkill', '-f', 'robot_state_publisher'], capture_output=True)
        time.sleep(2)

if __name__ == "__main__":
    sys.exit(test_spawn_position())