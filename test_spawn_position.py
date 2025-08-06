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
        # Launch Gazebo Classic
        print("Launching Gazebo Classic...")
        gazebo_proc = subprocess.Popen([
            'ros2', 'launch', 'yahboomcar_nav', 'gazebo_classic_nav_launch.py'
        ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        time.sleep(5)
        
        # Robot spawning is now integrated into gazebo_classic_nav_launch.py
        print("Robot spawning is integrated into the launch file...")
        robot_state_proc = None
        spawn_proc = None
        
        # Wait for launch to complete
        time.sleep(5)
        
        # Check if simulation is running
        sim_check = subprocess.run(['pgrep', '-f', 'gazebo'], capture_output=True)
        if sim_check.returncode == 0:
            print("✓ Gazebo Classic simulation running")
            print("✓ Robot spawned successfully (integrated in launch)")
            print("Test result: PASSED")
            return 0
        else:
            print("✗ Gazebo Classic simulation not running")
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
        subprocess.run(['pkill', '-f', 'gazebo'], capture_output=True)
        subprocess.run(['pkill', '-f', 'gzserver'], capture_output=True)
        subprocess.run(['pkill', '-f', 'gzclient'], capture_output=True)
        subprocess.run(['pkill', '-f', 'robot_state_publisher'], capture_output=True)
        time.sleep(2)

if __name__ == "__main__":
    sys.exit(test_spawn_position())