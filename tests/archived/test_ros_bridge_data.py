#!/usr/bin/env python3
"""
Test lidar data through ROS-Gazebo bridge
"""
import subprocess
import time
import sys

def main():
    print("=== Testing Lidar Data Through ROS Bridge ===")
    
    # Start Gazebo
    print("Starting Ignition Gazebo...")
    gazebo_proc = subprocess.Popen([
        'ign', 'gazebo', 'working_lidar_test.sdf'
    ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    
    time.sleep(15)
    
    # Start ROS-Gazebo bridge for lidar topic
    print("Starting ROS-Gazebo bridge...")
    bridge_proc = subprocess.Popen([
        'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
        '/lidar@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan'
    ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    
    time.sleep(10)  # Wait for bridge
    
    # Check ROS topics
    print("Checking ROS topics...")
    try:
        result = subprocess.run(['ros2', 'topic', 'list'], 
                              capture_output=True, text=True, timeout=5)
        if result.returncode == 0:
            topics = result.stdout.strip().split('\n')
            lidar_topics = [t for t in topics if 'lidar' in t.lower() or 'scan' in t.lower()]
            print(f"ROS lidar topics: {lidar_topics}")
            
            if '/lidar' in topics:
                print("\n✅ /lidar topic exists in ROS!")
                
                # Try to get data
                print("Attempting to get ROS lidar data...")
                try:
                    result = subprocess.run(['ros2', 'topic', 'echo', '/lidar', '--once'], 
                                          capture_output=True, text=True, timeout=10)
                    if result.stdout.strip():
                        print("✅ SUCCESS! Got ROS lidar data:")
                        print(result.stdout[:500] + "..." if len(result.stdout) > 500 else result.stdout)
                    else:
                        print("❌ No data from ROS /lidar topic")
                        print(f"Error: {result.stderr}")
                except subprocess.TimeoutExpired:
                    print("❌ Timeout waiting for ROS lidar data")
            else:
                print("❌ /lidar topic not bridged to ROS")
        else:
            print(f"❌ Error listing ROS topics: {result.stderr}")
    except Exception as e:
        print(f"❌ ROS topic error: {e}")
    
    # Check bridge status
    print("\nChecking bridge process status...")
    if bridge_proc.poll() is None:
        print("✅ Bridge process still running")
    else:
        print("❌ Bridge process terminated")
        bridge_stdout, bridge_stderr = bridge_proc.communicate()
        print(f"Bridge stdout: {bridge_stdout.decode() if bridge_stdout else 'None'}")
        print(f"Bridge stderr: {bridge_stderr.decode() if bridge_stderr else 'None'}")
    
    # Cleanup
    print("\nCleaning up...")
    bridge_proc.terminate()
    gazebo_proc.terminate()
    
    try:
        bridge_proc.wait(timeout=5)
        gazebo_proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        bridge_proc.kill()
        gazebo_proc.kill()
    
    print("Test complete!")

if __name__ == '__main__':
    main()