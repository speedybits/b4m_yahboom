#!/usr/bin/env python3
"""
Test minimal sensor configuration in Ignition Gazebo
"""
import subprocess
import time
import sys

def main():
    print("=== Testing Minimal Sensor Configuration ===")
    
    # Start Ignition Gazebo with minimal sensor world
    print("Starting Ignition Gazebo with minimal sensor world...")
    gazebo_proc = subprocess.Popen([
        'ign', 'gazebo', 'test_minimal_sensor.sdf'
    ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    
    time.sleep(20)  # Wait for Gazebo to fully load
    
    # Check topics
    print("\n=== Checking topics ===")
    try:
        result = subprocess.run(['ign', 'topic', '-l'], capture_output=True, text=True, timeout=10)
        print("All topics:")
        print(result.stdout)
        
        # Filter for sensor-related topics
        lines = result.stdout.split('\n')
        sensor_topics = [line for line in lines if any(word in line.lower() for word in ['scan', 'lidar', 'sensor'])]
        
        if sensor_topics:
            print("Sensor-related topics found:")
            for topic in sensor_topics:
                print(f"  {topic}")
        else:
            print("No sensor-related topics found")
            
    except Exception as e:
        print(f"Error checking topics: {e}")
    
    # Try to get sensor data
    print("\n=== Trying to get sensor data ===")
    potential_topics = ['/scan', '/lidar', '/minimal_robot/lidar/scan']
    
    for topic in potential_topics:
        try:
            print(f"Checking topic: {topic}")
            result = subprocess.run(['ign', 'topic', '-e', '-t', topic], 
                                  capture_output=True, text=True, timeout=3)
            if result.stdout.strip():
                print(f"SUCCESS: Got data from {topic}")
                print(result.stdout[:200] + "..." if len(result.stdout) > 200 else result.stdout)
                break
            else:
                print(f"No data from {topic}")
        except subprocess.TimeoutExpired:
            print(f"Timeout waiting for data from {topic}")
        except Exception as e:
            print(f"Error with {topic}: {e}")
    
    # Cleanup
    print("\nCleaning up...")
    gazebo_proc.terminate()
    try:
        gazebo_proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        gazebo_proc.kill()
    
    print("Minimal sensor test complete!")

if __name__ == '__main__':
    main()