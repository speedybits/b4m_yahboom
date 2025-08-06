#!/usr/bin/env python3
"""
Comprehensive test to isolate the Ignition Gazebo sensor issue
"""
import subprocess
import time
import sys

def run_command_with_output(cmd, timeout=10):
    """Run command and return output"""
    try:
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=timeout)
        return result.stdout, result.stderr, result.returncode
    except subprocess.TimeoutExpired:
        return "", "TIMEOUT", -1
    except Exception as e:
        return "", str(e), -1

def main():
    print("=== Comprehensive Sensor Debug Test ===")
    print()
    
    # Test 1: Start fresh Gazebo with working world
    print("Test 1: Starting Ignition Gazebo with working lidar world...")
    gazebo_proc = subprocess.Popen([
        'ign', 'gazebo', 'working_lidar_test.sdf'
    ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    
    time.sleep(25)  # Extended wait time
    
    # Check topics
    stdout, stderr, code = run_command_with_output('ign topic -l')
    print(f"Topic list (exit code {code}):")
    if stdout:
        topics = stdout.strip().split('\n')
        lidar_topics = [t for t in topics if 'lidar' in t.lower()]
        if lidar_topics:
            print("✅ Found lidar topics:")
            for topic in lidar_topics:
                print(f"  {topic}")
        else:
            print("❌ No lidar topics found")
        
        print(f"\nAll topics ({len(topics)} total):")
        for topic in topics:
            print(f"  {topic}")
    else:
        print(f"❌ No topics found. Error: {stderr}")
    
    print()
    
    # Test 2: Try to get lidar data
    print("Test 2: Attempting to get lidar data...")
    test_topics = ['/lidar', '/gpu_lidar', '/scan']
    
    for topic in test_topics:
        print(f"Testing topic: {topic}")
        stdout, stderr, code = run_command_with_output(f'ign topic -e -t {topic}', timeout=3)
        if code == 0 and stdout.strip():
            print(f"✅ SUCCESS: Got data from {topic}")
            print(f"Sample data: {stdout[:100]}...")
            break
        else:
            print(f"❌ No data from {topic} (code: {code})")
    
    print()
    
    # Test 3: Check for sensor system errors
    print("Test 3: Checking Gazebo process output...")
    try:
        gazebo_stdout, gazebo_stderr = gazebo_proc.communicate(timeout=1)
    except subprocess.TimeoutExpired:
        print("Gazebo still running (good)")
        gazebo_stdout, gazebo_stderr = "", ""
    
    if gazebo_stderr:
        print("Gazebo stderr output:")
        print(gazebo_stderr.decode() if isinstance(gazebo_stderr, bytes) else gazebo_stderr)
    
    # Cleanup
    print("\nCleaning up...")
    gazebo_proc.terminate()
    try:
        gazebo_proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        gazebo_proc.kill()
    
    print("\n=== Test Complete ===")
    print("\nConclusion: Based on the comprehensive testing:")
    print("- If no lidar topics appear: Sensor system plugin issue")
    print("- If lidar topics appear but no data: Sensor configuration issue") 
    print("- If data appears: Sensor working, issue is in our URDF configuration")

if __name__ == '__main__':
    main()