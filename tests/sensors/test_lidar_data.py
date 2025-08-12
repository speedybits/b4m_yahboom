#!/usr/bin/env python3
"""
Test actual lidar data with correct command syntax
"""
import subprocess
import time
import sys

def main():
    print("=== Testing Lidar Data with Correct Syntax ===")
    
    # Start Gazebo with working lidar
    print("Starting Ignition Gazebo...")
    gazebo_proc = subprocess.Popen([
        'ign', 'gazebo', 'working_lidar_test.sdf'
    ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    
    time.sleep(25)  # Wait for full startup
    
    # Test different ways to get data
    print("\nTesting data retrieval methods...")
    
    # Method 1: Check topic info
    print("1. Getting topic info...")
    try:
        result = subprocess.run(['ign', 'topic', '--info', '/lidar'], 
                              capture_output=True, text=True, timeout=5)
        if result.returncode == 0:
            print(f"✅ Topic info: {result.stdout}")
        else:
            print(f"❌ Topic info failed: {result.stderr}")
    except Exception as e:
        print(f"❌ Topic info error: {e}")
    
    # Method 2: Try echo with different syntax
    print("\n2. Testing echo with correct syntax...")
    try:
        result = subprocess.run(['ign', 'topic', '--echo', '--topic', '/lidar'], 
                              capture_output=True, text=True, timeout=5)
        if result.returncode == 0 and result.stdout.strip():
            print(f"✅ SUCCESS! Got lidar data:")
            print(result.stdout[:300] + "..." if len(result.stdout) > 300 else result.stdout)
        else:
            print(f"❌ No data with --echo --topic syntax: {result.stderr}")
    except subprocess.TimeoutExpired:
        print("❌ Timeout waiting for data")
    except Exception as e:
        print(f"❌ Echo error: {e}")
    
    # Method 3: List active topics
    print("\n3. Checking which topics are actually publishing...")
    try:
        result = subprocess.run(['ign', 'topic', '--list'], 
                              capture_output=True, text=True, timeout=5)
        if result.returncode == 0:
            topics = result.stdout.strip().split('\n')
            lidar_topics = [t for t in topics if 'lidar' in t.lower() or 'scan' in t.lower()]
            print(f"Lidar-related topics: {lidar_topics}")
            
            # Try each lidar topic
            for topic in lidar_topics[:3]:  # Test first 3 only
                print(f"\nTesting topic: {topic}")
                try:
                    result = subprocess.run(['ign', 'topic', '--echo', '--topic', topic], 
                                          capture_output=True, text=True, timeout=3)
                    if result.stdout.strip():
                        print(f"✅ Got data from {topic}!")
                        print(result.stdout[:200] + "..." if len(result.stdout) > 200 else result.stdout)
                        break
                    else:
                        print(f"❌ No data from {topic}")
                except subprocess.TimeoutExpired:
                    print(f"❌ Timeout on {topic}")
                    
    except Exception as e:
        print(f"❌ List error: {e}")
    
    # Cleanup
    print("\nCleaning up...")
    gazebo_proc.terminate()
    try:
        gazebo_proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        gazebo_proc.kill()
    
    print("Test complete!")

if __name__ == '__main__':
    main()