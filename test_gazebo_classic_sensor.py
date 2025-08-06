#!/usr/bin/env python3
"""
Test laser sensor with Gazebo Classic to compare
"""
import subprocess
import time
import sys

def main():
    print("=== Testing with Gazebo Classic ===")
    
    # Create a simple world file for Gazebo Classic
    world_content = '''<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="lidar_test">
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>
    
    <model name="lidar_robot">
      <pose>0 0 0.5 0 0 0</pose>
      <link name="base_link">
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.083</ixx>
            <iyy>0.083</iyy>
            <izz>0.083</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry>
            <box>
              <size>0.5 0.5 0.2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.5 0.5 0.2</size>
            </box>
          </geometry>
        </visual>
        
        <sensor name="laser" type="ray">
          <pose>0 0 0.2 0 0 0</pose>
          <visualize>true</visualize>
          <update_rate>10</update_rate>
          <ray>
            <scan>
              <horizontal>
                <samples>360</samples>
                <resolution>1</resolution>
                <min_angle>-3.14159</min_angle>
                <max_angle>3.14159</max_angle>
              </horizontal>
            </scan>
            <range>
              <min>0.08</min>
              <max>10.0</max>
              <resolution>0.01</resolution>
            </range>
          </ray>
          <plugin name="laser_controller" filename="libgazebo_ros_ray_sensor.so">
            <ros>
              <remapping>~/out:=scan</remapping>
            </ros>
            <output_type>sensor_msgs/LaserScan</output_type>
            <frame_name>laser</frame_name>
          </plugin>
        </sensor>
      </link>
    </model>
    
    <model name="box1">
      <pose>3 0 0.5 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </visual>
      </link>
    </model>
  </world>
</sdf>'''
    
    # Write world file
    with open('/tmp/gazebo_classic_test.world', 'w') as f:
        f.write(world_content)
    
    print("Starting Gazebo Classic...")
    gazebo_proc = subprocess.Popen([
        'gazebo', '--verbose', '/tmp/gazebo_classic_test.world'
    ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    
    time.sleep(15)  # Wait for Gazebo Classic to start
    
    print("Checking ROS topics...")
    try:
        result = subprocess.run(['ros2', 'topic', 'list'], 
                              capture_output=True, text=True, timeout=5)
        if '/scan' in result.stdout:
            print("✅ /scan topic found!")
            
            print("Testing scan data...")
            try:
                result = subprocess.run(['ros2', 'topic', 'echo', '/scan', '--once'], 
                                      capture_output=True, text=True, timeout=10)
                if result.stdout.strip():
                    print("✅ SUCCESS! Gazebo Classic laser sensor working!")
                    print("Data preview:", result.stdout[:200] + "...")
                else:
                    print("❌ No data from Gazebo Classic sensor")
            except subprocess.TimeoutExpired:
                print("❌ Timeout waiting for Gazebo Classic sensor data")
        else:
            print("❌ No /scan topic in Gazebo Classic")
            print("Available topics:", result.stdout.split('\n')[:10])
            
    except Exception as e:
        print(f"❌ Error: {e}")
    
    # Cleanup
    print("\nCleaning up...")
    gazebo_proc.terminate()
    try:
        gazebo_proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        gazebo_proc.kill()
    
    print("Gazebo Classic test complete!")

if __name__ == '__main__':
    main()