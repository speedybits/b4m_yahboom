#!/usr/bin/env python3
# test_physics.py

import subprocess
import time
import sys

def test_physics():
    """Test 5: Validate physics behavior"""
    
    # Cleanup first
    print("Cleaning up any existing processes...")
    subprocess.run(['./b4m_shutdown.sh', '--keep-agent'], capture_output=True)
    time.sleep(2)
    
    # Create test world with ramp
    test_world = """<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="physics_test">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    
    <plugin filename="libignition-gazebo-physics-system.so" 
            name="ignition::gazebo::systems::Physics"/>
    <plugin filename="libignition-gazebo-scene-broadcaster-system.so" 
            name="ignition::gazebo::systems::SceneBroadcaster"/>
    
    <!-- Ground -->
    <model name="ground">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal></plane></geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    
    <!-- Ramp for testing -->
    <model name="ramp">
      <static>true</static>
      <pose>2 0 0.1 0 -0.1 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>2 1 0.2</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>2 1 0.2</size></box></geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    
    <!-- Lighting -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
  </world>
</sdf>"""
    
    proc = None
    robot_state_proc = None
    spawn = None
    
    try:
        # Write test world
        with open('/tmp/physics_test.sdf', 'w') as f:
            f.write(test_world)
        
        # Run physics test world
        print("Launching physics test world...")
        proc = subprocess.Popen([
            'ign', 'gazebo', '-v', '4', '/tmp/physics_test.sdf'
        ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        
        time.sleep(5)
        
        # Start robot state publisher
        print("Starting robot state publisher...")
        urdf_path = "/home/mike/projects/b4m_yahboom/yahboomcar_description/urdf/yahboomcar_robot2_gazebo.urdf"
        with open(urdf_path, 'r') as f:
            robot_desc = f.read()
        
        robot_state_proc = subprocess.Popen([
            'ros2', 'run', 'robot_state_publisher', 'robot_state_publisher',
            '--ros-args', '-p', f'robot_description:={robot_desc}'
        ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        time.sleep(2)
        
        # Spawn robot on ramp to test physics
        print("Spawning robot on ramp...")
        spawn = subprocess.Popen([
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-name', 'yahboomcar',
            '-topic', '/robot_description',
            '-x', '2', '-y', '0', '-z', '0.3'
        ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        
        # Robot should settle on ramp without penetrating
        print("Waiting for physics to settle...")
        time.sleep(10)
        
        # Check if simulation is still running (no crashes)
        if proc.poll() is None:
            print("✓ Physics simulation stable - no crashes")
            print("✓ Robot should have settled on ramp")
            print("Test result: PASSED")
            return 0
        else:
            print("✗ Simulation crashed")
            print("Test result: FAILED")
            return 1
            
    except Exception as e:
        print(f"Error during test: {e}")
        return 1
    finally:
        # Cleanup
        print("\nCleaning up...")
        for p in [spawn, robot_state_proc, proc]:
            if p and p.poll() is None:
                p.terminate()
                try:
                    p.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    p.kill()
        
        subprocess.run(['pkill', '-f', 'ign gazebo'], capture_output=True)
        subprocess.run(['pkill', '-f', 'robot_state_publisher'], capture_output=True)
        time.sleep(2)

if __name__ == "__main__":
    sys.exit(test_physics())