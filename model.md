# Ignition Gazebo Robot Model Test Plan

## Goal
Ensure the Yahboom robot model loads correctly in Ignition Gazebo with:
1. Wheels in direct contact with the ground (no floating or sinking)
2. Motors properly attached to wheels with functional differential drive
3. Correct physics and collision properties

## Current Model Analysis

### Key Measurements from URDF
- **Base footprint to base_link**: z = 0.0815m
- **Base_link to wheel joints**: z = -0.03295m
- **Wheel radius**: 0.033m (from DiffDrive plugin)
- **Expected wheel bottom position**: 0.0815 - 0.03295 - 0.033 = 0.0156m (15.6mm above ground)

### Identified Issues
1. Robot spawns at z=0.1m in spawn script, adding extra height
2. No wheel radius defined in wheel link geometry (using mesh)
3. Only front wheels connected to DiffDrive plugin (no rear wheels mentioned)

## Automated Test Plan

### Test 1: Model Geometry Validation
**Objective**: Verify URDF model dimensions and joint configurations

```python
#!/usr/bin/env python3
# test_model_geometry.py

import xml.etree.ElementTree as ET
import math

def test_urdf_geometry():
    """Test 1: Validate URDF geometry and calculate ground contact"""
    urdf_path = "/home/mike/projects/b4m_yahboom/yahboomcar_description/urdf/yahboomcar_robot2_gazebo.urdf"
    
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    
    # Extract key measurements
    base_joint = root.find(".//joint[@name='base_joint']/origin")
    base_z = float(base_joint.get('xyz').split()[2])
    
    wheel_joints = {}
    for wheel in ['left_front', 'right_front']:
        joint = root.find(f".//joint[@name='{wheel}_joint']/origin")
        xyz = joint.get('xyz').split()
        wheel_joints[wheel] = {
            'x': float(xyz[0]),
            'y': float(xyz[1]),
            'z': float(xyz[2])
        }
    
    # Get wheel radius from DiffDrive plugin
    diff_drive = root.find(".//plugin[@name='ignition::gazebo::systems::DiffDrive']")
    wheel_radius = float(diff_drive.find('wheel_radius').text)
    
    # Calculate expected ground contact
    for wheel, pos in wheel_joints.items():
        ground_clearance = base_z + pos['z'] - wheel_radius
        print(f"{wheel}: Expected ground clearance = {ground_clearance*1000:.1f}mm")
        assert ground_clearance < 0.001, f"{wheel} not touching ground! Clearance: {ground_clearance}m"
    
    return True
```

### Test 2: Spawn Position Test
**Objective**: Test robot spawning at correct height

```python
#!/usr/bin/env python3
# test_spawn_position.py

import subprocess
import time
import rclpy
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import GetEntityState

def test_spawn_position():
    """Test 2: Verify robot spawns at correct height"""
    rclpy.init()
    node = rclpy.create_node('spawn_test')
    
    # Launch Ignition Gazebo
    gazebo_proc = subprocess.Popen([
        'ros2', 'launch', 'yahboomcar_nav', 'ignition_gazebo_launch.py'
    ])
    time.sleep(5)
    
    # Spawn robot at ground level (z=0)
    spawn_proc = subprocess.Popen([
        'ros2', 'run', 'ros_gz_sim', 'create',
        '-name', 'test_robot',
        '-topic', '/robot_description',
        '-x', '0', '-y', '0', '-z', '0'
    ])
    time.sleep(3)
    
    # Get robot state
    client = node.create_client(GetEntityState, '/gazebo/get_entity_state')
    request = GetEntityState.Request()
    request.name = 'test_robot'
    
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    
    if future.result():
        base_z = future.result().state.pose.position.z
        print(f"Robot base Z position: {base_z}")
        
        # Expected: base_footprint at z=0, wheels touching ground
        assert abs(base_z) < 0.001, f"Robot not at ground level! Z={base_z}"
    
    # Cleanup
    spawn_proc.terminate()
    gazebo_proc.terminate()
    node.destroy_node()
    rclpy.shutdown()
```

### Test 3: Wheel Ground Contact Test
**Objective**: Verify wheels are in contact with ground

```bash
#!/bin/bash
# test_wheel_contact.sh

# Launch simulation
ros2 launch yahboomcar_nav ignition_gazebo_launch.py &
GAZEBO_PID=$!
sleep 5

# Spawn robot at calculated height for perfect ground contact
# Wheel bottom should be at z=0, so spawn at wheel_radius
ros2 run ros_gz_sim create \
    -name yahboomcar \
    -topic /robot_description \
    -x 0 -y 0 -z 0.033 &
sleep 3

# Monitor contact points using Ignition topic
ign topic -e -t /world/empty_world/dynamic_pose/info | grep -A 20 "yahboomcar" > contact_test.log &
MONITOR_PID=$!

# Let it run for 10 seconds
sleep 10

# Check for wheel contact in log
if grep -q "contact" contact_test.log; then
    echo "✓ Wheel contact detected"
else
    echo "✗ No wheel contact detected"
fi

# Cleanup
kill $MONITOR_PID
kill $GAZEBO_PID
```

### Test 4: Motor-Wheel Connection Test
**Objective**: Verify differential drive functionality

```python
#!/usr/bin/env python3
# test_motor_control.py

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time

def test_motor_control():
    """Test 4: Verify motors drive wheels correctly"""
    rclpy.init()
    node = rclpy.create_node('motor_test')
    
    # Publishers and subscribers
    cmd_pub = node.create_publisher(Twist, '/cmd_vel', 10)
    odom_data = {'received': False, 'linear_x': 0.0}
    
    def odom_callback(msg):
        odom_data['received'] = True
        odom_data['linear_x'] = msg.twist.twist.linear.x
    
    odom_sub = node.create_subscription(Odometry, '/odometry', odom_callback, 10)
    
    # Wait for system to stabilize
    time.sleep(2)
    
    # Send forward command
    cmd = Twist()
    cmd.linear.x = 0.5  # 0.5 m/s forward
    
    start_time = time.time()
    while time.time() - start_time < 5.0:
        cmd_pub.publish(cmd)
        rclpy.spin_once(node, timeout_sec=0.1)
    
    # Stop robot
    cmd.linear.x = 0.0
    cmd_pub.publish(cmd)
    
    # Verify movement
    assert odom_data['received'], "No odometry data received"
    assert abs(odom_data['linear_x'] - 0.5) < 0.1, f"Incorrect velocity: {odom_data['linear_x']}"
    
    node.destroy_node()
    rclpy.shutdown()
```

### Test 5: Physics Validation Test
**Objective**: Verify correct physics behavior

```python
#!/usr/bin/env python3
# test_physics.py

import subprocess
import time

def test_physics():
    """Test 5: Validate physics behavior"""
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
        </visual>
      </link>
    </model>
  </world>
</sdf>"""
    
    with open('/tmp/physics_test.sdf', 'w') as f:
        f.write(test_world)
    
    # Run physics test
    proc = subprocess.Popen([
        'ign', 'gazebo', '-v', '4', '/tmp/physics_test.sdf'
    ])
    
    time.sleep(5)
    
    # Spawn robot on ramp
    spawn = subprocess.Popen([
        'ros2', 'run', 'ros_gz_sim', 'create',
        '-name', 'yahboomcar',
        '-topic', '/robot_description',
        '-x', '2', '-y', '0', '-z', '0.3'
    ])
    
    # Robot should settle on ramp without penetrating
    time.sleep(10)
    
    # Cleanup
    spawn.terminate()
    proc.terminate()
```

### Test 6: Square Navigation Test
**Objective**: Verify robot can navigate in a 1-meter square pattern

```python
#!/usr/bin/env python3
# test_square_navigation.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import math
import subprocess
import sys
import threading

class SquareNavigationNode(Node):
    def __init__(self):
        super().__init__('square_navigation_test')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry', self.odom_callback, 10)
        
        # Position tracking
        self.position = {'x': 0.0, 'y': 0.0}
        self.orientation = 0.0  # yaw in radians
        self.path_points = []
        
    def odom_callback(self, msg):
        self.position['x'] = msg.pose.pose.position.x
        self.position['y'] = msg.pose.pose.position.y
        
        # Extract yaw from quaternion
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.orientation = 2 * math.atan2(qz, qw)
        
    def move_forward(self, distance, speed=0.2):
        """Move forward a specified distance"""
        start_pos = {'x': self.position['x'], 'y': self.position['y']}
        
        cmd = Twist()
        cmd.linear.x = speed
        
        while True:
            self.cmd_pub.publish(cmd)
            
            # Calculate distance traveled
            dx = self.position['x'] - start_pos['x']
            dy = self.position['y'] - start_pos['y']
            traveled = math.sqrt(dx**2 + dy**2)
            
            if traveled >= distance:
                break
            
            time.sleep(0.1)
        
        # Stop
        cmd.linear.x = 0.0
        self.cmd_pub.publish(cmd)
        time.sleep(0.5)
        
        # Record position
        self.path_points.append({'x': self.position['x'], 'y': self.position['y']})
        print(f"Position: ({self.position['x']:.2f}, {self.position['y']:.2f})")
        
    def turn_left_90(self, angular_speed=0.5):
        """Turn left 90 degrees"""
        start_orientation = self.orientation
        target_change = math.pi / 2  # 90 degrees in radians
        
        cmd = Twist()
        cmd.angular.z = angular_speed
        
        while True:
            self.cmd_pub.publish(cmd)
            
            # Calculate angle turned (handle wrap-around)
            angle_diff = self.orientation - start_orientation
            if angle_diff < -math.pi:
                angle_diff += 2 * math.pi
            elif angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            
            if angle_diff >= target_change:
                break
            
            time.sleep(0.1)
        
        # Stop
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)
        time.sleep(0.5)
        print(f"Orientation: {math.degrees(self.orientation):.1f}°")

def test_square_navigation():
    """Test 6: Navigate robot in 1-meter square"""
    
    # Cleanup first
    print("Cleaning up any existing processes...")
    subprocess.run(['./b4m_shutdown.sh', '--keep-agent'], capture_output=True)
    time.sleep(2)
    
    # Launch simulation
    print("Launching simulation...")
    gazebo_proc = subprocess.Popen([
        'ros2', 'launch', 'yahboomcar_nav', 'ignition_gazebo_launch.py'
    ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    
    sim_proc = subprocess.Popen([
        'ros2', 'launch', 'yahboomcar_nav', 'spawn_robot_with_controllers_ignition.py'
    ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    
    try:
        # Wait for system initialization
        print("Waiting for system initialization...")
        time.sleep(10)
        
        # Initialize ROS2
        rclpy.init()
        node = SquareNavigationNode()
        
        # Create executor in thread
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(node)
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()
        
        # Wait for odometry
        time.sleep(2)
        
        print("\nStarting 1-meter square navigation:")
        print("Initial position:", f"({node.position['x']:.2f}, {node.position['y']:.2f})")
        
        # Navigate in a square: 4 sides of 1 meter each
        for side in range(4):
            print(f"\n--- Side {side + 1} ---")
            node.move_forward(1.0, speed=0.2)
            if side < 3:  # Don't turn after the last side
                node.turn_left_90(angular_speed=0.5)
        
        print("\nNavigation complete!")
        
        # Verify we returned close to start
        final_x = node.position['x']
        final_y = node.position['y']
        distance_from_start = math.sqrt(final_x**2 + final_y**2)
        
        print(f"\nFinal position: ({final_x:.2f}, {final_y:.2f})")
        print(f"Distance from start: {distance_from_start:.3f}m")
        
        # Verify path forms a square
        if len(node.path_points) >= 4:
            # Calculate side lengths
            side_lengths = []
            for i in range(4):
                p1 = node.path_points[i]
                p2 = node.path_points[(i + 1) % 4]
                length = math.sqrt((p2['x'] - p1['x'])**2 + (p2['y'] - p1['y'])**2)
                side_lengths.append(length)
                print(f"Side {i+1} length: {length:.3f}m")
            
            # Check if all sides are close to 1 meter
            avg_length = sum(side_lengths) / 4
            max_deviation = max(abs(l - 1.0) for l in side_lengths)
            
            print(f"\nAverage side length: {avg_length:.3f}m")
            print(f"Maximum deviation from 1m: {max_deviation:.3f}m")
            
            # Success criteria
            if distance_from_start < 0.2 and max_deviation < 0.15:
                print("\n✓ Square navigation successful!")
                print("Test result: PASSED")
                return 0
            else:
                print("\n✗ Square navigation failed accuracy requirements")
                print("Test result: FAILED")
                return 1
        else:
            print("\n✗ Insufficient path points recorded")
            print("Test result: FAILED")
            return 1
            
    except Exception as e:
        print(f"Error during test: {e}")
        return 1
    finally:
        # Cleanup
        print("\nCleaning up...")
        if 'node' in locals():
            node.destroy_node()
        if 'executor' in locals():
            executor.shutdown()
        rclpy.shutdown()
        
        for proc in [sim_proc, gazebo_proc]:
            if proc and proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill()
        
        subprocess.run(['pkill', '-f', 'ign gazebo'], capture_output=True)
        subprocess.run(['pkill', '-f', 'controller_manager'], capture_output=True)
        subprocess.run(['pkill', '-f', 'robot_state_publisher'], capture_output=True)
        time.sleep(2)

if __name__ == "__main__":
    sys.exit(test_square_navigation())
```

### Test 7: Automated Full System Test
**Objective**: Complete integration test

```bash
#!/bin/bash
# test_full_system.sh

set -e

echo "=== Yahboom Ignition Gazebo Model Test Suite ==="

# Test 1: Model Geometry
echo "Test 1: Validating URDF geometry..."
python3 test_model_geometry.py
if [ $? -eq 0 ]; then
    echo "✓ Model geometry valid"
else
    echo "✗ Model geometry failed"
    exit 1
fi

# Test 2: Spawn Position
echo "Test 2: Testing spawn position..."
python3 test_spawn_position.py
if [ $? -eq 0 ]; then
    echo "✓ Spawn position correct"
else
    echo "✗ Spawn position failed"
    exit 1
fi

# Test 3: Wheel Contact
echo "Test 3: Testing wheel ground contact..."
./test_wheel_contact.sh
if [ $? -eq 0 ]; then
    echo "✓ Wheel contact verified"
else
    echo "✗ Wheel contact failed"
    exit 1
fi

# Test 4: Motor Control
echo "Test 4: Testing motor-wheel connection..."
python3 test_motor_control.py
if [ $? -eq 0 ]; then
    echo "✓ Motor control working"
else
    echo "✗ Motor control failed"
    exit 1
fi

# Test 5: Physics
echo "Test 5: Testing physics behavior..."
python3 test_physics.py
if [ $? -eq 0 ]; then
    echo "✓ Physics behavior correct"
else
    echo "✗ Physics behavior failed"
    exit 1
fi

# Test 6: Square Navigation
echo "Test 6: Testing 1-meter square navigation..."
python3 test_square_navigation.py
if [ $? -eq 0 ]; then
    echo "✓ Square navigation successful"
else
    echo "✗ Square navigation failed"
    exit 1
fi

echo "=== All tests passed! ==="
```

## Corrections Needed

Based on the analysis, the following corrections are needed:

### 1. Fix Spawn Height
In `spawn_robot_with_controllers_ignition.py`, change:
```python
'-x', '0', '-y', '0', '-z', '0.1'  # Current
```
To:
```python
'-x', '0', '-y', '0', '-z', '0.033'  # Wheel radius height
```

### 2. Add Wheel Collision Geometry
The URDF uses STL meshes for wheels. Add explicit cylinder collision geometry:
```xml
<collision>
  <origin xyz="0 0 0" rpy="1.57079632679 0 0"/>
  <geometry>
    <cylinder radius="0.033" length="0.015"/>
  </geometry>
</collision>
```

### 3. Add Rear Wheels to DiffDrive
Currently only front wheels are connected. Either:
- Add rear wheels to the plugin, or
- Set rear wheels as casters with proper friction

### 4. Validate Inertia Values
Ensure wheel inertias are appropriate for the mass and size.

## Running the Tests

1. Make test scripts executable:
```bash
chmod +x test_*.sh test_*.py
```

2. Run the full test suite:
```bash
./test_full_system.sh
```

3. For individual tests:
```bash
python3 test_model_geometry.py
./test_wheel_contact.sh
# etc.
```

## Expected Results

When all tests pass:
- Robot spawns with wheels touching ground (no gap)
- Wheels rotate when commanded
- Robot moves forward/backward/turns correctly
- Physics behave realistically (no jittering or penetration)
- Odometry data matches commanded velocities
- Robot can navigate a 1-meter square with <15cm deviation per side
- Robot returns to within 20cm of starting position after square navigation

## Debugging Tools

1. Visual inspection in Ignition Gazebo GUI
2. Use `ign topic -l` to list available topics
3. Monitor contacts: `ign topic -e -t /world/empty_world/dynamic_pose/info`
4. Check model state: `ros2 service call /gazebo/get_entity_state`
5. Enable physics debug visualization in Ignition GUI