# Regression Test Fix Documentation

## Date: 2025-08-04

## Summary
Successfully fixed the `test_square_corners.py` regression test at commit cb4fb726a8acc7ebe94be88cf1e28a9ca80bcdc1. The test now passes with the robot successfully navigating a 1-meter square pattern in Ignition Gazebo simulation.

## Issues Identified and Fixed

### 1. World File Path Issue
**Problem**: The launch file was looking for a missing world file at `/home/mike/projects/b4m_yahboom/worlds/empty.sdf`

**Solution**: Updated `yahboomcar_nav/launch/ignition_gazebo_launch.py` to use the system world file:
```python
# World file path - use system empty world
world_file = '/usr/share/ignition/ignition-gazebo6/worlds/empty.sdf'
```

### 2. DiffDrive Plugin Topic Configuration
**Problem**: The robot model wasn't responding to movement commands. The DiffDrive plugin was creating model-specific topics but not subscribing to commands properly.

**Root Cause**: The URDF DiffDrive plugin configuration was using relative topic names (`cmd_vel`, `odometry`, `tf`) instead of global topic names.

**Solution**: Updated `yahboomcar_description/urdf/yahboomcar_robot2_gazebo.urdf` to use global topic names:
```xml
<topic>/cmd_vel</topic>
<odom_topic>/odometry</odom_topic>
<tf_topic>/tf</tf_topic>
```

### 3. ROS-Gazebo Bridge Configuration
**Problem**: Initial attempts to remap model-specific topics were unsuccessful.

**Solution**: With the URDF using global topic names, the spawn script could use simple direct bridging in `yahboomcar_nav/launch/spawn_robot_with_controllers_ignition.py`:
```python
# ROS-Ignition Bridge for cmd_vel and odometry - using global topics
Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=[
        '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
        '/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
        '/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V'
    ],
    output='screen',
    remappings=[
        ('/tf', '/tf')
    ]
),
```

## Test Results

### Before Fix
- Robot stuck at position (0,0,0)
- No response to movement commands
- Odometry topic not publishing data
- Test failed with "SQUARE NAVIGATION FAILED"

### After Fix
- Robot successfully navigates all corners
- Final position accuracy: 0.149m from start (threshold: 0.2m)
- All 5 corners reached successfully
- Test passes with "SQUARE NAVIGATION SUCCESSFUL!"

### Actual Navigation Path
1. Start: (0.000, 0.000)
2. Corner 2: (0.857, 0.000)
3. Corner 3: (0.982, 0.854)
4. Corner 4: (0.147, 0.981)
5. Return to Start: (0.019, 0.148)

## Key Learnings

1. **Ignition Gazebo DiffDrive Plugin**: Requires global topic names (starting with `/`) in the URDF configuration to work properly with ROS2 bridges.

2. **Topic Discovery**: The DiffDrive plugin creates topics like `/model/{model_name}/cmd_vel` when using relative names, but these don't always appear in topic lists until subscribed to.

3. **System World Files**: Using system-provided world files is more reliable than local copies for standard environments.

## Commands to Reproduce Fix

```bash
# 1. Clean shutdown
./b4m_shutdown.sh --keep-agent

# 2. Rebuild affected packages
colcon build --packages-select yahboomcar_description yahboomcar_nav --allow-overriding yahboomcar_nav

# 3. Source environment
source install/setup.bash

# 4. Run test
python3 test_square_corners.py
```

## Files Modified

1. `/home/mike/projects/b4m_yahboom/yahboomcar_nav/launch/ignition_gazebo_launch.py`
   - Updated world file path to use system empty.sdf

2. `/home/mike/projects/b4m_yahboom/yahboomcar_description/urdf/yahboomcar_robot2_gazebo.urdf`
   - Changed DiffDrive plugin topics to use global names

3. `/home/mike/projects/b4m_yahboom/yahboomcar_nav/launch/spawn_robot_with_controllers_ignition.py`
   - Simplified bridge configuration to use global topics directly

## Verification

The fix can be verified by:
1. Running `python3 test_square_corners.py`
2. Observing the robot in Ignition Gazebo GUI navigating the square pattern
3. Checking the test output shows all corners reached and "SQUARE NAVIGATION SUCCESSFUL!"