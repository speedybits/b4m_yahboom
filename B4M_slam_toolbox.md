# B4M SLAM Toolbox Integration Guide

This document provides comprehensive steps to convert the existing B4M Robot system from AMCL localization to slam_toolbox for improved localization performance and simultaneous localization and mapping (SLAM) capabilities.

## Overview

The B4M Robot currently uses:
- **AMCL** for localization with pre-built maps
- **Gmapping** for SLAM mapping (to be replaced)
- **Nav2** for navigation stack
- **EKF** from robot_localization for sensor fusion
- **Micro-ROS Agent** for ESP32 hardware communication

This guide converts the system to use:
- **slam_toolbox** for unified SLAM-based localization and mapping (replaces both AMCL and gmapping)
- Maintains **Nav2** navigation stack compatibility
- **EKF integration analysis** with options for optimization or bypass
- **Micro-ROS compatibility** considerations

## Benefits of slam_toolbox

- **Unified Solution**: Replaces both AMCL and gmapping with a single, robust solution
- **Improved Localization**: Dynamic map updates improve localization accuracy
- **Real-time Mapping**: Ability to create and update maps during operation
- **Loop Closure**: Automatic map correction when revisiting known areas
- **Better Performance**: More robust in dynamic environments compared to AMCL
- **Map Updates**: Can handle environment changes automatically
- **Micro-ROS Compatible**: Works seamlessly with ESP32 hardware communication
- **Fresh Start**: No need to convert existing maps - creates new, optimized maps

## Prerequisites

### Package Dependencies

Ensure these packages are installed:

```bash
sudo apt install ros-humble-slam-toolbox ros-humble-nav2-bringup
```

### Current System Analysis

Your current system configuration:
- **Launch Script**: `b4m_HA_launch.sh` coordinates the entire system
- **Navigation Launch**: `yahboomcar_nav/launch/waypoint_navigation_launch.py`
- **AMCL Navigation**: `yahboomcar_nav/launch/waypoint_navigation_launch.py`
- **Gmapping**: `yahboomcar_nav/launch/map_gmapping_launch.py` (to be removed)
- **Navigation Parameters**: `yahboomcar_nav/params/dwb_nav_params.yaml`
- **Map**: Static map loaded from `yahboomcar_nav/maps/yahboom_map.yaml`
- **Hardware Integration**: EKF sensor fusion with IMU and odometry via Micro-ROS Agent

### EKF Integration Analysis

Your current EKF configuration (`yahboomcar_bringup/param/ekf.yaml`) uses:
- **Odometry Source**: `/odom_raw` from ESP32 via Micro-ROS
- **IMU Source**: `/imu` from ESP32 via Micro-ROS  
- **Output**: Filtered odometry to `/odom` (remapped from `/odometry/filtered`)
- **Transform**: Publishes `odom -> base_footprint` transform
- **2D Mode**: Enabled for ground robot operation

**EKF Integration with slam_toolbox:**

slam_toolbox will use the filtered odometry from your existing EKF configuration:
- slam_toolbox uses filtered odometry from EKF (`/odom` topic)
- Maintains robust sensor fusion of IMU + wheel odometry
- EKF handles noisy ESP32 sensor data providing smooth input for SLAM
- EKF publishes `odom -> base_footprint` transform
- slam_toolbox publishes `map -> odom` transform
- Results in stable `map -> odom -> base_footprint` transform chain

### Micro-ROS Agent Considerations

Your ESP32 connection through Micro-ROS Agent provides:
- **Raw Odometry**: `/odom_raw` topic
- **IMU Data**: `/imu` topic  
- **Laser Data**: `/scan` topic
- **Hardware Control**: Motor commands via Micro-ROS

**SLAM-specific considerations:**
- slam_toolbox scan matching may provide better odometry than raw ESP32 encoder data
- Loop closure can correct drift that EKF cannot handle
- Transform timing critical with Micro-ROS network latency

## Implementation Steps

### Step 1: Create slam_toolbox Configuration

Create new parameter file for slam_toolbox:

**File**: `yahboomcar_nav/params/slam_toolbox_params.yaml`

```yaml
slam_toolbox:
  ros__parameters:
    # Basic Configuration
    odom_frame: odom
    map_frame: map
    base_frame: base_footprint
    scan_topic: /scan
    use_map_saver: true
    use_lifecycle_manager: false

    # SLAM Parameters
    mode: mapping  # Options: mapping, localization, lifelong
    
    # EKF Integration - Use filtered odometry
    # EKF publishes odom->base_footprint, slam_toolbox publishes map->odom
    
    # Loop Closure Parameters (tuned for indoor environment)
    loop_search_maximum_distance: 3.0
    loop_match_minimum_response_coarse: 0.35
    loop_match_minimum_response_fine: 0.45
    
    # Scan Processing (optimized for Micro-ROS latency)
    max_laser_range: 20.0
    minimum_time_interval: 0.8  # Slightly higher for Micro-ROS stability
    transform_timeout: 0.5      # Higher timeout for network latency
    tf_buffer_duration: 30.0
    
    # Performance Tuning for B4M Robot with EKF
    stack_size_to_use: 40000000
    minimum_travel_distance: 0.3  # More sensitive with EKF filtering
    minimum_travel_heading: 0.3   # More sensitive with EKF filtering
    scan_buffer_size: 5           # Smaller buffer for limited CPU
    scan_buffer_maximum_scan_distance: 10.0
    link_match_minimum_response_fine: 0.1
    link_scan_maximum_distance: 1.5
    
    # Map Resolution and Quality
    resolution: 0.05
    map_file_name: yahboom_slam_map
    map_start_at_dock: true
    
    # Solver Parameters
    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: SCHUR_JACOBI
    ceres_trust_strategy: LEVENBERG_MARQUARDT
    ceres_dogleg_type: TRADITIONAL_DOGLEG
    ceres_loss_function: None

# Map Saver Parameters
map_saver:
  ros__parameters:
    use_sim_time: false
    save_map_timeout: 5.0
    free_thresh_default: 0.25
    occupied_thresh_default: 0.65
    map_subscribe_transient_local: true
```

### Step 2: Create SLAM Launch File

Create launch file for slam_toolbox integration:

**File**: `yahboomcar_nav/launch/slam_toolbox_launch.py`

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_path = get_package_share_directory('yahboomcar_nav')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    slam_params_file = LaunchConfiguration('slam_params_file', 
        default=os.path.join(package_path, 'params', 'slam_toolbox_params.yaml'))

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value=use_sim_time,
                              description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('slam_params_file', default_value=slam_params_file,
                              description='Full path to slam_toolbox params file'),

        # SLAM Toolbox Node
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params_file, {'use_sim_time': use_sim_time}],
            remappings=[('/scan', '/scan')],
        ),

        # TF static transform publisher for laser
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_base_laser',
            arguments=['-0.0046412', '0', '0.094079', '0', '0', '0', 'base_link', 'laser_frame']
        ),
    ])
```

### Step 3: Create Modified Navigation Launch File

Create new navigation launch file that uses slam_toolbox instead of AMCL:

**File**: `yahboomcar_nav/launch/slam_navigation_launch.py`

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_path = get_package_share_directory('yahboomcar_nav')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    namespace = LaunchConfiguration('namespace', default='')
    
    # Use slam_toolbox navigation parameters (modified to exclude AMCL)
    nav2_param_path = LaunchConfiguration('params_file', default=os.path.join(
        package_path, 'params', 'slam_nav_params.yaml'))
    
    # SLAM Toolbox parameters
    slam_params_file = LaunchConfiguration('slam_params_file', 
        default=os.path.join(package_path, 'params', 'slam_toolbox_params.yaml'))

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value=use_sim_time,
                              description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('namespace', default_value=namespace,
                              description='Robot namespace'),
        DeclareLaunchArgument('params_file', default_value=nav2_param_path,
                              description='Full path to nav2 param file to load'),
        DeclareLaunchArgument('slam_params_file', default_value=slam_params_file,
                              description='Full path to slam_toolbox params file'),

        # SLAM Toolbox Launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [package_path, '/launch', '/slam_toolbox_launch.py']),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'slam_params_file': slam_params_file}.items(),
        ),

        # Nav2 Launch (without map_server and AMCL)
        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[nav2_param_path, {'use_sim_time': use_sim_time}],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_param_path, {'use_sim_time': use_sim_time}],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        ),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[nav2_param_path, {'use_sim_time': use_sim_time}],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_param_path, {'use_sim_time': use_sim_time}],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        ),

        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[nav2_param_path, {'use_sim_time': use_sim_time}],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        ),

        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[nav2_param_path, {'use_sim_time': use_sim_time}],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static'),
                        ('/cmd_vel', 'cmd_vel_nav'), ('/cmd_vel_smoothed', 'cmd_vel')],
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': ['controller_server',
                                       'planner_server',
                                       'behavior_server',
                                       'bt_navigator',
                                       'waypoint_follower',
                                       'velocity_smoother']}],
        ),
        
        # Waypoint navigation node
        Node(
            package='yahboomcar_nav',
            executable='waypoint_navigation',
            name='waypoint_navigation_node',
            output='screen'
        ),
        
        # Stop car node for safe shutdown
        Node(
            package='yahboomcar_nav',
            executable='stop_car'
        )
    ])
```

### Step 4: Create SLAM-Compatible Navigation Parameters

Copy your existing nav parameters and remove AMCL configuration:

**File**: `yahboomcar_nav/params/slam_nav_params.yaml`

```yaml
# Remove the entire amcl: section from dwb_nav_params.yaml
# Keep all other sections: bt_navigator, controller_server, local_costmap, 
# global_costmap, planner_server, smoother_server, behavior_server, 
# robot_state_publisher, waypoint_follower, velocity_smoother

# All parameters from dwb_nav_params.yaml EXCEPT amcl section
bt_navigator:
  ros__parameters:
    use_sim_time: False
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    default_bt_xml_filename: "navigate_to_pose_w_replanning_and_recovery.xml"
    plugin_lib_names:
      - nav2_compute_path_to_pose_action_bt_node
      - nav2_compute_path_through_poses_action_bt_node
      - nav2_smooth_path_action_bt_node
      - nav2_follow_path_action_bt_node
      - nav2_spin_action_bt_node
      - nav2_wait_action_bt_node
      - nav2_assisted_teleop_action_bt_node
      - nav2_back_up_action_bt_node
      - nav2_drive_on_heading_bt_node
      - nav2_clear_costmap_service_bt_node
      - nav2_is_stuck_condition_bt_node
      - nav2_goal_reached_condition_bt_node
      - nav2_goal_updated_condition_bt_node
      - nav2_globally_updated_goal_condition_bt_node
      - nav2_is_path_valid_condition_bt_node
      - nav2_initial_pose_received_condition_bt_node
      - nav2_reinitialize_global_localization_service_bt_node
      - nav2_rate_controller_bt_node
      - nav2_distance_controller_bt_node
      - nav2_speed_controller_bt_node
      - nav2_truncate_path_action_bt_node
      - nav2_truncate_path_local_action_bt_node
      - nav2_goal_updater_node_bt_node
      - nav2_recovery_node_bt_node
      - nav2_pipeline_sequence_bt_node
      - nav2_round_robin_node_bt_node
      - nav2_transform_available_condition_bt_node
      - nav2_time_expired_condition_bt_node
      - nav2_path_expiring_timer_condition
      - nav2_distance_traveled_condition_bt_node
      - nav2_single_trigger_bt_node
      - nav2_goal_updated_controller_bt_node
      - nav2_is_battery_low_condition_bt_node
      - nav2_navigate_through_poses_action_bt_node
      - nav2_navigate_to_pose_action_bt_node
      - nav2_remove_passed_goals_action_bt_node
      - nav2_planner_selector_bt_node
      - nav2_controller_selector_bt_node
      - nav2_goal_checker_selector_bt_node
      - nav2_controller_cancel_bt_node
      - nav2_path_longer_on_approach_bt_node
      - nav2_wait_cancel_bt_node
      - nav2_spin_cancel_bt_node
      - nav2_back_up_cancel_bt_node
      - nav2_assisted_teleop_cancel_bt_node
      - nav2_drive_on_heading_cancel_bt_node
      - nav2_is_battery_charging_condition_bt_node

# Copy all other sections from dwb_nav_params.yaml...
# (controller_server, local_costmap, global_costmap, planner_server, etc.)
```

### Step 5: Update b4m_HA_launch.sh Script

The existing launch script needs modifications to integrate slam_toolbox and remove AMCL/gmapping dependencies:

**Key Changes Required:**

1. **Remove gmapping references** from cleanup functions
2. **Update navigation detection** to look for slam_toolbox instead of AMCL  
3. **Modify Step 5** to use slam_navigation_launch.py
4. **Remove/modify Step 6** (pose initialization not needed)
5. **Update test functions** to work with slam_toolbox

**Step 5 Command Change:**
```bash
# OLD (AMCL):
launch_in_terminal "Launching the navigation system with pre-built map" \
    "cd \"$WORKSPACE_ROOT\" && . install/setup.bash && ros2 launch yahboomcar_nav waypoint_navigation_launch.py maps:=\"$WORKSPACE_ROOT/yahboomcar_nav/maps/yahboom_map.yaml\"" \
    "5"

# NEW (SLAM Toolbox):
launch_in_terminal "Launching SLAM-based navigation system" \
    "cd \"$WORKSPACE_ROOT\" && . install/setup.bash && ros2 launch yahboomcar_nav slam_navigation_launch.py" \
    "5"
```

**Step 6 Modification:**
```bash
# OLD (Pose initialization):
launch_in_terminal "Setting automatic pose estimate at map center for testing" \
    "cd \"$WORKSPACE_ROOT\" && . install/setup.bash && python3 \"$WORKSPACE_ROOT/scripts/set_initial_pose.py\"" \
    "6"

# NEW (SLAM monitoring - optional):
launch_in_terminal "Monitoring SLAM initialization and map building" \
    "cd \"$WORKSPACE_ROOT\" && . install/setup.bash && echo 'SLAM system initializing - map will be built automatically as robot moves'" \
    "6"
```

**Test Function Updates:**

The existing test functions need to be updated for slam_toolbox:

```bash
# Update navigation node detection (line 133, 270):
# OLD:
local navigation_nodes=$(ros2 node list | grep -E "(amcl|nav2_container)" | wc -l 2>/dev/null || echo "0")

# NEW:
local navigation_nodes=$(ros2 node list | grep -E "(slam_toolbox|nav2_container)" | wc -l 2>/dev/null || echo "0")

# Update Step 5 validation (line 529-531):
# OLD:
if ros2 node list 2>/dev/null | grep -q "map_server" && \
   ros2 topic list 2>/dev/null | grep -q "/map" && \
   ros2 node list 2>/dev/null | grep -q "amcl"; then

# NEW:
if ros2 node list 2>/dev/null | grep -q "slam_toolbox" && \
   ros2 topic list 2>/dev/null | grep -q "/map"; then

# Update test_global_localization function (line 601):
# Replace AMCL pose checking with slam_toolbox pose checking
test_slam_localization() {
    debug_log "Testing SLAM localization initialization"
    local timeout=60
    local end_time=$(($(date +%s) + timeout))
    
    # Check if slam_toolbox is publishing poses and map updates
    while [ $(date +%s) -lt $end_time ]; do
        if ros2 topic list 2>/dev/null | grep -q "/slam_toolbox" && \
           timeout 5 ros2 topic echo /map --once >/dev/null 2>&1; then
            debug_log "SLAM toolbox active - localization working"
            return 0
        fi
        sleep 2
    done
    
    echo "ERROR: SLAM localization failed - no slam_toolbox activity within $timeout seconds"
    return 1
}

# Update Step 6 validation (line 561):
# OLD: Check AMCL pose publication
# NEW: Check SLAM system initialization
validate_slam_initialization() {
    debug_log "Step 6: Validating SLAM system initialization"
    
    # Wait for SLAM system to start
    sleep 5
    
    # Verify slam_toolbox is running and publishing transforms
    local end_time=$(($(date +%s) + timeout))
    while [ $(date +%s) -lt $end_time ]; do
        if ros2 node list 2>/dev/null | grep -q "slam_toolbox" && \
           ros2 run tf2_ros tf2_echo map odom --timeout 2 >/dev/null 2>&1; then
            debug_log "Step 6 validation passed: SLAM system initialized and publishing transforms"
            return 0
        fi
        sleep 1
    done
    
    echo "ERROR: Step 6 validation failed - SLAM system not initializing properly within $timeout seconds"
    return 1
}

# Update pose monitoring (line 939, 1044):
# Replace /amcl_pose with slam_toolbox pose topic or use /tf for pose information
```

### Step 6: Gazebo Integration

For Gazebo simulation support, create simulation-specific parameters:

**File**: `yahboomcar_nav/params/slam_toolbox_sim_params.yaml`

```yaml
slam_toolbox:
  ros__parameters:
    # Inherit from slam_toolbox_params.yaml but override:
    use_sim_time: true
    
    # Simulation-specific tuning
    minimum_time_interval: 0.1  # Faster for simulation
    minimum_travel_distance: 0.2  # Smaller increments
    minimum_travel_heading: 0.2
    
    # More aggressive loop closure for simulation
    loop_search_maximum_distance: 5.0
    loop_match_minimum_response_coarse: 0.25
    loop_match_minimum_response_fine: 0.35
```

**Gazebo Launch Integration:**

```python
# In your Gazebo launch file, add:
use_sim_time = LaunchConfiguration('use_sim_time', default='true')

# Update slam_toolbox parameters for simulation
slam_params_file = LaunchConfiguration('slam_params_file', 
    default=os.path.join(package_path, 'params', 'slam_toolbox_sim_params.yaml'))
```

## Usage Instructions

### Starting the System

1. **Launch Micro-ROS Agent** (Step 1):
   ```bash
   ./b4m_HA_launch.sh --only-agent
   ```

2. **Power on Robot** (Step 2):
   - Physical robot power-on as usual

3. **Launch Robot Bringup** (Step 3):
   ```bash
   ros2 launch yahboomcar_bringup yahboomcar_bringup_launch.py
   ```

4. **Launch RViz** (Step 4):
   ```bash
   ros2 launch yahboomcar_nav display_launch.py
   ```

5. **Launch SLAM Navigation** (Step 5 - Modified):
   ```bash
   ros2 launch yahboomcar_nav slam_navigation_launch.py
   ```

6. **Skip Initial Pose** (Step 6 - No longer needed):
   - slam_toolbox handles localization automatically

7. **Launch Waypoint Navigation** (Step 7):
   ```bash
   python3 b4m_waypoint_nav/b4m_waypoint_nav/b4m_waypoint_nav.py --ros-args -p mqtt_broker:=192.168.68.111 -p mqtt_port:=1883 -p mqtt_username:=robot -p mqtt_password:=robot123
   ```

### SLAM vs Localization Modes

#### Mapping Mode (Initial Setup)
```bash
# Use for creating new maps
ros2 launch yahboomcar_nav slam_navigation_launch.py slam_mode:=mapping
```

#### Localization Mode (Normal Operation)
```bash
# Use for navigation with existing map
ros2 launch yahboomcar_nav slam_navigation_launch.py slam_mode:=localization map_file:=/path/to/saved_map.yaml
```

#### Lifelong Learning Mode (Adaptive)
```bash
# Use for continuous map improvement
ros2 launch yahboomcar_nav slam_navigation_launch.py slam_mode:=lifelong
```

## Integration with Existing B4M System

### MQTT Integration Compatibility

The existing B4M waypoint navigation system will work unchanged with slam_toolbox:

- **Waypoint commands** via MQTT continue to work
- **Navigation status** reporting unchanged  
- **Coordinate system** remains the same (map frame)
- **B4M Robot Manager GUI** requires no changes

### Transform Tree Changes

**Before (AMCL):**
```
map -> odom -> base_footprint -> base_link
            -> laser_frame
            -> imu_frame
```

**After (slam_toolbox):**
```
map -> odom -> base_footprint -> base_link  (slam_toolbox provides map->odom)
            -> laser_frame
            -> imu_frame
```

### Performance Considerations

- **CPU Usage**: slam_toolbox uses more CPU than AMCL
- **Memory Usage**: Map updates require additional memory
- **Loop Closure**: Occasional processing spikes during loop closure detection
- **Startup Time**: Slightly longer initialization time

## Testing and Validation

### Validation Steps

1. **Transform Chain Validation**:
   ```bash
   ros2 run tf2_ros tf2_echo map base_link
   ```

2. **Map Publication Check**:
   ```bash
   ros2 topic echo /map --once
   ```

3. **SLAM Status Monitoring**:
   ```bash
   ros2 topic echo /slam_toolbox/scan_visualization
   ```

4. **Navigation Testing**:
   ```bash
   ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"
   ```

### Integration Testing

Run the existing localization test framework with slam_toolbox:

```bash
./b4m_HA_launch.sh --autotest --localization-test
```

## Troubleshooting

### Common Issues

1. **Map Frame Errors**:
   - Ensure slam_toolbox is publishing map->odom transform
   - Check `/tf` topic for transform availability

2. **Performance Issues**:
   - Reduce `scan_buffer_size` in slam_toolbox params
   - Increase `minimum_time_interval` for slower processing

3. **Loop Closure Problems**:
   - Adjust `loop_match_minimum_response_*` parameters  
   - Ensure good sensor data quality

4. **Memory Usage**:
   - Enable `map_start_at_dock: true` for memory efficiency
   - Use `lifelong` mode cautiously in long-term operation

### Debug Commands

```bash
# Monitor SLAM performance
ros2 topic hz /scan
ros2 topic hz /map

# Check node status
ros2 node list | grep slam
ros2 service list | grep slam

# Save current map
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: emergency_map}}"
```

## Advanced Configuration

### Custom Map Saving

Add automatic map saving to your system:

```python
# In slam_navigation_launch.py, add:
Node(
    package='nav2_map_server',
    executable='map_saver_server',
    name='map_saver_server',
    output='screen',
    parameters=[{'use_sim_time': use_sim_time}],
),
```

### Performance Tuning for B4M Robot

Optimize parameters for your specific hardware:

```yaml
# In slam_toolbox_params.yaml
slam_toolbox:
  ros__parameters:
    # B4M-specific optimizations
    throttle_scans: 1
    scan_buffer_size: 5  # Reduce for limited CPU
    minimum_time_interval: 0.8  # Slower processing for stability
    link_match_minimum_response_fine: 0.2  # More lenient matching
```

---

This comprehensive guide provides all necessary components to successfully integrate slam_toolbox with your B4M Robot system while maintaining compatibility with existing MQTT navigation and GUI components.