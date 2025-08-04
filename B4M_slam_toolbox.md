# B4M SLAM Toolbox Integration Guide

## Goal

Get slam_toolbox running in **Gazebo simulation**, **RViz visualization**, and on the **real robot** to:
- Map the environment dynamically
- Navigate from waypoint to waypoint
- Replace the current AMCL/gmapping setup with a unified SLAM solution

## Launch and Shutdown Methods

**Primary Launch Method**: `./b4m_HA_launch.sh`
- Use `--simulation` flag for Gazebo testing
- Handles all startup sequences automatically
- Provides proper validation for each step

**Primary Shutdown Method**: `./b4m_shutdown.sh`
- Always use `--keep-agent` flag to preserve hardware connection
- Cleans up all processes safely
- Maintains Micro-ROS agent for real robot testing

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
- Works with existing EKF and Micro-ROS setup

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

### Current System Configuration

- **Launch Script**: `b4m_HA_launch.sh` coordinates the entire system
- **Navigation**: Currently uses AMCL with pre-built maps
- **Hardware**: ESP32 via Micro-ROS providing odometry, IMU, and laser data
- **Sensor Fusion**: EKF combines IMU and odometry data

## Implementation Steps

### Step 1: Create slam_toolbox Configuration

Create configuration files for slam_toolbox to work with both real robot and Gazebo simulation.

### Step 2: Create SLAM Launch Files

Create launch files for slam_toolbox to replace AMCL/gmapping.

### Step 3: Modify Navigation Launch

Update navigation launch to use slam_toolbox instead of AMCL.

### Step 4: Update Navigation Parameters

Remove AMCL configuration from navigation parameters.

### Step 5: Update Launch Script

Modify b4m_HA_launch.sh to use slam_toolbox instead of AMCL/gmapping.

### Step 6: Gazebo Integration

Create simulation-specific configuration for running slam_toolbox in Gazebo.

## Usage Instructions

### For Real Robot Testing

```bash
# Launch the full system with SLAM
./b4m_HA_launch.sh --autotest --debug

# When finished, shutdown (preserving hardware connection)
./b4m_shutdown.sh --keep-agent
```

### For Gazebo Simulation Testing

```bash
# Launch in simulation mode
./b4m_HA_launch.sh --simulation --autotest --debug

# When finished, shutdown
./b4m_shutdown.sh 
```

### Key Points
- No initial pose setting required - SLAM localizes automatically
- Map builds dynamically as robot moves
- Works seamlessly in both simulation and real robot modes

### Operating Modes

- **Mapping Mode**: Creates new maps while navigating
- **Localization Mode**: Uses existing maps for navigation
- **Lifelong Mode**: Continuously improves maps during operation

## Key Integration Points

- **MQTT/Waypoint Navigation**: Works unchanged with slam_toolbox
- **Transform Tree**: slam_toolbox provides map->odom transform
- **GUI Compatibility**: B4M Robot Manager needs no modifications

## Testing and Validation

- Verify slam_toolbox is publishing map->odom transform
- Check map updates in RViz
- Test waypoint navigation functionality
- Monitor performance in both real and simulated environments

## Troubleshooting

- **Transform issues**: Check slam_toolbox is running and publishing transforms
- **Performance**: Adjust scan_buffer_size and minimum_time_interval
- **Map quality**: Tune loop closure parameters for better results

## Next Steps

1. Install slam_toolbox package
2. Create configuration files for both real robot and Gazebo
3. Update launch files to replace AMCL with slam_toolbox
4. Test using `./b4m_HA_launch.sh --simulation` for Gazebo
5. Test using `./b4m_HA_launch.sh` for real robot
6. Always shutdown with `./b4m_shutdown.sh --keep-agent`

This guide ensures slam_toolbox works across all three environments:
- **Gazebo**: Full simulation environment for testing
- **RViz**: Visualization of SLAM mapping and navigation
- **Real Robot**: Actual hardware deployment
