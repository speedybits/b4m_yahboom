# B4M Robot System User Guide

## Overview

The B4M Robot is an autonomous navigation system built on ROS2 with Home Assistant integration. This guide covers how to use the `b4m_HA_launch.sh` script to launch and operate the robot system.

## Prerequisites

- ROS2 Humble installed and sourced
- Docker (for Micro-ROS agent)
- Physical robot or Gazebo simulation environment
- Built workspace (`colcon build` completed)

## Quick Start

### Basic Interactive Launch
```bash
./b4m_HA_launch.sh
```
Launches the full system with user prompts for each step.

### Simulation Mode
```bash
./b4m_HA_launch.sh --simulation
```
Runs the robot in Gazebo simulation instead of using physical hardware.

### Automated Testing
```bash
./b4m_HA_launch.sh --simulation --autotest
```
Runs all steps automatically for testing and validation.

## Command Line Options

| Option | Description |
|--------|-------------|
| `--skip-agent` | Skip the Micro-ROS agent launch (Step 1) |
| `--only-agent` | Launch ONLY the Micro-ROS agent and exit |
| `--autotest` | Run in automated test mode (non-interactive) |
| `--debug` | Enable verbose debug logging |
| `--simulation` | Launch in Gazebo simulation mode |
| `--slam-test` | Add automated SLAM testing steps (implies --autotest) |
| `-h, --help` | Show help information |

## Usage Scenarios

### 1. Interactive SLAM Mapping in Simulation
```bash
./b4m_HA_launch.sh --simulation
```
Then use keyboard teleop to drive the robot:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 2. Automated System Validation
```bash
./b4m_HA_launch.sh --simulation --autotest --slam-test
```
Runs full automated test suite including square navigation pattern.

### 3. Quick System Check
```bash
./b4m_HA_launch.sh --simulation --autotest --debug
```
Validates all components with verbose output for troubleshooting.

### 4. Real Robot Testing
```bash
./b4m_HA_launch.sh --autotest --debug
```
Tests with physical robot (requires robot to be powered on).

### 5. Development/Debug Mode
```bash
./b4m_HA_launch.sh --simulation --debug
```
Interactive mode with verbose logging for development work.

## System Launch Steps

The script executes the following steps in sequence:

### Real Robot Mode

1. **Micro-ROS Agent**: Establishes communication with ESP32 microcontroller
2. **Robot Power**: Manual step to power on physical robot
3. **Data Processing**: Launches sensor integration and robot state publisher
4. **RViz Visualization**: Starts visualization interface
5. **Navigation System**: Launches SLAM-based navigation
6. **SLAM Initialization**: Monitors SLAM system startup
7. **MQTT Navigation**: Starts waypoint navigation with Home Assistant integration
8. **Robot Manager GUI**: Launches visual control interface (interactive mode only)

### Simulation Mode

1. **Gazebo Launch**: Starts Ignition Gazebo simulation environment
2. **Robot Spawning**: Spawns robot model with controllers in simulation
3. **Robot Systems**: Initializes robot state and sensor systems
4. **RViz Visualization**: Starts visualization with simulation time
5. **SLAM Navigation**: Launches SLAM system for simulation
6. **SLAM Initialization**: Initializes SLAM in simulation environment
7. **MQTT Navigation**: Starts waypoint navigation system
8. **Robot Manager GUI**: Visual control interface (if not in autotest mode)

### SLAM Testing Mode (--slam-test)

When enabled, adds these automated testing steps:

8. **Automated Square Movement**: Robot executes 1-meter square pattern for mapping
9. **Map Validation**: Saves and validates the generated map
10. **MQTT Navigation Test**: Tests Home Assistant integration functionality

## Operation Modes

### Interactive Mode (Default)
- Each step opens in a separate terminal window
- User confirmation required between steps
- Terminals remain open for monitoring and debugging
- Ideal for development and troubleshooting

### Autotest Mode (--autotest)
- All steps execute automatically
- Built-in validation for each step
- Automatic cleanup on failure
- 60-second timeout per step (120s for navigation)
- Ideal for CI/CD and automated testing

## System Requirements

### Hardware Mode
- Physical Yahboom robot
- ESP32 microcontroller with Micro-ROS firmware
- LIDAR sensor for navigation
- IMU for pose estimation

### Simulation Mode
- Ignition Gazebo installed
- Sufficient system resources for simulation
- Display server (X11) for RViz visualization

## Logging and Monitoring

### Log Files
All execution logs are saved to the `logs/` directory:
- Main log: `logs/b4m_launch_YYYYMMDD_HHMMSS.log`
- Step-specific logs: `logs/stepN_description_YYYYMMDD_HHMMSS.log`

### Process Monitoring
The script includes built-in process monitoring:
- Duplicate node detection
- Resource usage warnings
- Automatic cleanup on conflicts

## Troubleshooting

### Common Issues

**RViz fails to start**
- Check DISPLAY environment variable
- Ensure X11 forwarding if using SSH
- In autotest mode, virtual display is automatically configured

**Navigation system doesn't activate**
- Verify all sensor data is available (`ros2 topic list`)
- Check transform tree (`ros2 run tf2_tools view_frames`)
- Monitor logs for lifecycle manager errors

**Robot doesn't respond to commands**
- Verify Micro-ROS agent connection
- Check ESP32 firmware and network configuration
- Ensure robot is powered and connected

### Debug Commands
```bash
# Check active ROS2 nodes
ros2 node list

# Monitor topics
ros2 topic list
ros2 topic echo /scan

# Check transforms
ros2 run tf2_tools view_frames

# Monitor logs
tail -f logs/b4m_launch_*.log
```

## Cleanup

### Automatic Cleanup
The script includes automatic cleanup for:
- Duplicate process detection
- Failed step recovery
- Autotest mode failures

### Manual Cleanup
Always use the shutdown script when finished:
```bash
./b4m_shutdown.sh --keep-agent
```

**Important**: Always use `--keep-agent` flag to preserve hardware connection.

## Integration with Home Assistant

The system integrates with Home Assistant via MQTT:

### MQTT Topics
- `yahboom/navigation/command`: Send navigation commands
- `yahboom/navigation/status`: Receive navigation status
- `yahboom/navigation/info`: System information updates

### Command Format
```json
{
  "command": "goto",
  "waypoint_id": "living_room",
  "position": {"x": 2.5, "y": 1.0},
  "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
}
```

## Safety Considerations

- Always supervise robot operation, especially during initial testing
- Ensure clear navigation paths free of obstacles
- Monitor system logs for error conditions
- Use simulation mode for initial testing and development
- Keep emergency stop procedures readily available

## Support

For additional help:
- Check log files in the `logs/` directory
- Use `--debug` flag for verbose output
- Refer to individual component documentation in the workspace
- Monitor ROS2 system status with standard ROS2 tools