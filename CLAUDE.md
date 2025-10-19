# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Secret keys
Never use the actual value of environment variables located in the .bashrc in the code you write. Only use the environment variables themselves

## System File Modifications
**NEVER modify system files (like ~/.bashrc, ~/.bash_profile, /etc files) without explicit user permission.**

**Requirements:**
- Always create a timestamped backup BEFORE modifying any system file
- Ask for user permission before modifying files outside the git repository
- Document what changes will be made and why
- Provide clear restoration instructions if changes are made

**Backup naming convention:**
```bash
cp ~/.bashrc ~/.bashrc.backup.$(date +%Y%m%d_%H%M%S)
```

## Commits
Never commit files automatically

## Application Control Standards

### Graceful Shutdown Requirement
**All Python applications must implement proper CTRL+C (SIGINT) handling for graceful shutdown.**

**Requirements:**
- Applications must respond to CTRL+C within 100-500ms maximum
- All running threads must be cleanly terminated
- Any temporary files should be cleaned up
- Audio streams and hardware resources must be properly released
- Buffer contents should be saved if needed
- Application should return control to terminal cleanly

**Implementation:**
- Use `signal.signal(signal.SIGINT, handler)` to register shutdown handler
- Use `threading.Event()` for coordinated shutdown across all threads
- Replace blocking operations with interruptible polling loops
- Check shutdown event frequently during long-running operations

## Implementation Standards

### No Placeholder Code
**Never use placeholder code, mock implementations, or incomplete functionality in production code.**
**Never mock elements of a unit test

**Prohibited Examples:**
- `# Real implementation would go here`
- `# TODO: Implement actual functionality`
- `pass  # Placeholder`
- Simulated behavior when real implementation is expected
- Mock objects in production code paths

**Requirements:**
- All code must be fully functional when deployed
- If a feature cannot be implemented completely, it should not be included
- Use actual libraries, APIs, and integrations - not simulations
- Error handling should be real, not simulated

**Testing vs Implementation:**
- **Tests**: Mocking external dependencies (MQTT brokers, ROS services) is acceptable and encouraged
- **Implementation**: Never mock or simulate - use actual functionality
- If external dependency is unavailable, provide clear setup instructions rather than fake implementations

## Test file location
Any new test file created should be located in the 'tests' directory

### Testing Best Practices
- **Test Real Behavior**: Tests must use actual components, not mocks, to detect real issues
- **Integration Tests Required**: Test MQTT integration with actual message handling
- **Mock Only External Dependencies**: Mock MQTT broker, ROS services, etc. but test core navigation logic
- **Test Failure Scenarios**: Write tests that expose bugs in navigation and MQTT handling
- **Validate Against Real Usage**: If user reports a bug that tests don't catch, the tests are wrong, not the user

## Build Commands

Please remember to re-run colcon build whenever necessary after making changes to code

**CRITICAL: After rebuilding, you must restart any running ROS2 nodes/processes to load the new code. Python processes do not automatically reload changed modules.**

## MQTT Integration Development

### MQTT Command Handling
- **JSON Format**: All MQTT commands use structured JSON format
- **Topic Structure**: Uses `yahboom/navigation/*` topic hierarchy
- **Status Updates**: Real-time feedback via status topics
- **Error Handling**: Comprehensive error reporting via MQTT

### Home Assistant Integration
- **Coordinate-based Commands**: Direct position/orientation specification
- **Real-time Status**: Live navigation progress updates
- **Error Reporting**: Detailed error messages for troubleshooting

### ROS2 Workspace Build
```bash
# Build the main workspace
colcon build --symlink-install

# Build specific packages
colcon build --packages-select package_name

# Build with parallel jobs
colcon build --parallel-workers 4
```

### Environment Setup
```bash
# Source ROS2 and all workspaces
source source_workspaces.sh

# Source only the main workspace (without external dependencies)
source setup_env.sh
```

### Testing and Linting
The project uses standard ROS2 testing patterns:
```bash
# Run tests for specific package
colcon test --packages-select package_name

# Run all tests
colcon test
```

## Architecture Overview

### ROS2 Multi-Workspace Structure
This repository contains a complex ROS2 system with multiple workspaces:

- **Main workspace**: Core Yahboom robot packages at the root level
- **gmapping_ws**: SLAM mapping packages (openslam_gmapping, slam_gmapping)
- **imu_ws**: IMU sensor processing packages (imu_tools, filters)
- **uros_ws**: Micro-ROS agent for ESP32 communication

### Key Components

#### Robot Control & Navigation
- **yahboomcar_bringup**: Core robot launch files and sensor integration
- **yahboomcar_nav**: Navigation stack including waypoint navigation, mapping, and localization
- **yahboomcar_ctrl**: Robot control interfaces (joystick, keyboard)
- **b4m_waypoint_nav**: Advanced waypoint navigation with MQTT integration

#### Hardware Integration
- **yahboomcar_base_node**: Low-level hardware interface (C++)
- **yahboom_esp32_camera**: ESP32 camera integration
- **yahboom_esp32_mediapipe**: Computer vision with MediaPipe
- **yahboom_esp32ai_car**: AI-powered robot control features

#### SLAM & Mapping
- **Gmapping**: Traditional laser-based SLAM
- **Cartographer**: Google's SLAM implementation
- **Nav2**: Modern ROS2 navigation stack

### System Architecture Flow

1. **Hardware Layer**: ESP32 microcontroller communicates via Micro-ROS
2. **Communication Layer**: Micro-ROS agent bridges ESP32 to ROS2
3. **Sensor Layer**: IMU, LIDAR, camera data processing
4. **Navigation Layer**: SLAM mapping, localization, path planning
5. **Control Layer**: Waypoint navigation, teleoperation, autonomous behavior
6. **Integration Layer**: MQTT integration for Home Assistant

### Launch Sequence
The system follows a specific startup sequence (automated in `b4m_launch.sh`):
1. Start Micro-ROS agent for ESP32 communication
2. Power on physical robot
3. Launch robot bringup (sensor integration)
4. Start visualization (RViz)
5. Launch navigation system
6. Initialize robot pose
7. Start waypoint navigation with MQTT
8. System ready for operation

## Common Development Patterns

### ROS2 Package Structure
- Most packages follow standard ROS2 Python package layout
- Launch files in `launch/` directory
- Parameters in `param/` or `params/` directory
- RViz configs in `rviz/` directory
- Maps stored in `maps/` directory

### Waypoint Management System
- **Single map approach**: All waypoints stored under `yahboom_map` key
- **Data storage**: JSON format at `/home/yahboom/b4m_yahboom/install/b4m_waypoint_nav/waypoints.json`
- **MQTT Interface**: Command-based waypoint control via Home Assistant
- **Coordinate-based navigation**: Direct coordinate sending via MQTT eliminates waypoint lookup
- **Waypoint structure**: Includes position, orientation, timestamp, and visualization properties
- **Map format support**: Compatible with both Gmapping and Cartographer maps
- **Coordinate transformation**: Proper ROS coordinate system handling (bottom-left origin)

### Multi-Robot Support
The system supports multi-robot configurations:
- **yahboomcar_multi**: Multi-robot navigation and coordination
- Separate parameter files for robot1, robot2, etc.
- Namespace-based separation of robot instances

### Custom Message Types
- **yahboomcar_msgs**: Custom message definitions for robot-specific data
- **yahboom_web_savmap_interfaces**: Web interface message types

## File Organization

### Configuration Files
- `waypoints.json`: Stored waypoint data
- `*.yaml` files: ROS2 parameters and map configurations
- `*.rviz`: RViz visualization configurations

### Scripts & Utilities
- `b4m_launch.sh`: Automated launch sequence for MQTT/Home Assistant integration
- `source_workspaces.sh`: Environment setup for all workspaces
- `setup_env.sh`: Simplified environment setup
- `map_selector.sh`: Map selection utility

### Documentation
- `GIT_README.md`: Repository structure and git workflow
- `WORKSPACE_README.md`: Workspace usage instructions
- `B4M_*.md`: Project-specific documentation

## Development Notes

### Workspace Dependencies
The system requires proper workspace sourcing order:
1. ROS2 Humble base environment
2. Main workspace
3. Gmapping workspace
4. IMU workspace
5. Micro-ROS workspace

### Known Issues
- **rviz_imu_plugin**: May fail to build due to tinyxml2 linking issues
- **Path dependencies**: Always use provided sourcing scripts to ensure correct paths
- **Multi-workspace complexity**: Changes to one workspace may affect others

### MQTT Integration
The B4M system integrates with Home Assistant via MQTT:
- **Topics**: `yahboom/navigation/command`, `yahboom/navigation/status`, `yahboom/navigation/info`
- **Command formats**: 
  - Coordinate-based (preferred): `{"command": "goto", "waypoint_id": "name", "position": {"x": float, "y": float}, "orientation": {"x": float, "y": float, "z": float, "w": float}}`
  - Simple waypoint: `{"command": "goto", "waypoint_id": "waypoint_name"}`
- **Status updates**: Real-time navigation progress and completion status
- **Dynamic waypoint management**: Coordinates sent directly via MQTT without navigation node restart

### Hardware Dependencies
- **ESP32**: Micro-ROS compatible firmware required
- **LIDAR**: Laser scanner for SLAM and navigation
- **IMU**: Inertial measurement unit for pose estimation
- **Camera**: Optional vision processing capabilities

### Odometry Architecture
The system uses a two-stage odometry pipeline for accurate position tracking:
1. **Raw Odometry (`/odom_raw`)**: Published by ESP32/micro-ROS with raw encoder data
2. **Extended Kalman Filter (EKF)**: Fuses `/odom_raw` with IMU data for noise reduction
3. **Filtered Odometry (`/odom`)**: Clean output from EKF used by navigation stack

**Critical**: The EKF requires ALL workspaces to be sourced (especially `imu_ws`) to function.
Without proper workspace sourcing, the EKF won't start and `/odom` won't be published.

### MQTT Command Development
- **JSON Validation**: Ensure all MQTT commands follow proper JSON schema
- **Error Handling**: Comprehensive error reporting via MQTT status topics
- **Coordinate Validation**: Verify position/orientation data before navigation
- **Status Reporting**: Real-time progress updates for Home Assistant
- **Connection Management**: Robust MQTT broker connection handling

#### B4M Robot Manager - Central Launch Control
The B4M Robot Manager serves as the central application for system control, integrating the launch and shutdown sequences directly into the GUI:

**Control Panel Features:**
- **Three primary buttons**: Rebuild, Start, Stop
- **Smart state management**: Buttons are enabled/disabled based on system state
- **Launch sequence integration**: Executes b4m_launch.sh steps programmatically
- **Shutdown integration**: Executes b4m_shutdown.sh sequence when stopping
- **Status display**: Shows current step in launch sequence below control buttons
- **Parameter protection**: Navigation parameters become read-only during active system operation

**Button State Logic:**
- **Initial state**: Only "Start" button enabled
- **During launch**: "Start" disabled, "Stop" enabled, status shows current step
- **Parameter changes**: "Rebuild" becomes enabled when GUI changes require colcon build
- **Active system**: Navigation parameters section becomes read-only (browsable but not editable)

**Launch Sequence Steps (from b4m_launch.sh):**
1. Start Micro-ROS Agent (Docker container)
2. Power on physical robot (manual confirmation)
3. Launch car's underlying data processing (sensor integration)
4. Start RViz for visualization
5. Launch navigation system with pre-built map
6. Initial robot positioning (manual pose estimation)
7. Start B4M Waypoint Navigation Node with MQTT
8. Complete - Robot Manager GUI ready for operation

**Shutdown Sequence Steps (from b4m_shutdown.sh):**
1. Stop all ROS2 nodes
2. Force kill remaining ROS2 processes if needed
3. Stop waypoint navigation and robot manager processes
4. Leave Micro-ROS agent running (ALWAYS use --keep-agent)
5. Clean up remaining Python processes
6. Stop RViz if running

**CRITICAL SHUTDOWN RULE:**
- **ALWAYS use `./b4m_shutdown.sh --keep-agent`** - NEVER shut down the Micro-ROS agent
- The Micro-ROS agent provides the critical hardware connection to the physical robot
- Shutting down the agent requires physical robot restart and reconnection
- Use `--keep-agent` flag to preserve the hardware connection during system restarts

## Simulation Environment

The system uses **Gazebo Classic 11.10.2** as the primary simulation platform:

### Simulation Launch
- **Primary command**: `./b4m_launch.sh --simulation`
- **Test mode**: `./b4m_launch.sh --simulation --autotest`
- **SLAM testing**: `./b4m_launch.sh --simulation --slam-test`

### Gazebo Classic Features
- **Integrated robot spawning**: Robot model is automatically spawned in `gazebo_classic_nav_launch.py`
- **Working laser sensors**: Reliable sensor data for SLAM (360-point scans)
- **Differential drive control**: Direct `/cmd_vel` topic control
- **ROS2 Humble integration**: Mature plugin ecosystem with stable performance

### Simulation Files
- **World files**: `yahboomcar_nav/worlds/navigation_test_classic.world`
- **Robot URDF**: `yahboomcar_description/urdf/yahboomcar_robot_classic_nav.urdf`
- **Launch files**: `yahboomcar_nav/launch/gazebo_classic_nav_launch.py`

### SLAM Integration
- **Map generation**: 69x77 occupancy grid maps
- **SLAM launch**: Uses `slam_test_gazebo_classic.py` for dedicated SLAM testing
- **Real-time mapping**: Keyboard teleop with `teleop_twist_keyboard` for map building

## Testing Strategy

### Unit Testing
Standard ROS2 testing framework with pytest for Python nodes and gtest for C++ nodes.

### Integration Testing
- End-to-end navigation tests using Gazebo Classic simulation
- SLAM mapping validation with 69x77 grid verification
- Multi-robot coordination tests
- MQTT command/status validation

### Hardware Testing
- Sensor integration validation
- Motor control verification
- Communication link testing
- ESP32 Micro-ROS connectivity
