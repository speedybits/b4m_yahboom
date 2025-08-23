# B4M Robot System User Guide

## Overview

The B4M Robot is an autonomous navigation system built on ROS2 with optional MQTT/Home Assistant integration. This guide covers how to use the `b4m_launch.sh` script to launch and operate the robot system with SLAM (Simultaneous Localization and Mapping) capabilities using Gazebo Classic for simulation.

## Prerequisites

- ROS2 Humble installed and sourced
- Docker (for Micro-ROS agent)
- Physical robot or Gazebo Classic simulation environment
- Built workspace (`colcon build` completed)
- For regression testing: python3-opencv and python3-skimage (for image comparison)

## Quick Start

### First Time Setup (WiFi Configuration)
```bash
./b4m_launch.sh --setup-wifi
```
**New users should start here!** This interactive wizard guides you through:
- Connecting the robot via USB
- Configuring WiFi credentials
- Setting up mDNS hostname or fixed IP connection
- Testing the robot configuration

### Basic Interactive Launch
```bash
./b4m_launch.sh
```
Launches the core robot system without MQTT/Home Assistant features.

### With Home Assistant Integration
```bash
./b4m_launch.sh --b4m-HA
```
Launches the full system including MQTT and Home Assistant integration.

### Simulation Mode
```bash
./b4m_launch.sh --simulation
```
Runs the robot in Gazebo Classic simulation instead of using physical hardware.

### Regression Testing
```bash
./b4m_launch.sh --simulation --regression
```
Runs comprehensive regression test suite with automated validation.

## Command Line Options

| Option | Description |
|--------|-------------|
| `--setup-wifi` | **Interactive WiFi setup wizard for robot configuration** |
| `--skip-agent` | Skip the Micro-ROS agent launch (Step 1) |
| `--only-agent` | Launch ONLY the Micro-ROS agent and exit |
| `--debug` | Enable verbose debug logging |
| `--simulation` | Launch in Gazebo Classic simulation mode |
| `--regression` | Run comprehensive regression test suite (navigation + laser stability) |
| `--explore` | Enable autonomous exploration mode with obstacle avoidance |
| `--b4m-api` | Enable B4M API mode (duplicate of --explore for API integration) |
| `--b4m-HA` | Enable Home Assistant MQTT integration features |
| `--b4m-ping` | Test bike4mind API with random obstacle detection messages |
| `--navigation-performance-test` | (Experimental) Execute 1x1m square navigation circuit testing |
| `-h, --help` | Show help information |

## Home Assistant Integration (--b4m-HA)

The `--b4m-HA` flag controls whether MQTT and Home Assistant features are enabled:

### When --b4m-HA is ENABLED:
- Step 7 launches the B4M waypoint navigation node with MQTT parameters
- MQTT command interface for remote waypoint management
- Full Home Assistant integration via MQTT topics
- Waypoint navigation with coordinate-based commands
- Real-time status updates to Home Assistant
- MQTT command interface for system integration

### When --b4m-HA is NOT provided (default):
- Step 7 is skipped
- No MQTT connectivity required
- No Home Assistant integration
- Basic robot navigation without external control
- Suitable for standalone operation or development
- Reduces system complexity and dependencies

### Example Commands:
```bash
# Basic operation without Home Assistant
./b4m_launch.sh --simulation

# Full Home Assistant integration
./b4m_launch.sh --simulation --b4m-HA

# Regression testing (doesn't need MQTT)
./b4m_launch.sh --simulation --regression

# Exploration with Home Assistant monitoring
./b4m_launch.sh --explore --b4m-HA
```

## Usage Scenarios

### 📋 Quick Reference: Real Robot vs Simulation
| Use Case | Command | What Happens |
|----------|---------|--------------|
| **Explore with REAL robot** | `./b4m_launch.sh --explore` | Physical robot explores real environment |
| **Explore in simulation** | `./b4m_launch.sh --simulation --explore` | Simulated robot explores in Gazebo Classic |
| **Regular real robot launch** | `./b4m_launch.sh` | Basic robot setup without MQTT |
| **Real robot with Home Assistant** | `./b4m_launch.sh --b4m-HA` | Full system with MQTT/HA integration |
| **Regular simulation launch** | `./b4m_launch.sh --simulation` | Basic simulation without MQTT |
| **Simulation with Home Assistant** | `./b4m_launch.sh --simulation --b4m-HA` | Full simulation with MQTT/HA |

### 1. Interactive SLAM Mapping in Simulation
```bash
# Using integrated launch with Gazebo Classic
./b4m_launch.sh --simulation

# Or using direct Gazebo Classic launch for SLAM testing
ros2 launch yahboomcar_nav slam_test_gazebo_classic.py
```
Then use keyboard teleop to drive the robot:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Monitor SLAM map generation (69x77 grid) in RViz.

### 2. Automated System Validation
```bash
./b4m_launch.sh --simulation --regression
```
Runs comprehensive regression test suite including:
- 360-degree rotation test with laser scan stability validation
- Automated image comparison for RViz visualization
- SLAM map generation verification
- Full system health checks
- Note: MQTT testing only if --b4m-HA is also provided

### 3. Quick System Check
```bash
./b4m_launch.sh --simulation --regression --debug
```
Validates all components with verbose output for troubleshooting, including regression tests.

### 4. Real Robot Testing
```bash
./b4m_launch.sh --regression --debug
```
Runs regression tests with physical robot (requires robot to be powered on).

### 5. Development/Debug Mode
```bash
./b4m_launch.sh --simulation --debug
```
Interactive mode with verbose logging for development work.

### 6. B4M API Mode

#### Real Robot with B4M API
```bash
./b4m_launch.sh --b4m-api
```
Physical robot runs B4M Spatial Interpreter for manual navigation decisions.

#### Simulation with B4M API
```bash
./b4m_launch.sh --simulation --b4m-api
```
Simulated robot demonstrates spatial interpretation in Gazebo.

**B4M API Features:**
- **Spatial Interpretation**: Generates natural language descriptions of environment
- **Manual Decision Mode**: User provides navigation decisions via console
- **SLAM Integration**: Real-time mapping with Cartographer
- **Obstacle Detection**: Uses laser scanner data for spatial awareness
- **Interactive Console**: Displays spatial descriptions and waits for user input

### 7. Autonomous Exploration Mode

#### Real Robot Exploration (Physical Robot)
```bash
./b4m_launch.sh --explore
```
**Physical robot** autonomously explores the real environment with obstacle avoidance while building a SLAM map.

#### Gazebo Classic Simulation Exploration
```bash
./b4m_launch.sh --simulation --explore
```
**Simulated robot** autonomously explores the Gazebo Classic simulation environment with obstacle avoidance while building a SLAM map.

**Important:** 
- `--explore` alone = **REAL PHYSICAL ROBOT** exploration
- `--explore --simulation` = **GAZEBO SIMULATION** exploration

**Exploration Features:**
- **Safe movement**: Very slow speed (0.08 m/s) for safe exploration
- **Obstacle avoidance**: Uses laser scanner to detect and avoid obstacles
- **Random exploration**: Changes direction periodically to explore thoroughly
- **SLAM integration**: Builds map in real-time during exploration
- **Continuous operation**: Runs until manually stopped with Ctrl+C
- **Works in both modes**: Identical behavior in simulation and real robot

## System Launch Steps

The script executes the following steps in sequence:

### Real Robot Mode

1. **Micro-ROS Agent**: Establishes communication with ESP32 microcontroller
2. **Robot Power**: Manual step to power on physical robot
3. **Data Processing**: Launches sensor integration and robot state publisher
4. **RViz Visualization**: Starts visualization interface
5. **Navigation System**: Launches SLAM-based navigation
6. **SLAM Initialization**: Monitors SLAM system startup
7. **MQTT Navigation**: Starts waypoint navigation with Home Assistant integration (only if --b4m-HA enabled)

### Simulation Mode (`--simulation`)

Uses Gazebo Classic simulation environment:

1. **Gazebo Classic Launch**: Starts Gazebo Classic simulation with integrated robot spawning
2. **Robot Initialization**: Robot systems handled by integrated launch file
3. **Robot Systems**: Robot state publisher and transforms active
4. **RViz Visualization**: Starts visualization with laser scan display
5. **SLAM Navigation**: Launches SLAM toolbox for mapping and localization
6. **SLAM Initialization**: Verifies SLAM map generation (69x77 grid)
7. **MQTT Navigation**: Starts waypoint navigation system (only if --b4m-HA enabled)

### Exploration Mode (`--explore`)

Available in both simulation and real robot modes:

1. **System Launch**: Same as above (steps 1-7)
2. **Autonomous Explorer**: Launches obstacle-avoiding exploration script
   - Robot moves slowly (0.08 m/s) for safety
   - Uses laser scanner for obstacle detection (40cm safe distance)
   - Changes direction randomly every 8-15 seconds for thorough exploration
   - Turns away when obstacles detected within 60cm
   - Integrates with SLAM for real-time mapping
   - Runs continuously until Ctrl+C

### Special Testing Modes

#### B4M API Mode (--b4m-api)
Enables B4M Spatial Interpreter for manual navigation decisions:
- Generates natural language spatial descriptions
- Interactive console for user navigation input
- Integrates with Cartographer SLAM for mapping
- Processes laser scanner data for obstacle detection
- Functionally identical to --explore but with manual control
- Incompatible with other navigation modes

#### B4M Ping Mode (--b4m-ping)
Standalone API testing tool that:
- Tests bike4mind API connectivity
- Sends random obstacle detection messages
- Validates API response handling
- Incompatible with other modes


#### Navigation Performance Test (--navigation-performance-test) [Experimental]
Executes comprehensive 1x1m square navigation testing:
- Precise movement patterns
- Performance metrics collection
- Automatically enables localization testing
- Validates navigation stack accuracy

## Operation Modes

### Interactive Mode (Default)
- Each step opens in a separate terminal window
- User confirmation required between steps
- Terminals remain open for monitoring and debugging
- Ideal for development and troubleshooting

### Regression Mode (--regression)
- Comprehensive test suite execution
- Automated laser scan stability validation
- RViz screenshot comparison against reference images:
  - Captures screenshots at 3 key moments: initial, mid-rotation (180°), and final (360°)
  - Compares against mode-appropriate reference screenshots with 90% similarity threshold
  - Uses multi-method analysis: histogram, SSIM, feature matching, and template matching
  - Ensures laser scan visualization and SLAM mapping remain consistent
  - Test fails if screenshots differ by more than 10% from reference
  - Automatically detects simulation vs real robot mode
- Built-in validation for each step
- Cleanup at START (not end) to ensure clean state
- System left running after tests for debugging
- Uses filtered /odom topic from EKF, not raw /odom_raw
- Manual cleanup available: ./b4m_shutdown.sh --keep-agent
- Ideal for CI/CD and quality assurance

## System Requirements

### Hardware Mode
- Physical Yahboom robot
- ESP32 microcontroller with Micro-ROS firmware
- LIDAR sensor for navigation
- IMU for pose estimation

### Simulation Mode
- **Gazebo Classic**: Gazebo Classic 11 installed (`ros-humble-gazebo-*`)
- Sufficient system resources for simulation
- Display server (X11) for RViz visualization

### Simulation Environment

**Gazebo Classic (`--simulation`)**:
- ✅ **Reliable sensor data**: Proven laser sensor data publication for SLAM
- ✅ **Stable performance**: Mature, well-tested simulation physics
- ✅ **Integrated launches**: Robot spawning handled automatically in launch files
- ✅ **SLAM compatibility**: Consistent 360-point laser scans for mapping
- ✅ **ROS2 Humble integration**: Mature plugin ecosystem with reliable performance

## Navigation Testing

### Manual Navigation Tests
```bash
# Test 1-meter square navigation with Gazebo Classic
python3 tests/navigation/test_square_corners.py
```

### Automated Navigation Tests
The Gazebo Classic simulation supports the full automated test suite with navigation validation.

## SLAM Map Creation and Saving

### Creating Maps in Simulation (Gazebo Classic)

#### Step 1: Launch SLAM System
```bash
# Launch full system with SLAM in Gazebo Classic
./b4m_launch.sh --simulation

# Or launch dedicated SLAM test environment
ros2 launch yahboomcar_nav slam_test_gazebo_classic.py
```

#### Step 2: Drive Robot to Create Map
```bash
# In a separate terminal, launch keyboard teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Drive the robot around the environment to create a comprehensive map:
- Use **i** to move forward
- Use **j** and **l** to turn left/right  
- Use **k** to stop
- Use **u**, **o**, **m**, **.** for diagonal movement
- Use **q**/**z** to increase/decrease speed

#### Step 3: Monitor Map Generation
```bash
# Check map data in real-time
ros2 topic echo /map --once

# View map visualization in RViz
# The map should appear as a 69x77 occupancy grid
```

#### Step 4: Save the Map
```bash
# Save map using ROS2 map_server
ros2 run nav2_map_server map_saver_cli -f my_simulation_map

# This creates two files:
# - my_simulation_map.pgm (image file)
# - my_simulation_map.yaml (metadata file)
```

### Creating Maps with Real Robot

#### Step 1: Launch SLAM System on Real Robot
```bash
# Launch full system on physical robot
./b4m_launch.sh

# Ensure robot is powered on and Micro-ROS agent is connected
```

#### Step 2: Drive Robot to Map Environment
```bash
# Use keyboard teleop to drive robot
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Manual waypoint control is available via MQTT commands when --b4m-HA is enabled
```

**Important mapping tips:**
- Drive slowly and methodically
- Ensure good laser scanner visibility
- Cover all areas you want mapped
- Make overlapping paths for better map quality
- Close loops by returning to starting positions

#### Step 3: Monitor Real-time Mapping
```bash
# Monitor laser scan data (should see 360 points)
ros2 topic echo /scan

# Check map generation
ros2 topic echo /map --once

# View live mapping in RViz
# Watch the map build as you drive around
```

#### Step 4: Save the Real Robot Map
```bash
# Save the map when mapping is complete
ros2 run nav2_map_server map_saver_cli -f my_real_robot_map

# Creates:
# - my_real_robot_map.pgm
# - my_real_robot_map.yaml
```

### Using Saved Maps

#### Loading Maps for Navigation
```bash
# Copy map files to the maps directory
cp my_real_robot_map.* install/yahboomcar_nav/share/yahboomcar_nav/maps/

# Update launch files to use your map
# Edit params/nav2_params.yaml to reference your map file
```

#### Map File Format
The `.yaml` file contains map metadata:
```yaml
image: my_real_robot_map.pgm
mode: trinary
resolution: 0.05
origin: [-10.0, -10.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
```

### SLAM Services and Commands

#### Available SLAM Services
```bash
# List all SLAM toolbox services
ros2 service list | grep slam_toolbox

# Key services:
# /slam_toolbox/save_map - Save current map
# /slam_toolbox/clear_changes - Clear map changes
# /slam_toolbox/serialize_map - Serialize map for later use
```

#### Save Map via Service Call
```bash
# Save map using SLAM toolbox service
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: 'my_saved_map'}}"
```

#### Map Quality Verification
```bash
# Check map dimensions and data
ros2 topic echo /map --once | grep -E "(width|height|resolution)"

# Expected for good maps:
# - Resolution: ~0.05 meters per pixel
# - Width/Height: Depends on mapped area
# - Data should contain mix of 0 (free), 100 (occupied), -1 (unknown)
```

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

## WiFi Setup Wizard

The `--setup-wifi` option provides an interactive wizard to configure your robot's WiFi connection. This is recommended for first-time setup and when changing networks.

### Usage
```bash
./b4m_launch.sh --setup-wifi
```

### Setup Process

The wizard guides you through 6 steps:

**Step 1: Prerequisites Check**
- Verifies `config_robot.py` is available
- Checks Python 3 installation
- Validates user permissions for serial access

**Step 2: USB Connection**
- Guides you to connect the robot via USB
- Auto-detects available serial ports
- Allows manual port specification if needed

**Step 3: WiFi Network Configuration**
- Prompts for WiFi network name (SSID)
- Securely collects WiFi password (hidden input)
- Validates credential format

**Step 4: Agent Connection Method**
Choose between two connection methods:

1. **mDNS hostname (recommended)**: Uses `hostname.local` for automatic IP resolution
   - Example: `victus.local` → `192.168.68.105`
   - Works with dynamic IP addresses (DHCP)
   - Requires Avahi/mDNS support (included in most Linux distributions)

2. **Fixed IP address**: Manually specify the IP address
   - Example: `192.168.68.105`
   - Use when mDNS is not available or for static network configurations

**Step 5: Configuration Summary**
- Reviews all settings before applying
- Allows you to restart the wizard if needed

**Step 6: Robot Configuration**
- Executes the configuration on the robot
- Provides detailed error messages if issues occur
- Option to continue to full system launch after successful setup

### Example Session
```
🤖 B4M Robot WiFi Setup Wizard
==========================================

Step 1/6: Prerequisites Check
----------------------------
✅ config_robot.py found
✅ Python 3 available
✅ User has serial port access

Step 2/6: USB Connection
------------------------
📱 Please connect your robot via USB cable and power it on.
Press Enter when robot is connected and powered on...

🔍 Scanning for serial devices...
Found serial devices:
  1) /dev/ttyUSB0
  2) /dev/ttyACM0
  c) Enter custom port

Select port [1-2/c]: 1
✅ Selected port: /dev/ttyUSB0

Step 3/6: WiFi Network Configuration
------------------------------------
📶 Enter WiFi network name (SSID): MyHomeNetwork
🔐 Enter WiFi password: [hidden]
✅ WiFi credentials configured

Step 4/6: Agent Connection Method
---------------------------------
Choose how the robot will connect to the Micro-ROS agent:

1. mDNS hostname (recommended): victus.local
   → Resolves to: 192.168.68.105
2. Fixed IP address

Select connection method [1-2]: 1
✅ Will use mDNS: victus.local → 192.168.68.105

Step 5/6: Configuration Summary
-------------------------------
📋 Review your configuration:

   Serial Port: /dev/ttyUSB0
   WiFi SSID: MyHomeNetwork
   WiFi Password: [hidden]
   Agent Connection: mDNS hostname (victus.local)
   Agent Port: 8090

Proceed with this configuration? [y/N]: y

Step 6/6: Configuring Robot
---------------------------
🔄 Sending configuration to robot...

✅ Robot configuration completed successfully!

🎉 WiFi Setup Complete!

Continue to launch the robot system? [y/N]: y
```

### Troubleshooting WiFi Setup

**No serial devices found**
- Check USB cable connection
- Ensure robot is powered on
- Try a different USB port
- Verify device appears: `ls /dev/tty*`

**Permission denied accessing serial port**
- Add user to dialout group: `sudo usermod -a -G dialout $USER`
- Log out and back in, or run: `newgrp dialout`

**mDNS hostname not resolving**
- Install Avahi: `sudo apt-get install avahi-daemon avahi-utils`
- Test resolution: `avahi-resolve -n hostname.local`
- Ensure both devices are on the same network subnet

**Robot configuration failed**
- Check robot firmware compatibility
- Try power cycling the robot
- Verify WiFi credentials are correct
- Check network connectivity

### Environment Variables

The WiFi wizard sets these environment variables for `config_robot.py`:
- `ROBOT_SERIAL_PORT`: Selected serial port
- `ROBOT_WIFI_SSID`: WiFi network name
- `ROBOT_WIFI_PASSWORD`: WiFi password
- `ROBOT_AGENT_HOSTNAME`: mDNS hostname (if selected)
- `ROBOT_AGENT_IP`: Fixed IP address (if selected)
- `ROBOT_AGENT_PORT`: Agent port (default: 8090)

## Troubleshooting

### Common Issues

**RViz fails to start**
- Check DISPLAY environment variable
- Ensure X11 forwarding if using SSH
- In regression mode, virtual display is automatically configured

**Navigation system doesn't activate**
- Verify all sensor data is available (`ros2 topic list`)
- Check transform tree (`ros2 run tf2_tools view_frames`)
- Monitor logs for lifecycle manager errors
- Verify SLAM is generating maps (`ros2 topic echo /map --once`)

**SLAM not generating maps**
- Check laser scan data: `ros2 topic echo /scan` (should show 360 points)
- Verify transform chain: map->odom->base_footprint
- Ensure robot is moving to generate map data
- Check SLAM services: `ros2 service list | grep slam_toolbox`

**Robot doesn't respond to commands**
- Verify Micro-ROS agent connection
- Check ESP32 firmware and network configuration
- Ensure robot is powered and connected

**Gazebo Classic simulation issues**
- Check for `gzserver` and `gzclient` processes: `pgrep -f gazebo`
- Verify world file loading: Check for world file path errors in logs
- Robot spawning failures: Ensure `spawn_entity.py` completes successfully
- Physics simulation issues: Check for physics engine warnings in Gazebo output
- Sensor data publication: Verify laser scanner topics are active (`/scan`)
- Controller integration: Check differential drive plugin status in Gazebo logs

### Debug Commands
```bash
# Check active ROS2 nodes
ros2 node list

# Monitor topics
ros2 topic list
ros2 topic echo /scan  # Should show 360 laser points

# Check SLAM map generation
ros2 topic echo /map --once  # Shows occupancy grid info

# Check transforms
ros2 run tf2_tools view_frames

# Verify SLAM services
ros2 service list | grep slam_toolbox

# Monitor logs
tail -f logs/b4m_launch_*.log

# Test SLAM functionality
python3 tests/integration/test_slam_working.py

# Gazebo Classic simulation debugging
pgrep -f gazebo  # Check for Gazebo Classic processes
ps aux | grep gazebo  # Detailed process information
gazebo --verbose worlds/navigation_test_classic.world  # Launch world directly
rostopic list  # List active topics (if ROS1 bridge active)

# Launch file debugging
ros2 launch yahboomcar_nav gazebo_classic_nav_launch.py --show-args
ros2 launch yahboomcar_nav slam_test_gazebo_classic.py --show-args
```

## Cleanup

### Automatic Cleanup
The script includes automatic cleanup for:
- Duplicate process detection
- Failed step recovery
- Regression test failures

### Manual Cleanup
Always use the shutdown script when finished:
```bash
./b4m_shutdown.sh --keep-agent
```

**Important**: Always use `--keep-agent` flag to preserve hardware connection.

## MQTT Waypoint Management

When `--b4m-HA` is enabled, waypoint management is handled via MQTT commands:

- **Command format**: JSON messages sent to `yahboom/navigation/command` topic
- **Status updates**: Real-time feedback via `yahboom/navigation/status` topic  
- **Coordinate-based**: Direct position/orientation specification
- **Persistence**: Waypoints saved to JSON for future sessions
- **Home Assistant integration**: Native HA entity control

## Integration with Home Assistant

When enabled with the `--b4m-HA` flag, the system integrates with Home Assistant via MQTT:

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

### SLAM Integration
The system uses SLAM toolbox for dynamic mapping and localization:
- No pre-built map required
- Real-time map generation during navigation
- Automatic loop closure detection
- Compatible with existing MQTT waypoint system

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
- Use `--simulation` flag for Gazebo Classic simulation testing
- Review the migration guide: `FUTURE_GAZEBO_CLASSIC_MIGRATION.md`
- Refer to individual component documentation in the workspace
- Monitor ROS2 system status with standard ROS2 tools

## 🗺️ Autonomous Exploration Mode

### Quick Reference: Real Robot vs Simulation

| Use Case | Command | What Happens |
|----------|---------|--------------|
| **Explore with REAL robot** | `./b4m_launch.sh --explore` | Physical robot explores real environment |
| **Explore in simulation** | `./b4m_launch.sh --simulation --explore` | Simulated robot explores in Gazebo Classic |

### Enhanced Simulation Environment

When using `--simulation --explore`, the robot operates in a rich obstacle environment featuring:

- **Boundary Walls**: 7x7 meter enclosed area with proper boundaries
- **Central Table**: Large 0.8x1.2m obstacle for complex navigation testing  
- **Cylindrical Pillars**: Multiple columns of varying sizes for avoidance challenges
- **Box Obstacles**: Scattered rectangular obstacles at different orientations
- **L-shaped Obstacles**: Complex geometric challenges requiring sophisticated navigation
- **Narrow Passages**: Test precise navigation through tight spaces
- **Reference Markers**: Visual corner markers for orientation and debugging

### Exploration Performance Features

- **Safe Movement**: Very slow exploration speed (0.08 m/s) for safety
- **1-Foot Stop Distance**: Robot stops immediately when obstacles are ≤1 foot (30.48cm) away
- **Random Turn Direction**: Turns randomly left or right when obstacles are detected
- **Turn Until Clear**: Continues turning in place until path is clear (>40cm)
- **360° Obstacle Detection**: Full laser scan processing with real-time obstacle avoidance
- **Smart Navigation**: Forward → stop → turn in place → forward state machine
- **SLAM Integration**: Real-time map building while exploring obstacles
- **Continuous Operation**: Runs until manually stopped with Ctrl+C
- **Status Logging**: Regular exploration updates every 10 seconds

### Usage Examples

#### Quick Start - Real Robot
```bash
# Start exploration on physical robot
./b4m_launch.sh --explore
# Robot will explore real environment, stopping at 1-foot distance from obstacles
```

#### Quick Start - Enhanced Simulation  
```bash
# Start exploration in Gazebo Classic with rich obstacle environment
./b4m_launch.sh --simulation --explore
# Virtual robot explores complex environment, demonstrating 1-foot stop behavior with obstacles
```

#### With Debug Logging
```bash
# Real robot with verbose logging
./b4m_launch.sh --explore --debug

# Enhanced simulation with verbose logging  
./b4m_launch.sh --simulation --explore --debug
```

### ⚠️ Safety Notes

**Real Robot Operation** (`--explore`):
- ALWAYS supervise physical robot operation
- Ensure clear space around robot before starting
- Robot moves slowly but continuously
- Emergency stop: Ctrl+C in terminal

**Enhanced Simulation** (`--simulation --explore`):
- Safe for testing and development
- No physical robot movement
- Visual monitoring via RViz
- Rich obstacle interactions demonstrating 1-foot stop behavior
- Perfect for observing obstacle avoidance algorithms in action

### Available Test Scripts
- `test_square_corners.py` - Main navigation test (uses Gazebo Classic)
- `test_slam_working.py` - SLAM functionality validation
- `test_simple.py` - Basic system functionality test
- `scripts/autonomous_exploration.py` - Autonomous exploration with obstacle avoidance
- Various specialized tests: `test_motor_control.py`, `test_turning.py`, etc.