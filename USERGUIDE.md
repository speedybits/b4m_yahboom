# B4M Robot System User Guide

## Overview

The B4M Robot is an autonomous navigation system built on ROS2 with Home Assistant integration. This guide covers how to use the `b4m_HA_launch.sh` script to launch and operate the robot system with SLAM (Simultaneous Localization and Mapping) capabilities using Gazebo Classic for simulation.

## Prerequisites

- ROS2 Humble installed and sourced
- Docker (for Micro-ROS agent)
- Physical robot or Gazebo Classic simulation environment
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
Runs the robot in Gazebo Classic simulation instead of using physical hardware.

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
| `--simulation` | Launch in Gazebo Classic simulation mode |
| `--slam-test` | Add automated SLAM testing steps (implies --autotest) |
| `--explore` | Enable autonomous exploration on **REAL ROBOT** (implies --autotest) |
| `-h, --help` | Show help information |

## Usage Scenarios

### 📋 Quick Reference: Real Robot vs Simulation
| Use Case | Command | What Happens |
|----------|---------|--------------|
| **Explore with REAL robot** | `./b4m_HA_launch.sh --explore` | Physical robot explores real environment |
| **Explore in simulation** | `./b4m_HA_launch.sh --simulation --explore` | Simulated robot explores in Gazebo Classic |
| **Regular real robot launch** | `./b4m_HA_launch.sh` | Interactive setup with physical robot |
| **Regular simulation launch** | `./b4m_HA_launch.sh --simulation` | Interactive setup in Gazebo Classic |

### 1. Interactive SLAM Mapping in Simulation
```bash
# Using integrated launch with Gazebo Classic
./b4m_HA_launch.sh --simulation

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
./b4m_HA_launch.sh --simulation --autotest --slam-test
```
Runs full automated test suite including:
- 1-meter square navigation pattern
- SLAM map generation verification (69x77 grid)
- MQTT waypoint navigation testing
- 15 SLAM services availability check

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

### 6. Autonomous Exploration Mode

#### Real Robot Exploration (Physical Robot)
```bash
./b4m_HA_launch.sh --explore
```
**Physical robot** autonomously explores the real environment with obstacle avoidance while building a SLAM map.

#### Gazebo Classic Simulation Exploration
```bash
./b4m_HA_launch.sh --simulation --explore
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
7. **MQTT Navigation**: Starts waypoint navigation with Home Assistant integration
8. **Robot Manager GUI**: Launches visual control interface (interactive mode only)

### Simulation Mode (`--simulation`)

Uses Gazebo Classic simulation environment:

1. **Gazebo Classic Launch**: Starts Gazebo Classic simulation with integrated robot spawning
2. **Robot Initialization**: Robot systems handled by integrated launch file
3. **Robot Systems**: Robot state publisher and transforms active
4. **RViz Visualization**: Starts visualization with laser scan display
5. **SLAM Navigation**: Launches SLAM toolbox for mapping and localization
6. **SLAM Initialization**: Verifies SLAM map generation (69x77 grid)
7. **MQTT Navigation**: Starts waypoint navigation system
8. **Robot Manager GUI**: Visual control interface (if not in autotest mode)

### Exploration Mode (`--explore`)

Available in both simulation and real robot modes:

1. **System Launch**: Same as above (steps 1-7)
2. **Autonomous Explorer**: Launches obstacle-avoiding exploration script instead of GUI
   - Robot moves slowly (0.08 m/s) for safety
   - Uses laser scanner for obstacle detection (40cm safe distance)
   - Changes direction randomly every 8-15 seconds for thorough exploration
   - Turns away when obstacles detected within 60cm
   - Integrates with SLAM for real-time mapping
   - Runs continuously until Ctrl+C

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
python3 test_square_corners.py
```

### Automated Navigation Tests
The Gazebo Classic simulation supports the full automated test suite with navigation validation.

## SLAM Map Creation and Saving

### Creating Maps in Simulation (Gazebo Classic)

#### Step 1: Launch SLAM System
```bash
# Launch full system with SLAM in Gazebo Classic
./b4m_HA_launch.sh --simulation

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
./b4m_HA_launch.sh

# Ensure robot is powered on and Micro-ROS agent is connected
```

#### Step 2: Drive Robot to Map Environment
```bash
# Use keyboard teleop to drive robot
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Or use the Robot Manager GUI for manual control
# GUI provides visual waypoint placement and robot control
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
python3 test_slam_working.py

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
| **Explore with REAL robot** | `./b4m_HA_launch.sh --explore` | Physical robot explores real environment |
| **Explore in simulation** | `./b4m_HA_launch.sh --simulation --explore` | Simulated robot explores in Gazebo Classic |

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
./b4m_HA_launch.sh --explore
# Robot will explore real environment, stopping at 1-foot distance from obstacles
```

#### Quick Start - Enhanced Simulation  
```bash
# Start exploration in Gazebo Classic with rich obstacle environment
./b4m_HA_launch.sh --simulation --explore
# Virtual robot explores complex environment, demonstrating 1-foot stop behavior with obstacles
```

#### With Debug Logging
```bash
# Real robot with verbose logging
./b4m_HA_launch.sh --explore --debug

# Enhanced simulation with verbose logging  
./b4m_HA_launch.sh --simulation --explore --debug
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