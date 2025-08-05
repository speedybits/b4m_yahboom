# SLAM Toolbox Regression Test Suite

This directory contains comprehensive regression tests for the SLAM toolbox integration in the B4M Yahboom robotics project. The tests validate all aspects of the SLAM system in Gazebo simulation, ensuring that any changes to the codebase don't break core SLAM functionality.

## Overview

The regression test suite covers all critical components identified in the `B4M_slam_toolbox_checklist.md`:

- **SLAM Launch & Initialization**: Validates SLAM toolbox startup and service availability
- **Robot Control**: Tests differential drive response to cmd_vel commands  
- **RViz Visualization**: Validates all visualization components work correctly
- **Transform Chain**: Tests TF tree integrity (map->odom->base_footprint)
- **Laser Scan**: Validates sensor data and obstacle detection capability
- **Map Building**: Tests real-time SLAM mapping during robot movement
- **Integrated Launch**: Validates end-to-end `b4m_HA_launch.sh` functionality

## Test Architecture

Each test is designed to:
- **Fail Fast**: Test suite aborts immediately on first failure (default behavior)
- **Be Granular**: Each test focuses on specific functionality to pinpoint issues
- **Run Quickly**: Most tests complete in under 2 minutes
- **Clean Up**: Automatic process cleanup using `b4m_shutdown.sh` between tests
- **Be Isolated**: Tests don't depend on each other and can run independently

## Quick Start

### Prerequisites

1. **ROS2 Humble** with workspace built:
   ```bash
   source install/setup.bash
   ```

2. **Required Packages**:
   ```bash
   sudo apt install ros-humble-slam-toolbox ros-humble-nav2-bringup \
                    ros-humble-teleop-twist-keyboard ignition-gazebo6 \
                    ros-humble-ros-gz-sim ros-humble-ros-gz-bridge
   ```

3. **Python Dependencies**:
   ```bash
   pip3 install -r regression/requirements.txt
   ```

### Running Tests

**Run all tests (fail-fast mode - aborts on first failure):**
```bash
./regression/run_regression_tests.sh
```

**Run all tests (continue despite failures):**
```bash
./regression/run_regression_tests.sh --no-fail-fast
```

**Quick tests only (core functionality, ~5 minutes):**
```bash
./regression/run_regression_tests.sh --quick
```

**Integration tests only (full system, ~15 minutes):**
```bash
./regression/run_regression_tests.sh --integration-only
```

**Run specific test:**
```bash
./regression/run_regression_tests.sh --test slam_launch
```

**List available tests:**
```bash
./regression/run_regression_tests.sh --list
```

## Individual Test Descriptions

### 1. SLAM Launch Test (`test_slam_launch.py`)
**Purpose**: Validates SLAM toolbox launches correctly in Gazebo simulation

**What it tests**:
- Ignition Gazebo starts successfully
- Robot spawns and publishes required topics
- SLAM toolbox initializes and services become available
- Map topic begins publishing occupancy grid data

**Validates checklist items**:
- ✅ SLAM Integration: SLAM toolbox running and ready for mapping
- ✅ Core Validation Requirements: SLAM Ready

**Runtime**: ~2 minutes

### 2. Robot Control Test (`test_robot_control.py`)
**Purpose**: Validates robot responds to movement commands

**What it tests**:
- Robot odometry data is available
- cmd_vel topic accepts and processes commands
- Robot moves forward in response to linear velocity commands
- Robot rotates in response to angular velocity commands
- Precise movement control for navigation patterns
- Robot stops when commanded

**Validates checklist items**:
- ✅ Robot Control: Differential drive plugin working with cmd_vel commands
- ✅ Robot Turning Fixed: Robot now successfully turns and navigates

**Runtime**: ~3 minutes

### 3. RViz Visualization Test (`test_rviz_visualization.py`)
**Purpose**: Validates RViz displays all required SLAM components

**What it tests**:
- RViz launches with test configuration
- Robot model description is available for display
- TF data is available for transform visualization
- Laser scan topic can be displayed in RViz
- Map topic data is available for map display
- All visualization topics are publishing

**Validates checklist items**:
- ✅ RViz Integration: Visualization system launched and operational
- ✅ RViz Robot Display: Robot model visible in RViz 3D view
- ✅ RViz Laser Scan Display: Laser scan points visible when data available
- ✅ RViz Map Display: Map topic configured for real-time SLAM mapping
- ✅ RViz Transform Display: TF frames displayed correctly

**Runtime**: ~4 minutes

### 4. Transform Chain Test (`test_transforms.py`)
**Purpose**: Validates TF transform chain integrity for SLAM

**What it tests**:
- TF topics (/tf, /tf_static) are publishing
- Key transforms are being received (map->odom, odom->base_footprint)
- Complete transform chain exists (map->base_link, map->laser)
- Transforms are stable over time with reasonable update rates
- Transform data is consistent and has reasonable values
- SLAM toolbox is authority for map->odom transform

**Validates checklist items**:
- ✅ Core Validation Requirements: Transform Chain
- ✅ Transform Tree: slam_toolbox provides map->odom transform
- ✅ Key Integration Points: Transform tree stability

**Runtime**: ~2 minutes

### 5. Laser Scan Test (`test_laser_scan.py`)
**Purpose**: Validates laser scan data and obstacle detection

**What it tests**:
- Laser scan topic is available and publishing correct message type
- Scan data is received with valid parameters
- Scan publishing rate is reasonable (1-50 Hz)
- Scan data quality metrics are acceptable
- Obstacle detection algorithms work with simulated data
- Real obstacle detection in simulation environment
- Laser scan frame is consistent with TF tree

**Validates checklist items**:
- ✅ Sensor Integration: Laser scan (/scan) topics active
- ✅ RViz Laser Scan Display: Laser scan points visible when data available
- ✅ Automated obstacle detection validation: verify detection of obstacles

**Runtime**: ~3 minutes

### 6. Map Building Test (`test_map_building.py`)
**Purpose**: Validates real-time SLAM mapping functionality

**What it tests**:
- Map topic publishes valid occupancy grid data
- Initial map state before robot movement
- Map grows and updates during robot movement
- Map quality metrics and exploration progress
- Map data consistency during updates
- SLAM services are available for map operations
- Map saving functionality works
- Loop closure detection capability

**Validates checklist items**:
- ✅ Automated Map Validation: Verify SLAM mapping functionality
- ✅ Check map topic publishing: /map topic has valid occupancy grid data
- ✅ Test loop closure: verify SLAM detects return to starting position
- ✅ Automated Map Saving: Save maps programmatically

**Runtime**: ~5 minutes

### 7. Integrated Launch Test (`test_integrated_launch.py`)
**Purpose**: Validates complete b4m_HA_launch.sh integration

**What it tests**:
- Launch script exists and is executable
- Script shows proper help information with SLAM flags
- Full integrated launch works in simulation mode
- Launch process completes all steps successfully
- System validation after launch completion
- Each launch step is properly validated
- SLAM-specific integration validation
- System can be cleanly shut down

**Validates checklist items**:
- ✅ Integrated Launch Test: ./b4m_HA_launch.sh --simulation --autotest --debug
- ✅ Full System Validation: Complete automated testing through launch script
- ✅ b4m_HA_launch.sh Integration: Successfully updated to use Ignition Gazebo

**Runtime**: ~8 minutes

## Test Environment Configuration

### ROS Configuration
- **ROS Domain ID**: 20 (isolated from other ROS systems)
- **Workspace**: Must be built and sourced (`source install/setup.bash`)
- **Environment**: Tests expect to run from project root directory

### Process Management
- Each test performs complete process cleanup using `b4m_shutdown.sh --keep-agent`
- Tests are isolated and don't interfere with each other
- Automatic timeout handling (10 minutes per test maximum)
- Proper cleanup is guaranteed even on test failure or script interruption (Ctrl+C)

### Temporary Files
- Tests create temporary directories for map files and configurations
- All temporary files are cleaned up automatically
- Test logs are printed to console in real-time

## Usage Patterns

### Development Workflow
```bash
# After making changes to SLAM integration (fail-fast):
./regression/run_regression_tests.sh --quick

# Before committing major changes (fail-fast):
./regression/run_regression_tests.sh

# To debug multiple issues (continue on failures):
./regression/run_regression_tests.sh --no-fail-fast

# To test specific functionality:
./regression/run_regression_tests.sh --test robot_control
```

### CI/CD Integration
```bash
# In automated testing pipeline:
./regression/run_regression_tests.sh --quick
# Returns exit code 0 for success, 1 for failure
```

### Debugging Failed Tests
```bash
# Run individual test with full output:
cd /home/mike/projects/b4m_yahboom
python3 regression/test_slam_launch.py -v

# Check specific functionality:
./regression/run_regression_tests.sh --test transforms
```

## Test Output and Results

### Success Output
```
================================== 
SLAM Toolbox Regression Test Suite
==================================

✓ PASSED: slam_launch (45s)
✓ PASSED: robot_control (32s) 
✓ PASSED: transforms (28s)

🎉 ALL TESTS PASSED! Success rate: 100%
```

### Fail-Fast Failure Output
```
✓ PASSED: slam_launch (45s)
✗ FAILED: robot_control (45s)

❌ FAIL-FAST: Test suite aborted due to test failure
❌ Failed test: robot_control

Tests completed before failure:
✓ slam_launch (45s)
✗ robot_control (45s)

🛑 REGRESSION TEST SUITE FAILED
   First failing test: robot_control
   Use --no-fail-fast to run all tests despite failures
```

### Continue-on-Failure Output
```
✓ PASSED: slam_launch (45s)
✗ FAILED: robot_control (45s)
⏰ TIMEOUT: integrated_launch (600s)
✓ PASSED: transforms (28s)

❌ Some tests failed. Success rate: 50%
```

## Troubleshooting

### Common Issues

**"No ROS environment sourced"**
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

**"Python dependencies missing"**
```bash
pip3 install -r regression/requirements.txt
```

**"Process cleanup issues"**
```bash
# The test runner automatically uses b4m_shutdown.sh for cleanup
# If manual cleanup is needed:
./b4m_shutdown.sh --keep-agent

# Or force cleanup of specific processes:
pkill -f "ign gazebo"
pkill -f "slam_toolbox"
pkill -f "ros2"
```

**"Permission denied on test runner"**
```bash
chmod +x regression/run_regression_tests.sh
```

### Test-Specific Issues

**SLAM Launch Test Fails**
- Check that SLAM toolbox package is installed
- Verify Ignition Gazebo is properly installed
- Ensure no other Gazebo instances are running

**Robot Control Test Fails**
- Verify robot URDF has correct differential drive plugin
- Check that ROS-Gazebo bridge is working
- Ensure cmd_vel topic is properly bridged

**Transform Test Fails**
- Check that robot_state_publisher is running
- Verify SLAM toolbox is publishing map->odom transform
- Check TF tree with `ros2 run tf2_ros tf2_monitor`

**Visualization Test Fails**
- Ensure DISPLAY environment variable is set
- Check that RViz2 is installed
- Verify robot description parameter is available

## Integration with Project Workflow

### Before Committing Changes
Always run the regression test suite before committing changes to ensure SLAM functionality remains intact:

```bash
# Quick validation before commit:
./regression/run_regression_tests.sh --quick

# Full validation for major changes:
./regression/run_regression_tests.sh
```

### Adding New Tests
When adding new SLAM-related functionality:

1. Create new test file in `regression/` directory
2. Follow existing test patterns and naming conventions
3. Add test to `run_regression_tests.sh` test suite
4. Update this README with test description
5. Ensure test cleans up after itself

### Maintenance
- Tests should be updated when SLAM integration changes
- Keep test runtime reasonable (< 5 minutes per test)
- Ensure tests fail quickly when functionality is broken
- Update checklist item validation as requirements evolve

## Related Files

- `../B4M_slam_toolbox_checklist.md` - Requirements checklist
- `../b4m_HA_launch.sh` - Main launch script being tested
- `../b4m_shutdown.sh` - Cleanup script used by tests
- `../yahboomcar_nav/launch/slam_*.py` - SLAM launch files
- `../yahboomcar_nav/params/slam_*.yaml` - SLAM configuration files

## Success Criteria

The regression test suite is considered successful when:
- All quick tests pass consistently (< 5 minutes)
- Integration tests validate end-to-end functionality
- Tests catch regressions when SLAM components are broken
- New SLAM features can be validated with additional tests
- Test suite runs reliably in automated environments
- Proper cleanup is performed using `b4m_shutdown.sh` after each test

## Cleanup and Process Management

The test suite integrates with `b4m_shutdown.sh` to ensure proper cleanup:
- **Automatic Cleanup**: `b4m_shutdown.sh --keep-agent` is called after each test
- **Fail-Fast Cleanup**: Proper shutdown even when tests fail in fail-fast mode
- **Interrupt Handling**: Cleanup is guaranteed even on Ctrl+C or script termination
- **Fallback Cleanup**: Manual process termination if `b4m_shutdown.sh` is unavailable
- **Micro-ROS Agent**: Preserved with `--keep-agent` flag to maintain hardware connection

This comprehensive test suite ensures that the SLAM toolbox integration remains robust and functional as the project evolves.