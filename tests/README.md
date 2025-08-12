# B4M Robot Test Suite Organization

This directory contains the organized test suite for the B4M Yahboom robot system. All tests have been categorized by functionality to improve maintainability and discoverability.

## Directory Structure

### `integration/` - System-wide Integration Tests
- **`test_basic_movement.py`** - Primary regression test (360° rotation validation)
- **`test_simple.py`** - Basic system functionality tests
- **`test_slam_working.py`** - SLAM system integration validation

### `navigation/` - Navigation and Movement Tests  
- **`test_square_corners_configurable.py`** - Fallback regression test (1m square navigation)
- **`test_square_corners_classic.py`** - Gazebo Classic square navigation
- **`test_square_corners.py`** - General square navigation test
- **`test_square_navigation.py`** - Navigation system validation

### `simulation/` - Gazebo Simulation Tests
- **`test_gazebo_classic_sensor.py`** - Sensor integration in Gazebo Classic
- **`test_gazebo_position.py`** - Position and odometry validation
- **`test_gazebo_slam_integration.py`** - SLAM integration without launching Gazebo
- **`test_physics.py`** - Physics simulation validation
- **`test_spawn_position.py`** - Robot spawning validation
- **`test_model_geometry.py`** - Robot model geometry tests

### `hardware/` - Physical Robot Hardware Tests
- **`test_motor_control.py`** - Motor control validation
- **`test_controller_manager.py`** - ROS2 controller manager tests
- **`test_real_turning.py`** - Real robot turning capabilities

### `sensors/` - Sensor-specific Tests
- **`test_laser_simple.py`** - Basic laser scanner tests
- **`test_lidar_data.py`** - LIDAR data validation
- **`test_minimal_sensor.py`** - Minimal sensor functionality

### `experimental/` - Development and Debugging Tests
- **`test_90_degree_turn.py`** - Specific turning angle tests
- **`test_simple_turn.py`** - Basic turning validation
- **`test_minimal_turn.py`** - Minimal turning functionality
- **`test_turning.py`** - General turning tests
- **`test_visual_observation.py`** - Visual feedback tests
- **`test_visual_turning.py`** - Visual turning validation
- **`test_square_debug.py`** - Square navigation debugging

### `archived/` - Historical Tests
- **`test_ros_bridge_data.py`** - Legacy ROS bridge tests
- **`test_wheel_contact.sh`** - Legacy wheel contact validation

## Running Tests

### Using the Test Runner
The provided test runner (`run_tests.sh`) offers organized test execution:

```bash
# Run all tests in a category
./tests/run_tests.sh integration
./tests/run_tests.sh navigation
./tests/run_tests.sh simulation

# Run all tests
./tests/run_tests.sh all

# List available tests
./tests/run_tests.sh list

# Run a specific test
./tests/run_tests.sh test tests/integration/test_basic_movement.py

# Run regression tests
./tests/run_tests.sh regression
```

### Direct Test Execution
Tests can also be run directly:

```bash
# Simulation mode (default)
python3 tests/integration/test_basic_movement.py --simulation

# Real robot mode  
python3 tests/integration/test_basic_movement.py --real-robot
```

## Integration with B4M System

### Script Dependencies
The main launch script `b4m_HA_launch.sh` references:
- **Primary regression test**: `tests/integration/test_basic_movement.py`
- **Fallback regression test**: `tests/navigation/test_square_corners_configurable.py`

### Environment Variables
Tests support these environment variables:
- `TEST_MODE=SIMULATION` or `TEST_MODE=REAL_ROBOT` - Override test mode
- `SYSTEM_ALREADY_RUNNING=true` - Skip system launch (for regression mode)

### Regression Testing
The regression test suite validates:
1. **Basic Movement**: 360° rotation capability (primary test)
2. **Navigation**: 1-meter square navigation (fallback test)
3. **Laser Scan Stability**: Coordinate frame validation (additional test)

## Adding New Tests

### Guidelines
1. **Place tests in appropriate categories** based on functionality
2. **Follow naming convention**: `test_<descriptive_name>.py`
3. **Support both simulation and real robot modes** where applicable
4. **Include proper cleanup** in test teardown
5. **Use meaningful test descriptions** and error messages

### Test Structure
```python
#!/usr/bin/env python3
# test_example.py - Brief description of what this test validates

import sys
import os

def test_functionality(use_simulation=True):
    """Test description
    
    Args:
        use_simulation: If True, uses Gazebo Classic. If False, uses real robot.
    """
    mode_str = "Gazebo Classic" if use_simulation else "Real Robot"
    print(f"=== Example Test ({mode_str}) ===")
    
    # Test implementation
    # Return 0 for success, 1 for failure
    return 0

if __name__ == "__main__":
    # Support command line arguments
    use_sim = "--real-robot" not in sys.argv
    
    # Support environment variable override
    if os.environ.get('TEST_MODE') == 'REAL_ROBOT':
        use_sim = False
    
    sys.exit(test_functionality(use_sim))
```

## Maintenance

### Regular Tasks
- **Update test paths** when moving test files
- **Verify script references** after reorganization
- **Clean up experimental tests** that are no longer needed
- **Archive obsolete tests** rather than deleting them

### Test Categories Usage
- **Integration**: Tests that validate end-to-end system behavior
- **Navigation**: Tests focused on movement and path planning
- **Simulation**: Tests that require or validate Gazebo functionality
- **Hardware**: Tests that require physical robot hardware
- **Sensors**: Tests focused on sensor data and processing
- **Experimental**: Temporary tests for debugging and development
- **Archived**: Historical tests kept for reference

This organization ensures easy test discovery, maintenance, and scalability as the test suite grows.