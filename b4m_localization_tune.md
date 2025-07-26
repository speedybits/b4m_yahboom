# B4M Localization Tuning Test Plan

## Overview

This document outlines a systematic approach to test and improve the reliability of navigation and localization in the B4M robot system. The test plan focuses on assessing localization quality, navigation performance, and provides automated tools for iterative tuning of localization parameters.

## Test Objectives

1. **Localization Quality Assessment**: Evaluate AMCL and EKF filter performance
2. **Navigation Reliability Testing**: Test waypoint navigation accuracy and consistency
3. **Parameter Optimization**: Systematic tuning of localization parameters
4. **Automated Testing Integration**: Compatible with existing `b4m_HA_testplan.md` framework
5. **Regression Prevention**: Ensure parameter changes don't degrade performance

## Integration with B4M Launch Script

### Execution Methods

```bash
# Standalone localization test (uses existing launch sequence + localization tests)
./b4m_HA_launch.sh --skip-agent --localization-test

# Automated localization test with parameter tuning
./b4m_HA_launch.sh --skip-agent --localization-test --tune-params

# Full system test including localization validation
./b4m_HA_launch.sh --autotest --localization-test

# Debug mode with verbose localization logging
./b4m_HA_launch.sh --skip-agent --localization-test --debug
```

### Test Architecture Integration

The localization test leverages the existing `b4m_HA_launch.sh` framework by adding:
- **New argument**: `--localization-test` to enable localization testing mode
- **New argument**: `--tune-params` to enable parameter tuning iterations
- **Step 8**: Localization Quality Assessment (after Step 7 in launch sequence)
- **Step 9**: Navigation Performance Testing
- **Step 10**: Parameter Validation and Results Logging

## Test Components

### 1. Localization Quality Metrics

#### AMCL Performance Assessment
```bash
validate_amcl_localization() {
    # Monitor AMCL particle spread and convergence
    # Test criteria:
    # - Particle cloud convergence time < 30 seconds
    # - Covariance matrix eigenvalues within acceptable bounds
    # - Pose estimate stability (low variance over time)
    # - Transform consistency (map→odom→base_link)
}
```

#### EKF Filter Validation
```bash
validate_ekf_performance() {
    # Assess Extended Kalman Filter behavior
    # Test criteria:
    # - Odometry drift over known distances
    # - IMU integration accuracy
    # - Sensor fusion consistency
    # - Covariance prediction accuracy
}
```

### 2. Navigation Performance Testing

#### Waypoint Navigation Accuracy
```bash
test_waypoint_accuracy() {
    # Execute predefined waypoint sequence
    # Test criteria:
    # - Goal reach tolerance < 0.2m position, < 0.1 rad orientation
    # - Path planning success rate > 95%
    # - Obstacle avoidance behavior
    # - Recovery behavior effectiveness
}
```

#### Localization Drift Assessment
```bash
test_localization_drift() {
    # Navigate robot in closed loop to measure drift
    # Test criteria:
    # - Return-to-start position error < 0.3m
    # - Cumulative heading error < 0.2 rad
    # - Consistency across multiple loop iterations
}
```

### 3. Parameter Tuning Framework

#### Parameter Storage Locations (Research Results)

**Primary Parameter Files**:
- **AMCL Parameters**: `/home/yahboom/b4m_yahboom/yahboomcar_nav/params/dwb_nav_params.yaml`
- **EKF Parameters**: `/home/yahboom/b4m_yahboom/yahboomcar_bringup/param/ekf.yaml`
- **Navigation Parameters**: `/home/yahboom/b4m_yahboom/yahboomcar_nav/params/dwb_nav_params.yaml`
- **GUI Access**: B4M Robot Manager loads parameters from `dwb_nav_params.yaml`

#### Current Parameter Baseline (From Original Demo)
Baseline parameters will be extracted from the earliest Git commit to establish demo defaults:

**AMCL Parameters (dwb_nav_params.yaml)**:
- `min_particles: 500`, `max_particles: 3000`
- Motion model: `alpha1: 0.1`, `alpha2: 0.1`, `alpha3: 0.1`, `alpha4: 0.1`, `alpha5: 0.1`
- Particle filter: `pf_err: 0.01`, `pf_z: 0.99`
- Update thresholds: `update_min_d: 0.05`, `update_min_a: 0.05`
- Recovery: `recovery_alpha_fast: 0.1`, `recovery_alpha_slow: 0.05`

**EKF Parameters (ekf.yaml)**:
- `frequency: 30.0` Hz (main) / `10.0` Hz (B4M system)
- `two_d_mode: true` (planar robot operation)
- Sensor configuration: `odom0_differential: false`, `imu0_differential: false`
- Process noise covariance: 15x15 matrix with diagonal values 0.01-0.06
- IMU settings: `imu0_remove_gravitational_acceleration: true`

**Navigation Parameters (dwb_nav_params.yaml)**:
- `controller_frequency: 20.0` Hz
- Velocity limits: `max_vel_x: 0.30`, `max_vel_theta: 1.0`
- Goal tolerance: `xy_goal_tolerance: 0.3`, `yaw_goal_tolerance: 0.3`
- Costmap inflation: `inflation_radius: 0.55`, `cost_scaling_factor: 3.0`

#### Automated Parameter Testing
```bash
tune_localization_parameters() {
    local param_set=$1
    local test_iteration=$2
    
    # Try runtime parameter update first (no rebuild required)
    if apply_runtime_parameters "$param_set"; then
        debug_log "Parameters applied at runtime - no rebuild needed"
    else
        # Apply parameter set to files
        update_parameter_files "$param_set"
        
        # Minimal rebuild based on changed parameters
        if [[ "$param_set" == *"ekf"* ]]; then
            colcon build --packages-select yahboomcar_bringup
        fi
        if [[ "$param_set" == *"amcl"* ]] || [[ "$param_set" == *"nav"* ]]; then
            colcon build --packages-select yahboomcar_nav
        fi
        
        # Restart affected navigation components (Steps 5-7)
        restart_navigation_stack
    fi
    
    # Execute 5-minute test suite
    run_localization_tests "$test_iteration" 300  # 5 minutes timeout
    
    # Log results
    log_tuning_results "$param_set" "$test_iteration"
}
```

## Test Execution Workflow

### Phase 1: System Startup and Baseline Assessment (2 minutes)
1. **System Startup**: Use existing launch sequence through Step 7
2. **Unknown Initial Pose Handling**: Enable global localization without manual pose setting
3. **Baseline Metrics Collection**: Record current performance metrics
4. **Parameter Snapshot**: Save current configuration

### Phase 2: Localization Quality Testing (2 minutes)
1. **Global Localization Test**: Test AMCL convergence from unknown pose
2. **Particle Convergence Assessment**: Monitor particle cloud stability
3. **Transform Consistency Check**: Validate map→odom→base_link stability
4. **Sensor Fusion Validation**: Compare odometry vs. AMCL estimates

### Phase 3: Navigation Performance Testing (1 minute)
1. **Waypoint Sequence**: Execute 3-point navigation test using yahboom_map.yaml
2. **Goal Tolerance Testing**: Measure position/orientation accuracy
3. **Path Planning Validation**: Verify successful path generation
4. **Timeout Handling**: 2x normal runtime (10 minutes max) for unresponsive tests

### Phase 4: Results Analysis and Logging (30 seconds)
1. **Metric Calculation**: Compute performance scores
2. **Comparison Analysis**: Compare against baseline/previous runs
3. **Parameter Impact Assessment**: Correlate parameters with performance
4. **Report Generation**: Create detailed test report

**Total Test Duration**: 5 minutes per parameter set (as requested)
**Timeout Policy**: 10 minutes maximum if test becomes unresponsive (2x normal)

## Test Implementation - b4m_HA_launch.sh Integration

### New Command Line Arguments

Add these arguments to the existing `b4m_HA_launch.sh` argument parsing:

```bash
# Add to existing argument parsing in b4m_HA_launch.sh (around line 13)
LOCALIZATION_TEST=false
TUNE_PARAMS=false

for arg in "$@"; do
    case $arg in
        --localization-test)
            LOCALIZATION_TEST=true
            shift
            ;;
        --tune-params)
            TUNE_PARAMS=true
            shift
            ;;
        # ... existing arguments ...
    esac
done
```

### Localization Test Configuration

```bash
# Add after existing configuration variables in b4m_HA_launch.sh
if [ "$LOCALIZATION_TEST" = true ]; then
    LOCALIZATION_TEST_DIR="$WORKSPACE_ROOT/localization_tests"
    PARAM_BACKUP_DIR="$LOCALIZATION_TEST_DIR/param_backups"
    TEST_RESULTS_DIR="$LOCALIZATION_TEST_DIR/results"
    LOCALIZATION_LOG="$TEST_RESULTS_DIR/localization_test_$(date +%Y%m%d_%H%M%S).log"
    
    # Test parameters
    WAYPOINT_SEQUENCE_FILE="$LOCALIZATION_TEST_DIR/test_waypoints.json"
    BASELINE_PARAMS_FILE="$LOCALIZATION_TEST_DIR/baseline_params.yaml"
    TUNING_PARAMS_DIR="$LOCALIZATION_TEST_DIR/param_sets"
    
    # Create directories
    mkdir -p "$LOCALIZATION_TEST_DIR" "$PARAM_BACKUP_DIR" "$TEST_RESULTS_DIR"
fi
```

### Test Waypoint Sequence

Create standardized test pattern in `test_waypoints.json`:
```json
{
  "map_name": "yahboom_map",
  "test_sequence": [
    {
      "name": "start_position",
      "position": {"x": 0.0, "y": 0.0},
      "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
      "tolerance": {"position": 0.2, "orientation": 0.1},
      "timeout": 120
    },
    {
      "name": "test_point_1",
      "position": {"x": 1.0, "y": 0.0},
      "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
      "tolerance": {"position": 0.2, "orientation": 0.1},
      "timeout": 120
    },
    {
      "name": "test_point_2",
      "position": {"x": 0.5, "y": 0.5},
      "orientation": {"x": 0.0, "y": 0.0, "z": 0.707, "w": 0.707},
      "tolerance": {"position": 0.2, "orientation": 0.1},
      "timeout": 120
    },
    {
      "name": "return_start",
      "position": {"x": 0.0, "y": 0.0},
      "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
      "tolerance": {"position": 0.3, "orientation": 0.2},
      "timeout": 240
    }
  ]
}
```

## Integration with Existing Launch Script

### Extension Point in b4m_HA_launch.sh

Add localization testing after Step 7 (around line 636):

```bash
# Add after Step 7 in b4m_HA_launch.sh
if [ "$LOCALIZATION_TEST" = true ]; then
    # Step 8: Localization Quality Assessment
    echo "======================================================"
    echo "STEP 8: Localization Quality Assessment"
    echo "======================================================"
    
    if [ "$AUTOTEST_MODE" = true ]; then
        log_message "AUTOTEST STEP 8: Localization Quality Assessment"
        if validate_localization_quality; then
            echo "✅ Step 8 validation passed"
            log_message "AUTOTEST STEP 8: PASSED"
        else
            handle_test_failure "8" "Localization quality assessment failed"
        fi
    else
        run_localization_quality_tests
    fi
    
    # Step 9: Navigation Performance Testing
    echo "======================================================"
    echo "STEP 9: Navigation Performance Testing"
    echo "======================================================"
    
    if [ "$AUTOTEST_MODE" = true ]; then
        log_message "AUTOTEST STEP 9: Navigation Performance Testing"
        if validate_navigation_performance; then
            echo "✅ Step 9 validation passed"
            log_message "AUTOTEST STEP 9: PASSED"
        else
            handle_test_failure "9" "Navigation performance test failed"
        fi
    else
        run_navigation_performance_tests
    fi
fi
```

### Validation Functions

Add these functions to `b4m_HA_launch.sh`:

```bash
# Add these functions after existing validate_step_success() function

validate_localization_quality() {
    local start_time=$(date +%s)
    debug_log "Starting localization quality assessment"
    
    # Test global localization (no manual pose setting required)
    if ! test_global_localization; then
        echo "ERROR: Global localization from unknown pose failed"
        return 1
    fi
    
    # Test AMCL convergence
    if ! test_amcl_convergence; then
        echo "ERROR: AMCL particle convergence failed"
        return 1
    fi
    
    # Test EKF consistency
    if ! test_ekf_consistency; then
        echo "ERROR: EKF filter consistency check failed"
        return 1
    fi
    
    # Test transform stability
    if ! test_transform_stability; then
        echo "ERROR: Transform tree stability test failed"
        return 1
    fi
    
    local duration=$(($(date +%s) - start_time))
    debug_log "Localization quality tests passed in ${duration}s"
    return 0
}

validate_navigation_performance() {
    local start_time=$(date +%s)
    debug_log "Starting navigation performance testing"
    
    # Execute waypoint sequence (works without MQTT)
    if ! execute_test_waypoint_sequence_no_mqtt; then
        echo "ERROR: Waypoint navigation sequence failed"
        return 1
    fi
    
    # Test navigation accuracy using yahboom_map.yaml
    if ! test_navigation_accuracy_yahboom_map; then
        echo "ERROR: Navigation accuracy test failed on yahboom_map"
        return 1
    fi
    
    local duration=$(($(date +%s) - start_time))
    debug_log "Navigation performance tests passed in ${duration}s"
    return 0
}
```

## Logging and Results Analysis

### Test Results Structure

```
localization_tests/
├── results/
│   ├── localization_test_20250725_140000.log
│   ├── navigation_metrics_20250725_140000.json
│   └── parameter_comparison_20250725_140000.csv
├── param_backups/
│   ├── baseline_params_20250725.yaml
│   └── modified_params_20250725_140000.yaml
├── param_sets/
│   ├── aggressive_tuning.yaml
│   ├── conservative_tuning.yaml
│   └── experimental_set_01.yaml
└── test_waypoints.json
```

### Performance Metrics Logging

```json
{
  "test_timestamp": "2025-07-25T14:00:00Z",
  "test_duration": 1800,
  "amcl_metrics": {
    "convergence_time": 25.3,
    "particle_spread": 0.15,
    "pose_stability_variance": 0.008,
    "transform_consistency_score": 0.95
  },
  "ekf_metrics": {
    "odometry_drift_per_meter": 0.02,
    "imu_integration_error": 0.005,
    "covariance_prediction_accuracy": 0.88
  },
  "navigation_metrics": {
    "waypoint_accuracy_average": 0.12,
    "path_planning_success_rate": 0.98,
    "closed_loop_position_error": 0.18,
    "closed_loop_heading_error": 0.08
  },
  "parameter_snapshot": {
    "min_particles": 500,
    "max_particles": 2000,
    "kld_err": 0.05,
    "odom_alpha1": 0.2
  }
}
```

## Parameter Tuning Methodology

### Systematic Parameter Testing

1. **Baseline Establishment**: Record current performance with default parameters
2. **Single Parameter Variation**: Test one parameter at a time
3. **Combined Parameter Sets**: Test proven parameter combinations
4. **Performance Regression**: Validate improvements don't break other functionality
5. **Documentation**: Record parameter impacts and recommendations

### Automated Tuning Iterations

```bash
# Execute parameter tuning using b4m_HA_launch.sh
./b4m_HA_launch.sh --skip-agent --localization-test --tune-params

# This will:
# 1. Execute normal launch sequence through Step 7
# 2. Test baseline parameters (Step 8-9)
# 3. Apply parameter set 1, rebuild navigation, restart steps 5-7, test, log results
# 4. Apply parameter set 2, rebuild navigation, restart steps 5-7, test, log results
# 5. Apply parameter set 3, rebuild navigation, restart steps 5-7, test, log results
# 6. Generate comparison report
# 7. Restore best-performing parameter set

# With full automation:
./b4m_HA_launch.sh --autotest --localization-test --tune-params
```

## Success Criteria

### Localization Quality Thresholds
- **AMCL Convergence**: < 30 seconds to stable particle cloud
- **Pose Stability**: Position variance < 0.01m², orientation variance < 0.01 rad²
- **Transform Consistency**: No transform timeouts or frame errors
- **Particle Spread**: Effective particle count > 80% of total particles

### Navigation Performance Thresholds
- **Waypoint Accuracy**: Average position error < 0.15m, orientation error < 0.1 rad
- **Path Planning**: Success rate > 95% for standard waypoint sequences
- **Closed Loop**: Return position error < 0.25m, heading error < 0.15 rad
- **Consistency**: Performance variation < 20% across test runs

### System Integration Requirements
- **Test Duration**: Complete localization test in < 30 minutes
- **Non-Disruptive**: Preserve robot connection throughout testing
- **Automated**: No manual intervention required during test execution
- **Reproducible**: Consistent results across multiple test runs

## Maintenance and Updates

### Regular Testing Schedule
- **Weekly**: Automated baseline performance validation
- **Pre-Release**: Full parameter tuning and validation
- **Post-Changes**: Regression testing after localization-related code changes

### Parameter Set Maintenance
- **Version Control**: Track parameter changes and performance impacts
- **Documentation**: Maintain parameter tuning rationale and results
- **Rollback Capability**: Quick restoration of known-good parameter sets

## Implementation Checklist

### Phase 1: Extend b4m_HA_launch.sh Arguments
- [x] Add `--localization-test` argument parsing
- [x] Add `--tune-params` argument parsing  
- [x] Update help text to include new arguments
- [x] Add localization test configuration variables

### Phase 2: Add Localization Test Functions
- [x] Add `validate_localization_quality()` function
- [x] Add `validate_navigation_performance()` function
- [x] Add `test_global_localization()` helper function
- [x] Add `test_amcl_convergence()` helper function
- [x] Add `test_ekf_consistency()` helper function
- [x] Add `test_transform_stability()` helper function
- [x] Add `execute_test_waypoint_sequence_no_mqtt()` function
- [x] Add `test_navigation_accuracy_yahboom_map()` function
- [x] Add `test_localization_drift()` helper function

### Phase 3: Integrate Test Steps
- [x] Add Step 8 (Localization Quality) after existing Step 7
- [x] Add Step 9 (Navigation Performance) after Step 8
- [x] Update success reporting to include localization steps
- [x] Ensure cleanup handles localization test processes

### Phase 4: Parameter Tuning Infrastructure
- [x] Create `localization_tests/` directory structure
- [x] Create test waypoint sequence JSON file for yahboom_map.yaml
- [x] Create parameter tuning sets (aggressive_tuning.yaml, conservative_tuning.yaml)
- [x] Add parameter tuning iteration logic with 5-minute test cycles
- [x] Add minimal rebuild logic: `yahboomcar_nav` for AMCL/nav, `yahboomcar_bringup` for EKF
- [x] Add restart mechanism for Steps 5-7 after parameter changes
- [x] Add runtime parameter update support (ros2 param set) before rebuilding
- [x] Add global localization support (no manual pose setting required)
- [x] Add MQTT-independent navigation testing using ROS2 actions
- [x] Add 2x timeout handling (10 minutes max) for unresponsive tests
- [x] Add parameter backup and restoration system
- [x] Add results logging and directory structure

### Phase 5: Testing and Validation  
- [x] Test `--localization-test` argument functionality (framework works, needs Step 6 pose estimate fix)
- [x] Verify error handling stops script appropriately when tests fail
- [ ] Fix Step 6 automatic pose estimate for reliable testing  
- [ ] Test integration with `--autotest` mode
- [ ] Test parameter tuning iterations
- [ ] Validate logging and results analysis
- [ ] Test cleanup and restoration of parameters

This localization tuning test plan leverages the existing robust `b4m_HA_launch.sh` infrastructure to provide comprehensive assessment tools for improving navigation reliability without requiring additional scripts.

## Implementation Lessons Learned & Troubleshooting

### Critical Issues Discovered During Implementation

#### 1. Step 6 Automatic Pose Estimate Reliability Issue
**Problem**: The automatic 2D pose estimate in Step 6 doesn't work reliably, causing localization tests to fail even with a connected robot.

**Root Cause**: The Python script in Step 6 that publishes to `/initialpose` topic may not be executing properly or AMCL may not be processing the pose estimate.

**Symptoms**:
- Localization tests fail with "ERROR: Global localization failed - no AMCL pose within 60 seconds"
- AMCL node is active but `/amcl_pose` topic publishes no data
- Manual 2D pose estimate in RViz works immediately

**Debug Commands**:
```bash
# Check if AMCL is running and active
ros2 node list | grep amcl
ros2 lifecycle get /amcl

# Check if AMCL pose topic is publishing
timeout 5 ros2 topic echo /amcl_pose --once

# Check if initialpose topic exists and has subscribers
ros2 topic info /initialpose
```

**Temporary Workaround**: Manually set 2D pose estimate in RViz before running localization tests.

**Proper Fix Needed**: Investigate and fix Step 6 automatic pose initialization to work reliably without manual intervention.

#### 2. Function Definition Order Issues
**Problem**: Early implementation had `debug_log` function calls before the function was defined, causing "command not found" errors.

**Solution**: Removed early debug_log calls from initialization section since function wasn't defined yet.

**Lesson**: Ensure all function calls come after function definitions in bash scripts.

#### 3. Error Handling Implementation Issues
**Problem**: Initial implementation continued to launch Robot Manager GUI even after localization tests failed.

**Root Cause**: Manual mode didn't have proper error handling to exit on test failures.

**Solution**: Added explicit `exit 1` calls in manual mode when tests fail:
```bash
if validate_localization_quality; then
    echo "✅ Localization quality tests passed"
else
    echo "❌ Localization quality tests failed"
    echo ""
    echo "Localization tests must pass before continuing."
    echo "Check robot connection, sensors, and navigation system."
    echo "Exiting..."
    exit 1
fi
```

#### 4. ROS2 Command Syntax Issues
**Problem**: Initial test function used incorrect `ros2 topic echo` syntax with `--timeout` flag.

**Incorrect**: `ros2 topic echo /amcl_pose --once --timeout 5`
**Correct**: `timeout 5 ros2 topic echo /amcl_pose --once`

**Lesson**: Use system `timeout` command instead of ROS2-specific timeout options.

### Testing Environment Requirements

#### Hardware Prerequisites
- **Physical Robot**: Must be powered on and connected via Micro-ROS
- **Sensor Data**: LIDAR and IMU must be publishing data for localization
- **Network**: Robot must be on same network as development machine
- **Map**: `yahboom_map.yaml` must exist and be valid

#### Software Prerequisites
- **ROS2 Humble**: Full installation with Nav2 stack
- **Navigation Stack**: All navigation nodes must be running and active
- **AMCL**: Must be in active lifecycle state
- **Transform Tree**: Complete `map→odom→base_link` transform chain

#### Testing Sequence Dependencies
1. **Step 3**: Robot bringup must complete successfully (sensor data available)
2. **Step 4**: RViz must be running for visualization 
3. **Step 5**: Navigation stack must be fully launched and active
4. **Step 6**: **CRITICAL** - Automatic pose estimate must work for tests to pass
5. **Step 7**: Waypoint navigation node must be running for navigation tests

### Debugging Workflow

#### When Localization Tests Fail
1. **Check Robot Connection**:
   ```bash
   ros2 node list | grep -E "(amcl|nav|ekf)"
   ros2 topic list | grep -E "(scan|odom|imu)"
   ```

2. **Verify AMCL State**:
   ```bash
   ros2 lifecycle get /amcl
   timeout 5 ros2 topic echo /amcl_pose --once
   ```

3. **Check Transform Tree**:
   ```bash
   ros2 run tf2_tools view_frames.py
   ros2 run tf2_ros tf2_echo map base_link
   ```

4. **Verify Sensor Data**:
   ```bash
   ros2 topic hz /scan
   ros2 topic hz /odom
   ros2 topic hz /imu/data
   ```

5. **Test Manual Pose Estimate**:
   - Open RViz
   - Click "2D Pose Estimate" tool
   - Set pose on map
   - Verify AMCL starts publishing poses

#### When Navigation Tests Fail
1. **Check Navigation Stack**:
   ```bash
   ros2 lifecycle get /bt_navigator
   ros2 lifecycle get /planner_server
   ros2 lifecycle get /controller_server
   ```

2. **Test Manual Navigation**:
   - Open RViz
   - Use "Nav2 Goal" tool to set navigation goal
   - Verify robot attempts to navigate

3. **Check Action Servers**:
   ```bash
   ros2 action list | grep navigate
   ros2 action info /navigate_to_pose
   ```

### Implementation Architecture Notes

#### Function Structure
- **9 Validation Functions**: Each tests specific localization/navigation aspect
- **Modular Design**: Functions can be called independently for debugging
- **Timeout Handling**: All tests have appropriate timeouts to prevent hanging
- **Error Propagation**: Failed tests return non-zero exit codes

#### Integration Points
- **Steps 8-9**: Added after existing Step 7 in launch sequence
- **Autotest Mode**: Full integration with existing automated testing
- **Parameter Tuning**: Supports iterative parameter testing with rebuilds
- **Cleanup**: Proper cleanup on failures to prevent resource leaks

#### File Locations
- **Test Directory**: `/home/mike/projects/b4m_yahboom/localization_tests/`
- **Parameter Sets**: `localization_tests/param_sets/*.yaml`
- **Test Waypoints**: `localization_tests/test_waypoints.json`
- **Results**: `localization_tests/results/` (auto-created, in .gitignore)

### Recommendations for Re-implementation

#### If Starting From Scratch
1. **Fix Step 6 First**: Ensure automatic pose estimation works reliably before implementing tests
2. **Test Incrementally**: Implement and test each validation function individually
3. **Use Debug Mode**: Always test with `--debug` flag initially to see detailed logging
4. **Verify Prerequisites**: Ensure all hardware and software dependencies are met
5. **Test Manual Mode First**: Get manual mode working before implementing autotest integration

#### Critical Success Factors
- **Automatic Pose Initialization**: Must work without manual intervention
- **Proper Error Handling**: Tests must fail fast and exit cleanly
- **Transform Tree Validation**: Verify complete localization pipeline
- **Sensor Data Validation**: Ensure all required sensors are publishing

This documentation captures the key implementation challenges and solutions discovered during the development process, providing a roadmap for future implementation or debugging efforts.