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
- [ ] Add `--localization-test` argument parsing
- [ ] Add `--tune-params` argument parsing  
- [ ] Update help text to include new arguments
- [ ] Add localization test configuration variables

### Phase 2: Add Localization Test Functions
- [ ] Add `validate_localization_quality()` function
- [ ] Add `validate_navigation_performance()` function
- [ ] Add `test_amcl_convergence()` helper function
- [ ] Add `test_ekf_consistency()` helper function
- [ ] Add `test_transform_stability()` helper function
- [ ] Add `execute_test_waypoint_sequence()` function
- [ ] Add `test_closed_loop_accuracy()` function

### Phase 3: Integrate Test Steps
- [ ] Add Step 8 (Localization Quality) after existing Step 7
- [ ] Add Step 9 (Navigation Performance) after Step 8
- [ ] Update success reporting to include localization steps
- [ ] Ensure cleanup handles localization test processes

### Phase 4: Parameter Tuning Infrastructure
- [ ] Create `localization_tests/` directory structure
- [ ] Create test waypoint sequence JSON file for yahboom_map.yaml
- [ ] Create baseline parameter backup system (from earliest Git commit)
- [ ] Add parameter tuning iteration logic (5 minutes per set)
- [ ] Add minimal rebuild logic: `yahboomcar_nav` for AMCL/nav, `yahboomcar_bringup` for EKF
- [ ] Add restart mechanism for Steps 5-7 after parameter changes
- [ ] Add runtime parameter update support (ros2 param set) before rebuilding
- [ ] Add global localization support (no manual pose setting required)
- [ ] Add MQTT-independent navigation testing
- [ ] Add 2x timeout handling (10 minutes max) for unresponsive tests

### Phase 5: Testing and Validation
- [ ] Test `--localization-test` argument functionality
- [ ] Test integration with `--autotest` mode
- [ ] Test parameter tuning iterations
- [ ] Validate logging and results analysis
- [ ] Test cleanup and restoration of parameters

This localization tuning test plan leverages the existing robust `b4m_HA_launch.sh` infrastructure to provide comprehensive assessment tools for improving navigation reliability without requiring additional scripts.