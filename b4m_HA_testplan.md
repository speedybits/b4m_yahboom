# B4M Robot Automated Test Plan for b4m_launch.sh

## Overview

This document outlines the automated regression test plan for the `b4m_launch.sh` script. The goal is to create a fully automated test that validates the robot launch sequence starting from Step 2 (after Micro-ROS Agent), detecting failures early and providing comprehensive logging for debugging.

**Note**: Testing begins after Step 1 (Micro-ROS Agent Launch) as this requires human intervention to power-on or reset the physical robot. We assume the Micro-ROS agent is already running and the robot is connected.

## Test Objectives

1. **Automated Execution**: Run launch steps 2-7 without human intervention (assumes Step 1 complete)
2. **Early Failure Detection**: Abort on first failure to prevent cascading issues
3. **Clean Shutdown**: Automatically run cleanup procedures on failure
4. **Comprehensive Logging**: Capture detailed logs for each step and failure conditions
5. **Regression Testing**: Ensure script continues to work after code changes

## Test Architecture

### Core Components

1. **Test Runner Script**: `b4m_HA_autotest.sh`
2. **Step Validators**: Individual validation functions for each launch step
3. **Failure Handler**: Cleanup and logging on test failures
4. **Test Report Generator**: Summary of test results and failure analysis

### Test Flow

```
Start Test → Step 2 → Validate → Step 3 → Validate → ... → Step 7 → Success Report
     ↓           ↓        ↓         ↓        ↓              ↓         ↓
   Setup      Execute  Check     Execute  Check         Execute   Cleanup
     ↓           ↓     Success      ↓     Success          ↓         ↓
 Pre-checks     Log   Continue     Log   Continue        Log    Generate
   (Verify      ↓      OR          ↓      OR              ↓     Report
   Step 1)      ↓   → FAIL →       ↓   → FAIL →           ↓        ↓
                 ↓   Cleanup &      ↓   Cleanup &          ↓    Archive
                 ↓   Abort          ↓   Abort              ↓     Logs
                 ↓                  ↓                      ↓
                 └─────────────────┴──────────────────────┘
```

## Test Steps and Validation Criteria

### Pre-Test Setup
- **Environment Check**: Verify workspace, dependencies, and required files
- **Clean State**: Ensure no previous robot processes are running (except Micro-ROS agent)
- **Micro-ROS Agent Assumption**: Always assume the Micro-ROS agent is running correctly and robot is connected
- **Physical Robot**: Assume robot is powered on and connected to Micro-ROS agent

### Step 1: Micro-ROS Agent (PRE-REQUISITE)
**Status**: Always assumed to be running correctly
**No Validation Required**:
- Do not check if Docker container is running
- Do not verify UDP port 8090 is listening
- Do not check robot connection or agent logs
- Always assume Step 1 is complete and functional
**Failure Actions**: N/A - no validation performed

### Step 2: Physical Robot Power-On Verification
**Command**: Verify robot connection status
**Validation Criteria**:
- Micro-ROS agent shows active robot connection messages
- Expected robot topics are being published (basic connectivity check)
- No connection timeout or error messages
**Timeout**: 10 seconds
**Failure Actions**: Abort test - robot not properly connected

### Step 3: Car's Underlying Data Processing
**Command**: `ros2 launch yahboomcar_bringup yahboomcar_bringup_launch.py`
**Validation Criteria**:
- Launch process starts without immediate exit
- Required ROS2 nodes are active:
  - `/complementary_filter_gain_node`
  - `/ekf_node`
  - `/robot_state_publisher`
  - `/joint_state_publisher`
- Required topics are published:
  - `/tf`
  - `/tf_static`
  - `/odom`
  - `/imu/data`
- No critical error messages in logs
- EKF node publishes odom→base_link transforms
**Timeout**: 10 seconds
**Failure Actions**: Kill processes, log errors, abort test

### Step 4: RViz Visualization
**Command**: `ros2 launch yahboomcar_nav display_launch.py`
**Validation Criteria**:
- RViz2 process starts successfully
- RViz2 connects to ROS2 system
- No OpenGL or display errors (in headless mode, check for proper startup)
**Timeout**: 10 seconds
**Failure Actions**: Kill RViz, log errors, abort test

### Step 5: Navigation System
**Command**: `ros2 launch yahboomcar_nav waypoint_navigation_launch.py`
**Validation Criteria**:
- Navigation container starts successfully
- Core navigation nodes are active:
  - `/map_server`
  - `/amcl`
  - `/controller_server`
  - `/planner_server`
  - `/bt_navigator`
- Required topics exist:
  - `/map`
  - `/amcl_pose`
  - `/global_costmap/costmap`
  - `/local_costmap/costmap`
- Map is successfully loaded and published
- AMCL lifecycle state is ACTIVE
- No transform timeout errors (map→odom→base_link chain)
- Waypoint navigation node starts (termios error causes test failure)
**Timeout**: 10 seconds
**Failure Actions**: Kill navigation processes, log errors, abort test

### Step 6: Automatic Pose Estimation
**Command**: Python script for pose estimation
**Validation Criteria**:
- Python script executes successfully
- Initial pose is published to `/initialpose` topic
- AMCL receives and processes the pose estimate
- AMCL pose updates from previous position
- Transform chain `map→odom→base_link` is established
- No "frame does not exist" errors after pose setting
**Timeout**: 10 seconds
**Failure Actions**: Log pose estimation failure, abort test

### Step 7: B4M Waypoint Navigation Node with MQTT
**Command**: `python3 b4m_waypoint_nav.py` with MQTT parameters
**Validation Criteria**:
- Python process starts successfully
- MQTT broker connection established (or acceptable connection failure for test environment)
- ROS2 node `/b4m_waypoint_nav` is active
- Required service interfaces are available
- No critical Python exceptions in logs
**Timeout**: 10 seconds
**Failure Actions**: Kill process, log errors, abort test


## Test Implementation Details

### Script Structure: `b4m_HA_autotest.sh`

```bash
#!/bin/bash
# Automated test runner for b4m_launch.sh

# Configuration
WORKSPACE_ROOT=$(cd "$(dirname "$0")" && pwd)
TEST_TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
TEST_LOG_DIR="$WORKSPACE_ROOT/test_logs"
TEST_RESULTS_DIR="$WORKSPACE_ROOT/test_results"
MAIN_TEST_LOG="$TEST_LOG_DIR/autotest_$TEST_TIMESTAMP.log"

# Test control variables
MOCK_PHYSICAL_ROBOT=true  # Set to false for actual robot testing
HEADLESS_MODE=true        # For CI/CD environments
MAX_STEP_TIMEOUT=120      # Maximum timeout for any single step

# Test functions for each step
verify_step1_prerequisite() { ... }
validate_step2() { ... }
validate_step3() { ... }
validate_step4() { ... }
validate_step5() { ... }
validate_step6() { ... }
validate_step7() { ... }

# Main test execution loop
run_automated_test() { ... }
```

### Validation Functions

Each step will have a dedicated validation function that:
1. Executes the command non-interactively
2. Monitors process health and output
3. Validates specific success criteria
4. Returns success/failure status with detailed logging

### Failure Handling

```bash
handle_test_failure() {
    local failed_step=$1
    local error_message=$2
    
    log_test_failure "TEST FAILED at Step $failed_step: $error_message"
    
    # Capture system state for debugging
    capture_debug_snapshot "$failed_step"
    
    # Run cleanup preserving Micro-ROS agent for next test
    ./b4m_shutdown.sh --keep-agent
    
    # Generate failure report
    generate_failure_report "$failed_step" "$error_message"
    
    exit 1
}
```

### Success Criteria Validation

For each step, implement specific checks:
- Process health monitoring
- ROS2 node availability checks
- Topic/service existence validation
- Log pattern matching for success/error indicators
- System resource utilization monitoring
- Transform tree validation

## Test Environment Setup

### Dependencies
- Docker (for Micro-ROS agent) - must be running before test
- ROS2 Humble
- All robot workspace packages built
- Required Python packages (MQTT client)
- Network connectivity for MQTT (or mock for testing)
- Physical robot powered on and connected to Micro-ROS agent

### Cleanup Behavior
- **On Test Failure**: Run `b4m_shutdown.sh --keep-agent` to preserve Micro-ROS agent connection
- **Agent Preservation**: Keeps robot connected for immediate re-testing without manual restart
- **Process Cleanup**: Stops all ROS2 nodes, navigation, and Python processes while maintaining robot connection

### Mock Components
- **MQTT Broker**: Optional mock broker for testing MQTT functionality
- **Display**: Xvfb for headless RViz testing

### CI/CD Integration
- **GitHub Actions**: Automated test runs on pull requests
- **Test Artifacts**: Automatic archival of logs and failure reports
- **Notification**: Test results sent to development team
- **Performance Tracking**: Test execution time and success rate metrics

## Test Reporting

### Success Report
```
B4M Robot Launch Test - PASSED
===============================
Test Run: 2025-07-24_17:30:15
Duration: 03:45
All 6 tested steps completed successfully

Step Summary:
✅ Step 1: Micro-ROS Agent (assumed running - prerequisite)
✅ Step 2: Robot Connection (5s)
✅ Step 3: Data Processing (28s)
✅ Step 4: RViz Launch (15s)
✅ Step 5: Navigation System (45s)
✅ Step 6: Pose Estimation (8s)
✅ Step 7: MQTT Navigation (18s)

Logs archived to: test_results/success_20250724_173015/
```

### Failure Report
```
B4M Robot Launch Test - FAILED
===============================
Test Run: 2025-07-24_17:30:15
Failed at: Step 5 (Navigation System)
Duration: 02:14 (aborted)

Error: Transform timeout - map frame not available
AMCL lifecycle activation failed after 60 seconds

Debug Information:
- ROS2 nodes status: [captured]
- Transform tree: [captured]
- Process list: [captured]
- Log excerpts: [relevant errors]

Cleanup Status: ✅ Complete (Micro-ROS agent preserved)
Logs archived to: test_results/failure_20250724_173015/

Recommended Actions:
1. Check AMCL configuration parameters
2. Verify map file integrity
3. Review EKF node timing and transforms
```

## Integration with Existing Scripts

The automated test will:
1. **Reuse Logic**: Leverage existing `b4m_launch.sh` command structure
2. **Non-Interactive**: Execute commands without user prompts
3. **Cleanup Integration**: Use `b4m_shutdown.sh --keep-agent` for proper cleanup while preserving robot connection
4. **Log Compatibility**: Use same logging format and directory structure

## Test Execution Commands

```bash
# Full automated test
./b4m_HA_autotest.sh

# Headless CI mode (requires robot pre-connected)
./b4m_HA_autotest.sh --headless

# Test specific steps only (steps 2-7)
./b4m_HA_autotest.sh --steps 2,3,5

# Debug mode with verbose logging
./b4m_HA_autotest.sh --debug --verbose
```

## Maintenance and Updates

1. **Test Updates**: Sync test validation criteria with script changes
2. **Timeout Tuning**: Adjust timeouts based on system performance
3. **Mock Improvements**: Enhance mocking for better test coverage
4. **Validation Enhancement**: Add more sophisticated success criteria
5. **Performance Monitoring**: Track test execution trends over time

## Success Metrics

1. **Test Reliability**: >95% consistent results
2. **Failure Detection**: Catch 100% of critical launch failures
3. **Execution Time**: Complete test in <8 minutes
4. **False Positives**: <5% false failure rate
5. **Coverage**: Test all critical launch sequence components

This automated test plan ensures robust validation of the B4M robot launch sequence while providing comprehensive debugging information for any failures that occur.

---

## Implementation Checklist

### Phase 1: Core Infrastructure
- [ ] **Modify b4m_launch.sh argument parsing**
  - [ ] Add `--autotest` flag support
  - [ ] Add `--debug` flag for verbose logging
  - [ ] Update help text and usage documentation

- [ ] **Create autotest mode execution logic**
  - [ ] Replace interactive user prompts with automatic execution
  - [ ] Implement non-terminal command execution for autotest mode
  - [ ] Add timeout handling for each step

- [ ] **Implement step validation framework**
  - [ ] Create `validate_step_success()` function template
  - [ ] Add step timeout monitoring
  - [ ] Implement failure detection and early abort

### Phase 2: Step-Specific Validation Functions
- [x] **Step 1 Prerequisite Verification (REMOVED)**
  - [x] No validation required - always assume Micro-ROS agent is running
  - [x] Remove all agent verification checks
  - [x] Update autotest to skip Step 1 validation

- [ ] **Step 2 Robot Connection (`validate_step2()`)**
  - [ ] Implement connection status check
  - [ ] Skip manual confirmation in autotest mode

- [ ] **Step 3 Data Processing (`validate_step3()`)**
  - [ ] Check required ROS2 nodes are active
  - [ ] Verify required topics are published
  - [ ] Validate EKF transform publishing
  - [ ] Monitor for critical error messages

- [ ] **Step 4 RViz Launch (`validate_step4()`)**
  - [ ] Verify RViz2 process starts successfully
  - [ ] Check ROS2 system connection
  - [ ] Handle headless mode validation

- [ ] **Step 5 Navigation System (`validate_step5()`) - CRITICAL**
  - [ ] Verify navigation container startup
  - [ ] Check core navigation nodes are active
  - [ ] Validate `/map` topic exists and has data
  - [ ] Verify AMCL lifecycle state is ACTIVE
  - [ ] Check transform chain `map→odom→base_link`
  - [ ] Validate no "frame does not exist" errors
  - [ ] Handle waypoint navigation termios errors as failures

- [ ] **Step 6 Pose Estimation (`validate_step6()`)**
  - [ ] Verify pose script execution
  - [ ] Check pose published to `/initialpose`
  - [ ] Validate AMCL receives and processes pose
  - [ ] Confirm transform chain establishment

- [ ] **Step 7 MQTT Navigation (`validate_step7()`)**
  - [ ] Verify Python process startup
  - [ ] Check ROS2 node activation
  - [ ] Validate service interfaces
  - [ ] Monitor for Python exceptions

### Phase 3: Error Handling and Cleanup
- [ ] **Implement failure handler (`handle_test_failure()`)**
  - [ ] Capture system state for debugging
  - [ ] Run `b4m_shutdown.sh --keep-agent`
  - [ ] Generate detailed failure reports
  - [ ] Archive logs with timestamps

- [ ] **Debug snapshot capture (`capture_debug_snapshot()`)**
  - [ ] ROS2 node list and status
  - [ ] Transform tree dump
  - [ ] Process list capture
  - [ ] Topic/service availability
  - [ ] Log excerpt extraction

### Phase 4: Reporting and Logging
- [ ] **Test report generation**
  - [ ] Success report with timing details
  - [ ] Failure report with debug information
  - [ ] Log archival system
  - [ ] Cleanup status verification

- [ ] **Enhanced logging system**
  - [ ] Step-by-step progress logging
  - [ ] Validation result logging
  - [ ] Error message capture and formatting
  - [ ] Debug mode verbose output

### Phase 5: Testing and Validation
- [ ] **Test the autotest system**
  - [ ] Verify autotest mode executes correctly
  - [ ] Test failure handling and cleanup
  - [ ] Validate log generation and archival
  - [ ] Confirm `--keep-agent` preserves connection

- [ ] **Integration testing**
  - [ ] Test with current broken navigation (should fail at Step 5)
  - [ ] Verify proper cleanup after failures
  - [ ] Test multiple consecutive runs
  - [ ] Validate timing and timeout behavior

### Phase 6: Documentation and Polish
- [ ] **Update existing documentation**
  - [ ] Update `b4m_launch.sh` help text
  - [ ] Add autotest usage examples
  - [ ] Document validation criteria

- [ ] **Performance optimization**
  - [ ] Tune timeout values based on testing
  - [ ] Optimize validation checks for speed
  - [ ] Minimize false positives

---

## Current Status
**PLANNING COMPLETE** ✅  
**IMPLEMENTATION PENDING** ⏳

Next step: Begin Phase 1 implementation by modifying `b4m_launch.sh` to support `--autotest` mode.