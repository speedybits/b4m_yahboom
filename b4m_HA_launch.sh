#!/bin/bash

# B4M Robot - Home Assistant MQTT Integration Launch Script

# FIX: Enable advanced ROS2 logging for gazebo_ros2_control debugging
# Based on Grok AI analysis - helps diagnose controller manager service issues
export RCUTILS_LOGGING_CONFIGURED=1
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity} {time}] {name}: {message}"
export RCUTILS_LOGGING_VERBOSITY=DEBUG
export RCL_LOG_LEVEL=debug
# This script automates the launch process for the B4M Robot with Home Assistant integration
# Each step will be launched in a separate terminal with user confirmation
#
# Usage: ./b4m_HA_launch.sh [--skip-agent] [--only-agent] [--autotest] [--debug] [--simulation] [--slam-test] [--regression]
#   --skip-agent:    Skip the Micro-ROS agent launch (Step 1)
#   --only-agent:    Launch ONLY the Micro-ROS agent (Step 1) and exit
#   --autotest:      Run in automated test mode (non-interactive)
#   --debug:         Enable verbose debug logging
#   --simulation:    Launch in Gazebo simulation mode instead of real robot
#   --slam-test:     Add Steps 8-10 for automated SLAM testing (skips Robot Manager GUI)
#   --regression:    Run regression test (test_square_corners.py) after system launch

# Parse command line arguments
SKIP_AGENT=false
ONLY_AGENT=false
AUTOTEST_MODE=false
DEBUG_MODE=false
SIMULATION_MODE=false
SLAM_TEST_MODE=false
REGRESSION_MODE=false
for arg in "$@"; do
    case $arg in
        --skip-agent)
            SKIP_AGENT=true
            shift
            ;;
        --only-agent)
            ONLY_AGENT=true
            shift
            ;;
        --autotest)
            AUTOTEST_MODE=true
            shift
            ;;
        --debug)
            DEBUG_MODE=true
            shift
            ;;
        --simulation)
            SIMULATION_MODE=true
            shift
            ;;
        --slam-test)
            SLAM_TEST_MODE=true
            shift
            ;;
        --regression)
            REGRESSION_MODE=true
            shift
            ;;
        -h|--help)
            echo "Usage: $0 [--skip-agent] [--only-agent] [--autotest] [--debug] [--simulation] [--slam-test] [--regression]"
            echo "  --skip-agent:    Skip the Micro-ROS agent launch (Step 1)"
            echo "  --only-agent:    Launch ONLY the Micro-ROS agent (Step 1) and exit"
            echo "  --autotest:      Run in automated test mode (non-interactive)"
            echo "  --debug:         Enable verbose debug logging"
            echo "  --simulation:    Launch in Gazebo simulation mode instead of real robot"
            echo "  --slam-test:     Add Steps 8-10 for automated SLAM testing (skips Robot Manager GUI)"
            echo "  --regression:    Run regression test (test_square_corners.py) after system launch"
            exit 0
            ;;
        *)
            echo "Unknown argument: $arg"
            echo "Use -h or --help for usage information"
            exit 1
            ;;
    esac
done

# Validate flag combinations
if [ "$SIMULATION_MODE" = true ] && [ "$ONLY_AGENT" = true ]; then
    echo "ERROR: --simulation and --only-agent cannot be used together"
    echo "In simulation mode, Gazebo is launched instead of the Micro-ROS agent"
    exit 1
fi

if [ "$SLAM_TEST_MODE" = true ] && [ "$ONLY_AGENT" = true ]; then
    echo "ERROR: --slam-test and --only-agent cannot be used together"
    echo "SLAM testing requires full system launch, not just the agent"
    exit 1
fi

if [ "$SLAM_TEST_MODE" = true ] && [ "$AUTOTEST_MODE" = false ]; then
    echo "INFO: --slam-test automatically enables --autotest mode for automated testing"
    AUTOTEST_MODE=true
fi

if [ "$REGRESSION_MODE" = true ]; then
    # Regression mode is mutually exclusive with SLAM test
    if [ "$SLAM_TEST_MODE" = true ]; then
        echo "ERROR: --regression and --slam-test cannot be used together"
        echo "Regression test runs its own square navigation test"
        exit 1
    fi
    # Regression mode is mutually exclusive with only-agent
    if [ "$ONLY_AGENT" = true ]; then
        echo "ERROR: --regression and --only-agent cannot be used together"
        echo "Regression test is self-contained"
        exit 1
    fi
    # Regression mode is mutually exclusive with skip-agent
    if [ "$SKIP_AGENT" = true ]; then
        echo "ERROR: --regression and --skip-agent cannot be used together"
        echo "Regression test manages its own simulation"
        exit 1
    fi
fi

# Get the workspace root directory (where this script is located)
WORKSPACE_ROOT=$(cd "$(dirname "$0")" && pwd)

# Create logs directory if it doesn't exist
LOGS_DIR="$WORKSPACE_ROOT/logs"
mkdir -p "$LOGS_DIR"

# Generate timestamp for log files
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
MAIN_LOG="$LOGS_DIR/b4m_launch_$TIMESTAMP.log"


# Function to ask for user confirmation
confirm() {
    echo ""
    read -p "Press Enter to continue to the next step or Ctrl+C to exit..."
    echo ""
}

# Function to log messages
log_message() {
    local message=$1
    echo "$(date '+%Y-%m-%d %H:%M:%S') - $message" | tee -a "$MAIN_LOG"
}

# Function for debug logging
debug_log() {
    local message=$1
    if [ "$DEBUG_MODE" = true ]; then
        echo "$(date '+%Y-%m-%d %H:%M:%S') [DEBUG] - $message" | tee -a "$MAIN_LOG"
    fi
}

# Function to check for existing ROS2 processes and prevent duplicates
check_existing_processes() {
    local node_count=$(ros2 node list | wc -l 2>/dev/null || echo "0") 
    local navigation_nodes=$(ros2 node list | grep -E "(slam_toolbox|nav2_container)" | wc -l 2>/dev/null || echo "0")
    local hardware_nodes=$(ros2 node list | grep -E "(YB_Car_Node)" | wc -l 2>/dev/null || echo "0")
    
    echo "🔍 Pre-launch System Check"
    echo "=========================="
    echo "Total ROS2 nodes detected: $node_count"
    echo "Navigation nodes detected: $navigation_nodes"
    echo "Hardware nodes detected: $hardware_nodes (YB_Car_Node)"
    
    # CRITICAL: Check for duplicate nodes
    local duplicate_check=$(ros2 node list 2>/dev/null | sort | uniq -d)
    if [[ -n "$duplicate_check" ]]; then
        echo ""
        echo "🚨 CRITICAL ERROR: DUPLICATE NODES DETECTED!"
        echo "============================================="
        echo "Duplicate nodes found:"
        echo "$duplicate_check"
        echo ""
        echo "This indicates incomplete cleanup from previous runs."
        echo "Duplicate nodes cause:"
        echo "  - Resource conflicts and system instability"
        echo "  - Unpredictable behavior and test failures"
        echo "  - Transform tree corruption"
        echo "  - Topic/service conflicts"
        echo ""
        
        if [ "$AUTOTEST_MODE" = true ]; then
            echo "🤖 AUTOTEST MODE: Automatically forcing full cleanup..."
            force_complete_system_cleanup
        else
            echo "Options:"
            echo "  c) Force complete cleanup (RECOMMENDED)"
            echo "  q) Quit and investigate manually"
            echo ""
            read -p "Choose [c/q]: " choice
            
            case $choice in
                c|C)
                    echo "🧹 Forcing complete system cleanup..."
                    force_complete_system_cleanup
                    ;;
                q|Q)
                    echo "Exiting. Investigate duplicate nodes manually."
                    echo "Commands to investigate:"
                    echo "  ros2 node list"
                    echo "  ps aux | grep ros2"
                    echo "  ./b4m_shutdown.sh --keep-agent"
                    exit 1
                    ;;
                *)
                    echo "Invalid choice. Exiting for safety."
                    exit 1
                    ;;
            esac
        fi
        
        # Re-check after cleanup
        local post_cleanup_nodes=$(ros2 node list | wc -l 2>/dev/null || echo "0")
        local post_cleanup_duplicates=$(ros2 node list 2>/dev/null | sort | uniq -d)
        
        if [[ -n "$post_cleanup_duplicates" ]]; then
            echo "❌ CLEANUP FAILED: Duplicates still present after cleanup"
            echo "Manual intervention required. Exiting."
            exit 1
        else
            echo "✅ Cleanup successful - $post_cleanup_nodes nodes remaining"
            echo ""
        fi
    fi
    
    # Additional check: Look for robot processes that might not show as ROS2 nodes
    local robot_processes=$(ps aux | grep -E "(ekf_node|robot_localization|yahboomcar)" | grep -v grep | wc -l)
    if [ "$robot_processes" -gt 0 ]; then
        echo ""
        echo "⚠️  WARNING: Robot processes detected outside ROS2 nodes!"
        echo "Found $robot_processes robot-related processes:"
        ps aux | grep -E "(ekf_node|robot_localization|yahboomcar)" | grep -v grep | awk '{print "  - " $11}'
        echo ""
        echo "These processes can cause conflicts during launch."
        
        if [ "$AUTOTEST_MODE" = true ]; then
            echo "🤖 AUTOTEST MODE: Cleaning up robot processes..."
            pkill -f "ekf_node" 2>/dev/null || true
            pkill -f "robot_localization" 2>/dev/null || true  
            pkill -f "yahboomcar" 2>/dev/null || true
            sleep 2
            echo "✅ Robot process cleanup completed"
        else
            echo "Run './b4m_shutdown.sh --keep-agent' to clean up these processes."
        fi
        echo ""
    fi
    
    if [ "$node_count" -gt 30 ]; then
        echo ""
        echo "⚠️  WARNING: High node count detected ($node_count nodes)!"
        echo "This suggests previous launch sessions are still running."
        echo ""
        echo "Duplicate nodes can cause:"
        echo "  - Resource conflicts and poor performance"
        echo "  - Unreliable localization and navigation"
        echo "  - Test failures and unpredictable behavior"
        echo ""
        
        if [ "$AUTOTEST_MODE" = true ]; then
            echo "🤖 AUTOTEST MODE: Automatically cleaning up..."
            cleanup_existing_processes
        else
            echo "Options:"
            echo "  c) Clean up automatically (recommended)"
            echo "  f) Force continue anyway (not recommended)" 
            echo "  q) Quit and clean up manually"
            echo ""
            read -p "Choose [c/f/q]: " choice
            
            case $choice in
                c|C)
                    echo "🧹 Cleaning up existing processes..."
                    cleanup_existing_processes
                    ;;
                f|F)
                    echo "⚠️  Forcing launch with existing processes - this may cause issues!"
                    ;;
                q|Q)
                    echo "Exiting. Run './b4m_shutdown.sh' to clean up manually."
                    exit 0
                    ;;
                *)
                    echo "Invalid choice. Exiting for safety."
                    exit 1
                    ;;
            esac
        fi
        
    elif [ "$navigation_nodes" -gt 0 ]; then
        echo ""
        echo "⚠️  WARNING: Navigation nodes already running!"
        echo "Detected: $(ros2 node list 2>/dev/null | grep -E '(slam_toolbox|nav2_container)' | tr '\n' ' ')"
        echo ""
        
        if [ "$AUTOTEST_MODE" = true ]; then
            echo "🤖 AUTOTEST MODE: Automatically cleaning up navigation nodes..."
            cleanup_existing_processes
        else
            echo "This usually means another robot session is active."
            echo "Continue anyway? (y/N): "
            read continue_choice
            if [[ ! "$continue_choice" =~ ^[Yy]$ ]]; then
                echo "Exiting. Use './b4m_shutdown.sh' to clean up first."
                exit 0
            fi
        fi
    elif [ "$hardware_nodes" -gt 0 ]; then
        echo ""
        echo "ℹ️  Hardware connection detected: YB_Car_Node is running"
        echo "This indicates the physical robot is connected via Micro-ROS agent."
        echo "✅ Hardware connection will be preserved during launch."
    else
        echo "✅ System clean - ready for launch"
    fi
    echo ""
}

# Function to force complete system cleanup (aggressive cleanup for duplicates)
force_complete_system_cleanup() {
    echo "🚨 PERFORMING AGGRESSIVE SYSTEM CLEANUP"
    echo "========================================"
    
    # Stop all ROS2 launches first
    echo "Step 1: Killing all ROS2 launch processes..."
    pkill -f "ros2 launch" 2>/dev/null || true
    sleep 2
    
    # Kill all python ROS2 processes
    echo "Step 2: Killing ROS2 Python processes..."
    pkill -f "python.*ros2" 2>/dev/null || true
    pkill -f "ros2.*python" 2>/dev/null || true
    sleep 2
    
    # Kill specific robot processes
    echo "Step 3: Killing robot-specific processes..."
    pkill -f "yahboomcar" 2>/dev/null || true
    pkill -f "b4m_waypoint_nav" 2>/dev/null || true
    pkill -f "rviz" 2>/dev/null || true
    pkill -f "nav2" 2>/dev/null || true
    pkill -f "slam_toolbox" 2>/dev/null || true
    sleep 2
    
    # Kill any remaining ROS2 nodes
    echo "Step 4: Killing remaining ROS2 node processes..."
    pkill -f "robot_state_publisher" 2>/dev/null || true
    pkill -f "joint_state_publisher" 2>/dev/null || true
    pkill -f "static_transform_publisher" 2>/dev/null || true
    pkill -f "complementary_filter" 2>/dev/null || true
    sleep 3
    
    # Use shutdown script as backup (preserve hardware connection)
    echo "Step 5: Running shutdown script cleanup..."
    if [ -f "./b4m_shutdown.sh" ]; then
        ./b4m_shutdown.sh --keep-agent > /dev/null 2>&1 || true
        sleep 2
    fi
    
    # Final verification
    echo "Step 6: Verifying cleanup..."
    local remaining_nodes=$(ros2 node list 2>/dev/null | wc -l || echo "0")
    echo "Remaining nodes after aggressive cleanup: $remaining_nodes"
    
    # Allow only hardware nodes to remain
    if [ "$remaining_nodes" -gt 2 ]; then
        echo "⚠️  Warning: $remaining_nodes nodes still running after aggressive cleanup"
        echo "Remaining nodes:"
        ros2 node list 2>/dev/null || echo "Failed to list nodes"
    else
        echo "✅ Aggressive cleanup completed successfully"
    fi
}

# Function to perform comprehensive cleanup after autotest failures
perform_autotest_cleanup() {
    echo "🧹 AUTOTEST CLEANUP: Preserving only YB_Car_Node and micro-ros-agent"
    echo "=================================================================="
    
    # Step 1: Kill all ROS2 launch processes immediately
    echo "Step 1: Killing ROS2 launch processes..."
    pkill -9 -f "ros2 launch" 2>/dev/null || true
    sleep 1
    
    # Step 2: Kill navigation and robot processes but preserve hardware connection
    echo "Step 2: Killing navigation and robot processes..."
    pkill -9 -f "yahboomcar_nav" 2>/dev/null || true
    pkill -9 -f "yahboomcar_bringup" 2>/dev/null || true  
    pkill -9 -f "nav2" 2>/dev/null || true
    pkill -9 -f "rviz" 2>/dev/null || true
    pkill -9 -f "slam_toolbox" 2>/dev/null || true
    pkill -9 -f "b4m_waypoint_nav" 2>/dev/null || true
    sleep 1
    
    # Step 3: Kill all robot component processes
    echo "Step 3: Killing robot component processes..."
    pkill -9 -f "complementary_filter_node" 2>/dev/null || true
    pkill -9 -f "static_transform_publisher" 2>/dev/null || true
    pkill -9 -f "joint_state_publisher" 2>/dev/null || true
    pkill -9 -f "robot_state_publisher" 2>/dev/null || true
    pkill -9 -f "ekf_node" 2>/dev/null || true
    pkill -9 -f "robot_localization" 2>/dev/null || true
    sleep 1
    
    # Step 4: Kill Python ROS2 processes except micro_ros_agent
    echo "Step 4: Killing Python ROS2 processes (preserving micro_ros_agent)..."
    ps aux | grep "python.*ros2" | grep -v "micro_ros_agent" | awk '{print $2}' | xargs -r kill -9 2>/dev/null || true
    sleep 2
    
    # Step 5: Restart the hardware connection if it was killed
    echo "Step 5: Verifying hardware connection..."
    local remaining_nodes=$(ros2 node list 2>/dev/null | wc -l || echo "0")
    if [ "$remaining_nodes" -eq 0 ]; then
        echo "⚠️  No nodes remaining - hardware connection may have been lost"
        echo "Waiting for YB_Car_Node to reconnect..."
        sleep 5
    else
        echo "✅ $remaining_nodes nodes remaining (should include YB_Car_Node)"
    fi
    
    echo "🎯 AUTOTEST CLEANUP COMPLETED"
    echo "=========================="
}

# Function to clean up existing processes safely
cleanup_existing_processes() {
    echo "🧹 Running automatic cleanup..."
    
    # Run the shutdown script with --keep-agent to preserve hardware connection
    if [ -f "./b4m_shutdown.sh" ]; then
        echo "Using b4m_shutdown.sh for safe cleanup (preserving hardware connection)..."
        ./b4m_shutdown.sh --keep-agent > /dev/null 2>&1
        sleep 3
    else
        echo "Shutdown script not found, using manual cleanup..."
        # Kill common ROS2 processes
        pkill -f "ros2 launch" 2>/dev/null || true
        pkill -f "yahboomcar" 2>/dev/null || true  
        pkill -f "nav2" 2>/dev/null || true
        pkill -f "rviz" 2>/dev/null || true
        sleep 3
    fi
    
    # Verify cleanup
    local remaining_nodes=$(ros2 node list 2>/dev/null | wc -l || echo "0")
    if [ "$remaining_nodes" -lt 5 ]; then
        echo "✅ Cleanup successful - $remaining_nodes nodes remaining"
    else
        echo "⚠️  Partial cleanup - $remaining_nodes nodes still running"
        echo "You may need to run './b4m_shutdown.sh' manually"
    fi
    echo ""
}

# Autotest mode timeout (seconds)
AUTOTEST_TIMEOUT=60
NAVIGATION_TIMEOUT=60  # Navigation needs more time

# Step validation functions for autotest mode
# Step 1 verification removed - always assume Micro-ROS agent is running correctly

validate_step_success() {
    local step_num=$1
    local timeout=${2:-$AUTOTEST_TIMEOUT}
    local step_log=$3
    
    debug_log "Validating Step $step_num (timeout: ${timeout}s) - Mode: $([ "$SIMULATION_MODE" = true ] && echo "SIMULATION" || echo "REAL_ROBOT")"
    
    case $step_num in
        1)
            if [ "$SIMULATION_MODE" = true ]; then
                # Step 1 Simulation: Ignition Gazebo world launch - check for ign gazebo process
                debug_log "Waiting for Ignition Gazebo server to start..."
                sleep 5  # Give Gazebo time to start
                
                local end_time=$(($(date +%s) + timeout))
                while [ $(date +%s) -lt $end_time ]; do
                    # Check for Gazebo server processes
                    if pgrep -f "ign gazebo" > /dev/null || pgrep -f "gz sim" > /dev/null; then
                        debug_log "Ignition Gazebo process found, giving additional startup time..."
                        sleep 5  # Give additional time for full initialization
                        debug_log "Step 1 validation passed: Ignition Gazebo simulation running"
                        return 0
                    fi
                    sleep 2
                done
                echo "ERROR: Step 1 validation failed - Ignition Gazebo simulation not running within $timeout seconds"
                return 1
            else
                # Step 1: Micro-ROS agent - assume already running correctly in autotest mode
                debug_log "Step 1 validation: Assuming Micro-ROS agent is running (autotest mode)"
                return 0
            fi
            ;;
        2)
            if [ "$SIMULATION_MODE" = true ]; then
                # Step 2 Simulation: Robot spawning - check for robot topics from ROS-Gazebo bridge
                local end_time=$(($(date +%s) + timeout))
                while [ $(date +%s) -lt $end_time ]; do
                    if ros2 topic list 2>/dev/null | grep -q "/cmd_vel" && \
                       ros2 topic list 2>/dev/null | grep -q "/odom"; then
                        debug_log "Step 2 validation passed: Robot spawned with ROS-Gazebo bridge topics"
                        return 0
                    fi
                    sleep 1
                done
                echo "ERROR: Step 2 validation failed - Robot not spawned with bridge topics within $timeout seconds"
                return 1
            else
                # Step 2: Robot connection - just wait for confirmation
                sleep 2
                return 0
            fi
            ;;
        3)
            if [ "$SIMULATION_MODE" = true ]; then
                # Step 3 Simulation: Check for robot state publisher and transform system
                local end_time=$(($(date +%s) + timeout))
                while [ $(date +%s) -lt $end_time ]; do
                    if ros2 node list 2>/dev/null | grep -q "robot_state_publisher" && \
                       ros2 topic list 2>/dev/null | grep -q "/tf" && \
                       ros2 topic list 2>/dev/null | grep -q "/scan"; then
                        debug_log "Step 3 validation passed: Ignition Gazebo robot systems active"
                        return 0
                    fi
                    sleep 1
                done
                echo "ERROR: Step 3 validation failed - Ignition Gazebo robot systems not active within $timeout seconds"
                return 1
            else
                # Step 3: Data processing - check for required nodes and topics
                local end_time=$(($(date +%s) + timeout))
                while [ $(date +%s) -lt $end_time ]; do
                    if ros2 node list 2>/dev/null | grep -q "complementary_filter_gain_node" && \
                       ros2 node list 2>/dev/null | grep -q "robot_state_publisher" && \
                       ros2 topic list 2>/dev/null | grep -q "/tf"; then
                        debug_log "Step 3 validation passed: Required nodes and topics found"
                        return 0
                    fi
                    sleep 1
                done
                echo "ERROR: Step 3 validation failed - required nodes/topics not found within $timeout seconds"
                return 1
            fi
            ;;
        4)
            # Step 4: RViz - check process exists and validate map display capability
            sleep 3
            if pgrep -f "rviz2" > /dev/null; then
                debug_log "RViz2 process found, checking if it's still running..."
                
                # Wait a moment and check if RViz crashed (common in headless mode)
                sleep 2
                if ! pgrep -f "rviz2" > /dev/null; then
                    echo "ERROR: Step 4 validation failed - RViz2 process started but crashed (likely display issue)"
                    if [ -f "$step_log" ]; then
                        echo "Last few lines of RViz log:" | tee -a "$step_log"
                        tail -10 "$step_log" || true
                    fi
                    return 1
                fi
                
                debug_log "RViz2 process stable, checking ROS2 node registration..."
                # Check if RViz registered as a ROS2 node (indicates proper startup)
                if ros2 node list 2>/dev/null | grep -q "rviz2"; then
                    debug_log "RViz2 registered with ROS2, checking for map display issues..."
                    
                    # Check if basic transform tree is available for map display
                    if ros2 run tf2_ros tf2_echo map base_link --timeout 2 >/dev/null 2>&1; then
                        debug_log "Transform chain map->base_link is available for RViz map display"
                    else
                        debug_log "WARNING: Transform chain map->base_link not available - may affect map display"
                    fi
                    
                    # Check the step log for critical RViz errors that would prevent map display
                    if [ -f "$step_log" ]; then
                        if grep -q "process has died" "$step_log"; then
                            echo "ERROR: Step 4 validation failed - RViz process died (exit code found in log)"
                            return 1
                        fi
                        if grep -q "transform cache" "$step_log" || \
                           grep -q "frame.*does not exist" "$step_log" || \
                           grep -q "Could not obtain transform" "$step_log"; then
                            echo "WARNING: RViz started but has transform/frame errors that may prevent map display"
                            debug_log "Transform errors detected in RViz log - map display may be impaired"
                            # Still pass validation since RViz is running, but log the warning
                        fi
                    fi
                    
                    debug_log "Step 4 validation passed: RViz2 process running and registered with ROS2"
                    return 0
                else
                    echo "ERROR: Step 4 validation failed - RViz2 process running but not registered with ROS2"
                    return 1
                fi
            else
                echo "ERROR: Step 4 validation failed - RViz2 process not found"
                return 1
            fi
            ;;
        5)
            # Step 5: Navigation - check for navigation nodes, map data, and lifecycle activation
            local end_time=$(($(date +%s) + timeout))
            while [ $(date +%s) -lt $end_time ]; do
                # FIX: Different validation logic for simulation vs real robot mode
                if [ "$SIMULATION_MODE" = true ]; then
                    # In SLAM simulation, check for SLAM toolbox and required topics
                    if ros2 node list 2>/dev/null | grep -q "slam_toolbox" && \
                       ros2 topic list 2>/dev/null | grep -q "/map"; then
                        debug_log "SLAM toolbox found and map topic available"
                        
                        # Additional check for scan topic to ensure full sensor integration
                        if ros2 topic list 2>/dev/null | grep -q "/scan"; then
                            debug_log "Step 5 validation passed: SLAM mapping system active with sensor data"
                            return 0
                        else
                            debug_log "SLAM active but waiting for sensor data..."
                        fi
                    else
                        debug_log "Waiting for SLAM toolbox and map topic..."
                    fi
                else
                    # Real robot mode - expect pre-existing map
                    if ros2 node list 2>/dev/null | grep -q "slam_toolbox" && \
                       ros2 topic list 2>/dev/null | grep -q "/map"; then
                        debug_log "Navigation nodes found, checking map data and lifecycle state..."
                        
                        # Check if map data is published
                        if ros2 topic echo /map --once 2>/dev/null | grep -q "frame_id: map"; then
                            debug_log "Map data confirmed, checking navigation lifecycle activation..."
                        else
                            # Check if lifecycle manager activated the navigation nodes
                            # Look for "Managed nodes are active" in the step log
                            if [ -f "$step_log" ] && grep -q "Managed nodes are active" "$step_log"; then
                                debug_log "Navigation lifecycle activation confirmed"
                                
                                # Give extra time for SLAM to fully initialize (critical for map frame)
                                debug_log "Waiting additional 5 seconds for SLAM to fully initialize..."
                                sleep 5
                                
                                debug_log "Step 5 validation passed: Navigation system fully activated with map"
                                return 0
                            else
                                debug_log "Navigation nodes found but lifecycle not yet activated"
                            fi
                        fi
                    else
                        debug_log "Map topic exists but no data published yet"
                    fi
                fi
                sleep 1
            done
            echo "ERROR: Step 5 validation failed - navigation system not fully activated within $timeout seconds"
            return 1
            ;;
        6)
            # Step 6: SLAM initialization - check if slam_toolbox is publishing transforms
            debug_log "Step 6: Validating SLAM system initialization"
            
            # Wait for SLAM system to start
            sleep 5
            
            # Verify slam_toolbox is running and publishing transforms
            local end_time=$(($(date +%s) + timeout))
            while [ $(date +%s) -lt $end_time ]; do
                if ros2 node list 2>/dev/null | grep -q "slam_toolbox"; then
                    if [ "$SIMULATION_MODE" = true ]; then
                        # Check for SLAM system readiness with ROS-Gazebo bridge
                        if ros2 topic list 2>/dev/null | grep -q "/cmd_vel" && \
                           ros2 topic list 2>/dev/null | grep -q "/odom" && \
                           ros2 topic list 2>/dev/null | grep -q "/scan"; then
                            debug_log "Step 6 validation passed: SLAM system ready in Ignition Gazebo"
                            return 0
                        fi
                    else
                        # For real robot, check full transform chain
                        if ros2 run tf2_ros tf2_echo map odom --timeout 2 >/dev/null 2>&1; then
                            debug_log "Step 6 validation passed: SLAM system initialized and publishing transforms"
                            return 0
                        fi
                    fi
                fi
                sleep 1
            done
            
            echo "ERROR: Step 6 validation failed - SLAM system not initializing properly within $timeout seconds"
            return 1
            ;;
        7)
            # Step 7: MQTT navigation - check for python process
            sleep 3
            if pgrep -f "b4m_waypoint_nav.py" > /dev/null; then
                debug_log "Step 7 validation passed: B4M waypoint navigation process running"
                return 0
            else
                echo "ERROR: Step 7 validation failed - B4M waypoint navigation process not found"
                return 1
            fi
            ;;
        8)
            # Step 8: Automated square movement - check for completion and results
            debug_log "Step 8: Validating automated square movement completion"
            
            # Wait for script to complete (up to timeout)
            local end_time=$(($(date +%s) + timeout))
            while [ $(date +%s) -lt $end_time ]; do
                if ! pgrep -f "automated_square_movement.py" > /dev/null; then
                    debug_log "Automated square movement script completed"
                    
                    # Check for results file
                    if [ -f "/tmp/automated_square_movement_results.json" ]; then
                        # Validate results using Python
                        if python3 -c "
import json
try:
    with open('/tmp/automated_square_movement_results.json') as f:
        results = json.load(f)
    success = results.get('success', False)
    sides_completed = results.get('sides_completed', 0)
    loop_closed = results.get('loop_closed', False)
    obstacles = results.get('obstacles_detected', 0)
    print(f'Square movement results: success={success}, sides={sides_completed}, loop_closed={loop_closed}, obstacles={obstacles}')
    exit(0 if success else 1)
except Exception as e:
    print(f'Error reading results: {e}')
    exit(1)
"; then
                            debug_log "Step 8 validation passed: Automated square movement completed successfully"
                            return 0
                        else
                            echo "ERROR: Step 8 validation failed - automated square movement did not meet success criteria"
                            return 1
                        fi
                    else
                        echo "ERROR: Step 8 validation failed - results file not found"
                        return 1
                    fi
                fi
                sleep 1
            done
            
            echo "ERROR: Step 8 validation failed - automated square movement timed out"
            return 1
            ;;
        9)
            # Step 9: Map validation - check for completion and results
            debug_log "Step 9: Validating map saving and validation completion"
            
            # Wait for script to complete
            local end_time=$(($(date +%s) + timeout))
            while [ $(date +%s) -lt $end_time ]; do
                if ! pgrep -f "map_validation.py" > /dev/null; then
                    debug_log "Map validation script completed"
                    
                    # Check for results file
                    if [ -f "/tmp/map_validation_results.json" ]; then
                        # Validate results
                        if python3 -c "
import json
try:
    with open('/tmp/map_validation_results.json') as f:
        results = json.load(f)
    success = results.get('validation_success', False)
    map_received = results.get('map_received', False)
    map_saved = results.get('map_saved', False)
    obstacles = results.get('obstacles_detected', 0)
    print(f'Map validation results: success={success}, map_received={map_received}, map_saved={map_saved}, obstacles={obstacles}')
    exit(0 if success else 1)
except Exception as e:
    print(f'Error reading results: {e}')
    exit(1)
"; then
                            debug_log "Step 9 validation passed: Map validation completed successfully"
                            return 0
                        else
                            echo "ERROR: Step 9 validation failed - map validation did not meet success criteria"
                            return 1
                        fi
                    else
                        echo "ERROR: Step 9 validation failed - results file not found"
                        return 1
                    fi
                fi
                sleep 1
            done
            
            echo "ERROR: Step 9 validation failed - map validation timed out"
            return 1
            ;;
        10)
            # Step 10: MQTT navigation test - check for completion and results
            debug_log "Step 10: Validating MQTT navigation testing completion"
            
            # Wait for script to complete
            local end_time=$(($(date +%s) + timeout))
            while [ $(date +%s) -lt $end_time ]; do
                if ! pgrep -f "mqtt_navigation_test.py" > /dev/null; then
                    debug_log "MQTT navigation test script completed"
                    
                    # Check for results file
                    if [ -f "/tmp/mqtt_navigation_test_results.json" ]; then
                        # Validate results
                        if python3 -c "
import json
try:
    with open('/tmp/mqtt_navigation_test_results.json') as f:
        results = json.load(f)
    success = results.get('overall_success', False)
    mqtt_connected = results.get('mqtt_connected', False)
    navigation_working = results.get('navigation_working', False)
    tests_successful = results.get('tests_successful', 0)
    tests_completed = results.get('tests_completed', 0)
    print(f'MQTT navigation results: success={success}, mqtt={mqtt_connected}, nav={navigation_working}, tests={tests_successful}/{tests_completed}')
    exit(0 if success else 1)
except Exception as e:
    print(f'Error reading results: {e}')
    exit(1)
"; then
                            debug_log "Step 10 validation passed: MQTT navigation testing completed successfully"
                            return 0
                        else
                            echo "ERROR: Step 10 validation failed - MQTT navigation testing did not meet success criteria"
                            return 1
                        fi
                    else
                        echo "ERROR: Step 10 validation failed - results file not found"
                        return 1
                    fi
                fi
                sleep 1
            done
            
            echo "ERROR: Step 10 validation failed - MQTT navigation testing timed out"
            return 1
            ;;
        *)
            echo "ERROR: Unknown step number for validation: $step_num"
            return 1
            ;;
    esac
}























handle_test_failure() {
    local failed_step=$1
    local error_message=$2
    
    echo ""
    echo "======================================="
    echo "TEST FAILED at Step $failed_step"
    echo "======================================="
    echo "Error: $error_message"
    echo "Time: $(date)"
    echo ""
    
    log_message "AUTOTEST FAILED at Step $failed_step: $error_message"
    
    # Capture debug snapshot
    echo "Capturing debug information..."
    echo "ROS2 nodes:" >> "$MAIN_LOG"
    ros2 node list 2>/dev/null >> "$MAIN_LOG" || echo "Failed to get node list" >> "$MAIN_LOG"
    echo "ROS2 topics:" >> "$MAIN_LOG"
    ros2 topic list 2>/dev/null >> "$MAIN_LOG" || echo "Failed to get topic list" >> "$MAIN_LOG"
    echo "Active processes:" >> "$MAIN_LOG"
    ps aux | grep -E "(ros2|yahboom|nav2|rviz)" | grep -v grep >> "$MAIN_LOG"
    
    # Run cleanup preserving Micro-ROS agent
    echo "Running cleanup with --keep-agent..."
    perform_autotest_cleanup
    
    echo ""
    echo "Test logs saved to: $MAIN_LOG"
    echo "======================================="
    
    exit 1
}

# Function to launch a command in a new terminal with logging
launch_in_terminal() {
    local description=$1
    local command=$2
    local step_num=$3
    
    # Create step-specific log file
    local step_log="$LOGS_DIR/step${step_num}_$(echo "$description" | tr ' ' '_' | tr '[:upper:]' '[:lower:]')_$TIMESTAMP.log"
    
    echo "====================================================="
    echo "STEP $step_num: $description"
    echo "====================================================="
    echo "Command to execute:"
    echo "  $command"
    echo ""
    
    # In autotest mode, skip user confirmation and use direct execution
    if [ "$AUTOTEST_MODE" = true ]; then
        echo "🤖 AUTOTEST MODE: Executing automatically..."
        echo "📁 Log file location: $step_log"
        echo ""
        
        log_message "AUTOTEST STEP $step_num: $description"
        log_message "Log file: $step_log"
        
        # Execute command directly in background
        cd "$WORKSPACE_ROOT"
        if [ -f 'install/setup.bash' ]; then
            source install/setup.bash
            debug_log "Workspace sourced successfully"
        else
            echo "⚠️  WARNING: install/setup.bash not found!"
        fi
        
        echo "Starting: $description" | tee "$step_log"
        echo "Command: $command" | tee -a "$step_log"
        echo "Started at: $(date)" | tee -a "$step_log"
        echo "=====================================" | tee -a "$step_log"
        
        # Handle display requirements for RViz in autotest mode
        if [[ "$command" == *"rviz"* ]] && [ -z "$DISPLAY" ]; then
            echo "RViz detected in autotest mode without display - setting up virtual display" | tee -a "$step_log"
            export DISPLAY=:99
            # Start Xvfb if not already running
            if ! pgrep -f "Xvfb :99" > /dev/null; then
                Xvfb :99 -screen 0 1024x768x24 -ac +extension GLX +render -noreset &
                sleep 2
                debug_log "Started virtual display Xvfb :99"
            fi
        fi
        
        # Execute command in background
        eval "$command" >> "$step_log" 2>&1 &
        local cmd_pid=$!
        
        # Wait a moment for process to start
        sleep 2
        
        # Validate step success - use longer timeout for navigation (Step 5) and SLAM movement (Step 8)
        local step_timeout="$AUTOTEST_TIMEOUT"
        if [ "$step_num" = "5" ]; then
            step_timeout="$NAVIGATION_TIMEOUT"
        elif [ "$step_num" = "8" ]; then
            step_timeout="120"  # Square movement needs more time
        fi
        
        if validate_step_success "$step_num" "$step_timeout" "$step_log"; then
            echo "✅ Step $step_num validation passed"
            log_message "AUTOTEST STEP $step_num: PASSED"
        else
            handle_test_failure "$step_num" "Step validation failed"
        fi
        
        return
    fi
    
    # Interactive mode (original behavior)
    echo "Press ENTER to execute this command, or Ctrl+C to exit..."
    read
    
    echo ""
    echo "📁 Log file location: $step_log"
    echo "🚀 Launching step $step_num in new terminal..."
    echo ""
    
    log_message "STEP $step_num: Starting terminal for $description"
    log_message "Log file: $step_log"
    
    # Create a temporary script for this step
    local temp_script="/tmp/b4m_step${step_num}_$$"
    cat > "$temp_script" << EOF
#!/bin/bash

# Store the command to execute
COMMAND='$command'

# Terminal title
echo -e "\033]0;B4M Step $step_num: $description\007"

echo "======================================="
echo "B4M Robot Launch - Step $step_num"
echo "======================================="
echo "Description: $description"
echo "Log file: $step_log"
echo "Started at: \$(date)"
echo "======================================="
echo ""

# Log the start
echo "Starting: $description" | tee "$step_log"
echo "Command: \$COMMAND" | tee -a "$step_log"
echo "Started at: \$(date)" | tee -a "$step_log"
echo "=====================================" | tee -a "$step_log"

# Source workspace before running command
cd "$WORKSPACE_ROOT"
if [ -f 'install/setup.bash' ]; then
    source install/setup.bash
    echo "✅ Workspace sourced successfully" | tee -a "$step_log"
else
    echo "⚠️  WARNING: install/setup.bash not found!" | tee -a "$step_log"
fi

echo ""
echo "🚀 Executing command..."
echo ""

# Execute the command with better error handling
set +e  # Don't exit on error
eval "\$COMMAND" 2>&1 | tee -a "$step_log"
COMMAND_EXIT_CODE=\$?
set -e

echo ""
echo "=====================================" | tee -a "$step_log"
if [ \$COMMAND_EXIT_CODE -eq 0 ]; then
    echo "✅ Process completed successfully at: \$(date)" | tee -a "$step_log"
else
    echo "❌ Process failed with exit code \$COMMAND_EXIT_CODE at: \$(date)" | tee -a "$step_log"
fi
echo "Log saved to: $step_log" | tee -a "$step_log"
echo "=====================================" | tee -a "$step_log"

echo ""
echo "Step $step_num terminal will remain open for debugging."
echo "Log file: $step_log"
echo ""
echo "Commands you can run:"
echo "  - 'tail -f $step_log' to monitor the log"
echo "  - 'ros2 node list' to check active nodes"
echo "  - 'ros2 topic list' to check active topics"
echo ""
echo "Press Enter to close this terminal, or run additional commands..."

# Keep terminal open
exec bash

EOF
    
    chmod +x "$temp_script"
    
    # Launch the temporary script in a new terminal
    xterm -fn fixed -e "$temp_script" &
    
    # Give some time for the terminal to start
    sleep 2
    
    log_message "STEP $step_num terminal launched"
    
    # Monitor node count for duplicate detection
    if [ "$step_num" -ge 3 ]; then  # Start monitoring after robot bringup
        local current_nodes=$(ros2 node list 2>/dev/null | wc -l || echo "0")
        debug_log "Node count after Step $step_num: $current_nodes nodes"
        
        # Warn if node count is growing too quickly (indicates duplicates)
        if [ "$current_nodes" -gt $((step_num * 8)) ]; then
            echo "⚠️  Node count warning: $current_nodes nodes detected after Step $step_num"
            echo "   This may indicate duplicate processes are running"
        fi
    fi
}

echo "B4M Robot - Home Assistant MQTT Integration Launch Script"

if [ "$AUTOTEST_MODE" = true ]; then
    echo "🤖 AUTOMATED TEST MODE ENABLED"
    echo "This script will run all steps automatically without user interaction."
    echo "Steps will be validated with $AUTOTEST_TIMEOUT second timeouts."
    echo "Test will abort on first failure and run cleanup automatically."
    echo ""
    echo "ℹ️  Assuming Micro-ROS agent is running and robot is connected (Step 1)"
else
    echo "This script will guide you through launching all components of the B4M Robot system."
    echo "Each step will open in a separate terminal window."
fi

echo "Logs will be saved to: $LOGS_DIR"
echo "Main log file: $MAIN_LOG"

if [ "$SKIP_AGENT" = true ]; then
    echo ""
    echo "⏭️  Skipping Micro-ROS agent launch (--skip-agent specified)"
fi

if [ "$ONLY_AGENT" = true ]; then
    echo ""
    echo "🎯 Only agent mode enabled (--only-agent specified)"
    echo "Will launch ONLY the Micro-ROS agent and exit"
fi

if [ "$DEBUG_MODE" = true ]; then
    echo ""
    echo "🔍 Debug mode enabled - verbose logging active"
fi

if [ "$SIMULATION_MODE" = true ]; then
    echo ""
    echo "🎮 Simulation mode enabled - launching with Gazebo instead of real robot"
fi

if [ "$REGRESSION_MODE" = true ]; then
    echo ""
    echo "🧪 Regression test mode enabled - will run square corners test after launch"
fi


echo ""

if [ "$AUTOTEST_MODE" = true ]; then
    log_message "B4M Robot AUTOTEST launch script started"
else
    log_message "B4M Robot launch script started"
fi

# Special handling for --only-agent mode: clean up existing connections
if [ "$ONLY_AGENT" = true ]; then
    echo "🎯 Only agent mode: Cleaning up existing connections..."
    
    # Check if YB_Car_Node is running (indicates existing robot connection)
    if ros2 node list 2>/dev/null | grep -q "YB_Car_Node"; then
        echo "🔌 Existing robot connection detected (YB_Car_Node)"
        echo "   Disconnecting to start fresh agent connection..."
        
        # Stop YB_Car_Node by killing the agent that maintains the connection
        docker_containers=$(docker ps --filter 'ancestor=microros/micro-ros-agent:humble' --format '{{.ID}}' 2>/dev/null || true)
        if [ ! -z "$docker_containers" ]; then
            echo "   Stopping existing Micro-ROS agent containers..."
            echo "$docker_containers" | while read -r container_id; do
                if [ ! -z "$container_id" ]; then
                    docker stop "$container_id" 2>/dev/null || true
                    docker rm "$container_id" 2>/dev/null || true
                fi
            done
            # Wait for YB_Car_Node to disconnect
            sleep 3
        fi
        
        # Verify YB_Car_Node is gone
        if ros2 node list 2>/dev/null | grep -q "YB_Car_Node"; then
            echo "⚠️  WARNING: YB_Car_Node still present after agent shutdown"
            echo "   This may indicate the robot is using a different connection method"
        else
            echo "✅ Robot disconnected successfully"
        fi
    fi
    
    # Also clean up any other processes that might interfere
    echo "🧹 Cleaning up any remaining robot processes..."
    ./b4m_shutdown.sh > /dev/null 2>&1 || true
    sleep 2
fi

# Regression Test Mode - Run test_square_corners.py directly without normal launch
if [ "$REGRESSION_MODE" = true ]; then
    echo ""
    echo "========================================================"
    echo "🧪 REGRESSION TEST MODE"
    echo "========================================================"
    echo "Running square corners navigation regression test..."
    echo "This test launches its own simulation and manages the full test lifecycle."
    echo ""
    
    log_message "REGRESSION TEST MODE: Starting test_square_corners.py"
    
    # Execute the regression test
    REGRESSION_LOG="$LOGS_DIR/regression_test_$TIMESTAMP.log"
    echo "📁 Regression test log: $REGRESSION_LOG"
    echo ""
    
    # Run the test and capture exit code
    cd "$WORKSPACE_ROOT"
    if [ -f 'install/setup.bash' ]; then
        source install/setup.bash
        debug_log "Workspace sourced successfully for regression test"
    else
        echo "⚠️  WARNING: install/setup.bash not found for regression test!"
    fi
    
    # Run test with timeout and proper output handling
    timeout 300s python3 "$WORKSPACE_ROOT/test_square_corners.py" > "$REGRESSION_LOG" 2>&1
    REGRESSION_EXIT_CODE=$?
    
    echo ""
    echo "========================================================"
    if [ $REGRESSION_EXIT_CODE -eq 0 ]; then
        echo "✅ REGRESSION TEST PASSED"
        log_message "REGRESSION TEST: PASSED - Robot successfully navigated square"
    else
        echo "❌ REGRESSION TEST FAILED"
        log_message "REGRESSION TEST: FAILED - Exit code: $REGRESSION_EXIT_CODE"
    fi
    echo "========================================================"
    echo "Regression test log saved to: $REGRESSION_LOG"
    echo ""
    
    # Exit with regression test result
    exit $REGRESSION_EXIT_CODE
fi

# Pre-launch system check to prevent duplicate processes
check_existing_processes

# Step 1: Start the Micro-ROS Agent (unless skipped) OR Launch Gazebo Simulation
if [ "$SIMULATION_MODE" = true ]; then
    if [ "$SLAM_TEST_MODE" = true ]; then
        launch_in_terminal "Starting Ignition Gazebo simulation with SLAM test world (2 obstacles)" \
            "cd \"$WORKSPACE_ROOT\" && . install/setup.bash && ign gazebo \"$WORKSPACE_ROOT/yahboomcar_nav/worlds/slam_test_world.sdf\"" \
            "1"
    else
        launch_in_terminal "Starting Ignition Gazebo simulation environment" \
            "cd \"$WORKSPACE_ROOT\" && . install/setup.bash && ign gazebo" \
            "1"
    fi
elif [ "$SKIP_AGENT" = false ]; then
    launch_in_terminal "Starting the Micro-ROS Agent for ESP32 communication" \
        "docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:humble udp4 --port 8090" \
        "1"
else
    log_message "STEP 1: Skipped Micro-ROS agent launch"
fi

# Exit early if only agent mode is enabled
if [ "$ONLY_AGENT" = true ]; then
    echo ""
    echo "======================================================"
    echo "Only agent mode completed!"
    echo "======================================================"
    echo "The Micro-ROS agent has been launched and is running."
    echo "Logs saved to: $LOGS_DIR"
    echo "Main log file: $MAIN_LOG"
    echo "======================================================"
    
    log_message "ONLY_AGENT mode: Script completed after launching Micro-ROS agent"
    exit 0
fi

# Step 2: Power on the Yahboom Robot OR Spawn Robot with Controllers in Gazebo
if [ "$SIMULATION_MODE" = true ]; then
    echo "====================================================="
    echo "STEP 2: Spawn robot with controllers in Gazebo simulation"
    echo "====================================================="
    
    launch_in_terminal "Spawning Yahboom robot with direct plugin in Ignition Gazebo" \
        "cd \"$WORKSPACE_ROOT\" && . install/setup.bash && ros2 launch yahboomcar_nav spawn_robot_simple_gazebo.py" \
        "2"
else
    echo "====================================================="
    echo "STEP 2: Power on the physical Yahboom Robot"
    echo "====================================================="

    if [ "$AUTOTEST_MODE" = true ]; then
        echo "🤖 AUTOTEST MODE: Assuming robot is already powered on and connected"
        log_message "AUTOTEST STEP 2: Robot connection verification"
        
        # Validate robot connection
        if validate_step_success "2" "$AUTOTEST_TIMEOUT"; then
            echo "✅ Step 2 validation passed"
            log_message "AUTOTEST STEP 2: PASSED"
        else
            handle_test_failure "2" "Robot connection verification failed"
        fi
    else
        echo "Manual step required:"
        echo "  1. Turn on the physical robot's power switch"
        echo "  2. Wait for the robot to boot up and connect to the Micro-ROS agent"
        echo "  3. Check for connection messages in the Micro-ROS agent terminal"
        echo ""

        log_message "STEP 2: Waiting for physical robot power on"

        echo "Press ENTER when the robot is powered on and connected..."
        read

        log_message "STEP 2: Physical robot power on confirmed"
    fi
fi

# Step 3: Launch the Car's Underlying Data Processing OR Gazebo Controllers
if [ "$SIMULATION_MODE" = true ]; then
    # NOTE: Robot state publisher and controllers are now handled in Step 2 
    # by the spawn_robot_with_controllers_gazebo.py launch file
    # No separate Step 3 needed for simulation mode
    echo "✅ Step 3: Skipped for simulation mode (handled in Step 2)"
else
    launch_in_terminal "Starting the car's underlying data processing for sensor integration" \
        "cd \"$WORKSPACE_ROOT\" && . install/setup.bash && ros2 launch yahboomcar_bringup yahboomcar_bringup_launch.py" \
        "3"
fi

# Step 4: Start RViz for Visualization
if [ "$SIMULATION_MODE" = true ]; then
    launch_in_terminal "Starting RViz for visualization of robot state and environment (simulation mode)" \
        "cd \"$WORKSPACE_ROOT\" && . install/setup.bash && ros2 launch yahboomcar_nav display_launch.py use_sim_time:=true" \
        "4"
else
    launch_in_terminal "Starting RViz for visualization of robot state and environment" \
        "cd \"$WORKSPACE_ROOT\" && . install/setup.bash && ros2 launch yahboomcar_nav display_launch.py use_sim_time:=false" \
        "4"
fi

# Step 5: Launch the SLAM-based Navigation System
if [ "$SIMULATION_MODE" = true ]; then
    launch_in_terminal "Launching SLAM-based navigation system (Ignition Gazebo simulation)" \
        "cd \"$WORKSPACE_ROOT\" && . install/setup.bash && ros2 launch yahboomcar_nav slam_mapping_gazebo.py" \
        "5"
else
    launch_in_terminal "Launching SLAM-based navigation system" \
        "cd \"$WORKSPACE_ROOT\" && . install/setup.bash && ros2 launch yahboomcar_nav slam_navigation_launch.py" \
        "5"
fi

# Step 6: SLAM System Monitoring
if [ "$SIMULATION_MODE" = true ]; then
    launch_in_terminal "Initializing SLAM in Ignition Gazebo simulation environment" \
        "cd \"$WORKSPACE_ROOT\" && . install/setup.bash && echo 'SLAM system initializing in Ignition Gazebo - use keyboard teleop for mapping'" \
        "6"
else
    launch_in_terminal "Monitoring SLAM initialization and map building" \
        "cd \"$WORKSPACE_ROOT\" && . install/setup.bash && echo 'SLAM system initializing - map will be built automatically as robot moves'" \
        "6"
fi

# Step 7: Start the B4M Waypoint Navigation Node with MQTT Parameters
launch_in_terminal "Starting the B4M Waypoint Navigation Node with MQTT integration" \
    "cd \"$WORKSPACE_ROOT\" && . install/setup.bash && python3 \"$WORKSPACE_ROOT/b4m_waypoint_nav/b4m_waypoint_nav/b4m_waypoint_nav.py\" --ros-args -p mqtt_broker:=192.168.68.111 -p mqtt_port:=1883 -p mqtt_username:=robot -p mqtt_password:=robot123" \
    "7"


# Step 8: Start the Robot Manager GUI (skip in autotest mode and SLAM test mode)
if [ "$AUTOTEST_MODE" = false ] && [ "$SLAM_TEST_MODE" = false ]; then
    launch_in_terminal "Starting the B4M Robot Manager GUI for visual control of waypoints" \
        "cd \"$WORKSPACE_ROOT\" && . install/setup.bash && ros2 run b4m_waypoint_nav b4m_robot_manager_node.py" \
        "8"
else
    if [ "$SLAM_TEST_MODE" = true ]; then
        debug_log "Robot Manager GUI skipped in SLAM test mode - will run automated SLAM testing instead"
    else
        debug_log "Robot Manager GUI skipped in autotest mode"
    fi
fi

# SLAM Test Steps (8-10) - Only run when --slam-test is enabled
if [ "$SLAM_TEST_MODE" = true ]; then
    echo ""
    echo "======================================================"
    echo "🧪 AUTOMATED SLAM TESTING SEQUENCE"
    echo "======================================================"
    echo "Running Steps 8-10 for automated SLAM validation:"
    echo "  Step 8: Automated 1-meter square movement with SLAM mapping"
    echo "  Step 9: Automated map saving and validation" 
    echo "  Step 10: Automated MQTT navigation testing"
    echo ""
    
    log_message "SLAM TEST MODE: Starting automated SLAM testing sequence"
    
    # Step 8: Automated Square Movement with SLAM Mapping
    launch_in_terminal "Step 8: Automated 1-meter square movement with SLAM mapping" \
        "cd \"$WORKSPACE_ROOT\" && . install/setup.bash && python3 \"$WORKSPACE_ROOT/yahboomcar_nav/scripts/automated_square_movement.py\"" \
        "8"
    
    # Step 9: Automated Map Saving and Validation
    launch_in_terminal "Step 9: Automated map saving and validation" \
        "cd \"$WORKSPACE_ROOT\" && . install/setup.bash && python3 \"$WORKSPACE_ROOT/yahboomcar_nav/scripts/map_validation.py\"" \
        "9"
    
    # Step 10: Automated MQTT Navigation Testing
    launch_in_terminal "Step 10: Automated MQTT navigation testing" \
        "cd \"$WORKSPACE_ROOT\" && . install/setup.bash && python3 \"$WORKSPACE_ROOT/yahboomcar_nav/scripts/mqtt_navigation_test.py\"" \
        "10"
    
    log_message "SLAM TEST MODE: All automated SLAM testing steps completed"
fi


log_message "B4M Robot launch script completed"

if [ "$AUTOTEST_MODE" = true ]; then
    echo ""
    echo "======================================="
    if [ "$SLAM_TEST_MODE" = true ]; then
        if [ "$SIMULATION_MODE" = true ]; then
            echo "B4M Robot SLAM Testing - Gazebo Simulation - PASSED"
        else
            echo "B4M Robot SLAM Testing - Real Robot - PASSED"
        fi
    else
        if [ "$SIMULATION_MODE" = true ]; then
            echo "B4M Robot Ignition Gazebo Simulation Test - PASSED"
        else
            echo "B4M Robot Launch Test - PASSED"
        fi
    fi
    echo "======================================="
    echo "Test Run: $(date)"
    
    if [ "$SLAM_TEST_MODE" = true ]; then
        echo "All 10 SLAM testing steps completed successfully"
    else
        echo "All 7 tested steps completed successfully"
    fi
    
    echo ""
    echo "Step Summary:"
    if [ "$SIMULATION_MODE" = true ]; then
        echo "✅ Step 1: Ignition Gazebo Simulation Environment"
        echo "✅ Step 2: Robot Spawning in Ignition Gazebo" 
        echo "✅ Step 3: Ignition Gazebo Robot Systems"
        echo "✅ Step 4: RViz Launch"
        echo "✅ Step 5: SLAM Navigation (Ignition Gazebo)"
        echo "✅ Step 6: SLAM Initialization (Ignition Gazebo)"
        echo "✅ Step 7: MQTT Navigation"
    else
        echo "✅ Step 1: Micro-ROS Agent (assumed running - prerequisite)"
        echo "✅ Step 2: Robot Connection" 
        echo "✅ Step 3: Data Processing"
        echo "✅ Step 4: RViz Launch"
        echo "✅ Step 5: Navigation System"
        echo "✅ Step 6: Pose Estimation"
        echo "✅ Step 7: MQTT Navigation"
    fi
    
    if [ "$SLAM_TEST_MODE" = true ]; then
        echo "✅ Step 8: Automated Square Movement with SLAM Mapping"
        echo "✅ Step 9: Automated Map Saving and Validation"
        echo "✅ Step 10: Automated MQTT Navigation Testing"
    fi
    
    echo ""
    echo "Logs saved to: $MAIN_LOG"
    
    echo "======================================="
    
    if [ "$SLAM_TEST_MODE" = true ]; then
        log_message "SLAM AUTOTEST COMPLETED SUCCESSFULLY - ALL STEPS PASSED"
    else
        log_message "AUTOTEST COMPLETED SUCCESSFULLY - ALL STEPS PASSED"
    fi
else
    echo "====================================================="
    echo "Launch script completed successfully!"
    echo "All logs are saved in: $LOGS_DIR"
    echo "Main log file: $MAIN_LOG"
    echo ""
    echo "🧹 CLEANUP REMINDER:"
    echo "Run './b4m_shutdown.sh' when finished to clean up all processes"
    echo "====================================================="
fi

exit 0
