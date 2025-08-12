#!/bin/bash

# B4M Robot - Shutdown Script
# This script properly shuts down all ROS2 nodes and processes
# Usage: ./b4m_shutdown.sh [--keep-agent]
#   --keep-agent: Keep Micro-ROS agent running (default: stop agent)

# Parse command line arguments
KEEP_AGENT=false
for arg in "$@"; do
    case $arg in
        --keep-agent)
            KEEP_AGENT=true
            shift
            ;;
        *)
            echo "Unknown option: $arg"
            echo "Usage: $0 [--keep-agent]"
            exit 1
            ;;
    esac
done

# Get the workspace root directory
WORKSPACE_ROOT=$(cd "$(dirname "$0")" && pwd)

# Create logs directory if it doesn't exist
LOGS_DIR="$WORKSPACE_ROOT/logs"
mkdir -p "$LOGS_DIR"

# Generate timestamp for log files
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
SHUTDOWN_LOG="$LOGS_DIR/b4m_shutdown_$TIMESTAMP.log"

# Function to log shutdown messages
shutdown_log() {
    local message=$1
    echo "$(date '+%Y-%m-%d %H:%M:%S') [SHUTDOWN] $message" | tee -a "$SHUTDOWN_LOG"
}

echo "B4M Robot - Shutdown Script"
echo "This script will safely shut down all ROS2 nodes and processes."
echo "Shutdown log: $SHUTDOWN_LOG"
echo ""

shutdown_log "B4M Robot shutdown script started"

# Step 1: Stop all ROS2 nodes gracefully (except YB_Car_Node if --keep-agent)
if [ "$KEEP_AGENT" = true ]; then
    shutdown_log "Step 1: Stopping all ROS2 nodes gracefully (preserving YB_Car_Node)"
else
    shutdown_log "Step 1: Stopping all ROS2 nodes gracefully"
fi

# First, get initial node list
initial_nodes=$(ros2 node list 2>/dev/null || true)
if [ ! -z "$initial_nodes" ]; then
    shutdown_log "Initial ROS2 nodes found:"
    echo "$initial_nodes" | while read -r node; do
        if [ ! -z "$node" ]; then
            shutdown_log "  - $node"
        fi
    done
else
    shutdown_log "No ROS2 nodes currently running"
fi

# Kill nodes by name
echo "$initial_nodes" | while read -r node; do
    if [ ! -z "$node" ]; then
        if [ "$KEEP_AGENT" = true ] && [[ "$node" == *"YB_Car_Node"* ]]; then
            shutdown_log "Preserving YB_Car_Node: $node"
        else
            shutdown_log "Stopping ROS2 node: $node"
            # Extract node name without namespace for killing
            node_name=$(basename "$node")
            # Try multiple methods to kill the node
            pkill -f "$node_name" 2>/dev/null
            # Also try killing by the full node path
            pkill -f "$node" 2>/dev/null
        fi
    fi
done

sleep 3

# Step 2: Force kill remaining ROS2 processes if needed (but preserve YB_Car_Node and agent)
shutdown_log "Step 2: Force killing any remaining ROS2 processes"

# Kill ROS2 launches and navigation processes
pkill -9 -f "ros2 launch" 2>/dev/null
pkill -9 -f "yahboomcar_nav" 2>/dev/null  
pkill -9 -f "yahboomcar_bringup" 2>/dev/null
pkill -9 -f "nav2" 2>/dev/null
pkill -9 -f "rviz" 2>/dev/null
pkill -9 -f "amcl" 2>/dev/null

# Kill Gazebo simulation processes
shutdown_log "Stopping Gazebo simulation processes"
pkill -9 -f "gazebo" 2>/dev/null
pkill -9 -f "gzserver" 2>/dev/null
pkill -9 -f "gzclient" 2>/dev/null
pkill -9 -f "gazebo_ros2_control" 2>/dev/null
pkill -9 -f "gazebo_ros_lidar_controller" 2>/dev/null
pkill -9 -f "spawn_entity" 2>/dev/null
pkill -9 -f "ros_gz_bridge" 2>/dev/null

# Kill ros2_control spawner processes
shutdown_log "Stopping ros2_control spawner processes"
pkill -9 -f "controller_manager/spawner.*diff_drive_controller" 2>/dev/null
pkill -9 -f "controller_manager/spawner.*joint_state_broadcaster" 2>/dev/null
pkill -9 -f "controller_manager/spawner" 2>/dev/null

# Kill Python ROS2 scripts but preserve the micro_ros_agent
ps aux | grep "python.*ros2" | grep -v "micro_ros_agent" | awk '{print $2}' | xargs -r kill -9 2>/dev/null

# Force kill common background ROS nodes that persist
pkill -9 -f "complementary_filter_node" 2>/dev/null
pkill -9 -f "static_transform_publisher" 2>/dev/null
pkill -9 -f "joint_state_publisher" 2>/dev/null
pkill -9 -f "robot_state_publisher" 2>/dev/null
pkill -9 -f "base_link_to_base_imu" 2>/dev/null
pkill -9 -f "base_link_to_base_laser" 2>/dev/null
pkill -9 -f "tf2_ros" 2>/dev/null

# CRITICAL: Kill robot_localization processes that survive cleanup
pkill -9 -f "ekf_node" 2>/dev/null
pkill -9 -f "robot_localization" 2>/dev/null
pkill -9 -f "ukf_node" 2>/dev/null

# Step 3: Stop waypoint navigation, robot manager, and autonomous explorer processes
shutdown_log "Step 3: Stopping waypoint navigation, robot manager, and autonomous explorer processes"
pkill -f "b4m_waypoint_nav" 2>/dev/null
pkill -f "b4m_robot_manager" 2>/dev/null
pkill -f "autonomous_explorer" 2>/dev/null
sleep 2
pkill -9 -f "b4m_waypoint_nav" 2>/dev/null
pkill -9 -f "b4m_robot_manager" 2>/dev/null
pkill -9 -f "autonomous_explorer" 2>/dev/null

# Step 4: Stop Micro-ROS agent Docker container (if not keeping it)
if [ "$KEEP_AGENT" = true ]; then
    shutdown_log "Step 4: Keeping Micro-ROS agent running (--keep-agent specified)"
else
    shutdown_log "Step 4: Stopping Micro-ROS agent Docker container"
    docker_containers=$(docker ps --filter 'ancestor=microros/micro-ros-agent:humble' --format '{{.ID}}' 2>/dev/null || true)
    if [ ! -z "$docker_containers" ]; then
        echo "$docker_containers" | while read -r container_id; do
            if [ ! -z "$container_id" ]; then
                shutdown_log "Stopping Micro-ROS agent container: $container_id"
                docker stop "$container_id" 2>/dev/null || true
                docker rm "$container_id" 2>/dev/null || true
            fi
        done
    else
        shutdown_log "No Micro-ROS agent Docker containers found running"
    fi
fi

# Step 5: Clean up remaining Python processes related to the robot
shutdown_log "Step 5: Cleaning up remaining Python processes"
pkill -f "python.*yahboom" 2>/dev/null
pkill -f "python.*b4m" 2>/dev/null

# Step 6: Stop RViz if running
shutdown_log "Step 6: Stopping RViz if running"
pkill -f "rviz2" 2>/dev/null
sleep 2
pkill -9 -f "rviz2" 2>/dev/null

# Step 7: Clean up any remaining terminals from the launch script
shutdown_log "Step 7: Cleaning up launch script terminals"
pkill -f "xterm.*b4m_step" 2>/dev/null

# Step 8: Stop ROS2 daemon to ensure all node discovery is cleared
shutdown_log "Step 8: Stopping ROS2 daemon"
ros2 daemon stop 2>/dev/null || true
sleep 2

# Step 9: Verify all nodes are stopped
shutdown_log "Step 9: Verifying all ROS2 nodes are stopped"
remaining_nodes=$(ros2 node list 2>/dev/null || true)

if [ "$KEEP_AGENT" = true ]; then
    # Check if only YB_Car_Node is running
    yb_car_found=false
    other_nodes_found=false
    
    if [ ! -z "$remaining_nodes" ]; then
        while read -r node; do
            if [ ! -z "$node" ]; then
                if [[ "$node" == *"YB_Car_Node"* ]]; then
                    yb_car_found=true
                    shutdown_log "YB_Car_Node still running (as expected): $node"
                else
                    other_nodes_found=true
                    shutdown_log "ERROR: Unexpected node still running: $node"
                fi
            fi
        done <<< "$remaining_nodes"
    fi
    
    if [ "$other_nodes_found" = true ]; then
        shutdown_log "ERROR: Failed to stop all non-YB_Car_Node nodes!"
        shutdown_log "Remaining nodes:"
        echo "$remaining_nodes" | tee -a "$SHUTDOWN_LOG"
        echo "======================================================"
        echo "ERROR: Shutdown FAILED - Some nodes are still running!"
        echo "======================================================"
        exit 1
    elif [ "$yb_car_found" = false ]; then
        shutdown_log "WARNING: YB_Car_Node is not running (expected with --keep-agent)"
        shutdown_log "This may indicate the Micro-ROS agent is not connected properly"
    else
        shutdown_log "SUCCESS: Only YB_Car_Node is running as expected"
    fi
else
    # No nodes should be running
    if [ ! -z "$remaining_nodes" ]; then
        shutdown_log "ERROR: ROS2 nodes are still running after shutdown:"
        echo "$remaining_nodes" | while read -r node; do
            if [ ! -z "$node" ]; then
                shutdown_log "  - $node"
            fi
        done
        echo "======================================================"
        echo "ERROR: Shutdown FAILED - ROS2 nodes are still running!"
        echo "======================================================"
        exit 1
    else
        shutdown_log "SUCCESS: All ROS2 nodes have been stopped"
    fi
fi

# Final status check
shutdown_log "Checking for any remaining robot-related processes:"
remaining_processes=$(ps aux | grep -E "(yahboom|b4m|nav2|rviz|gazebo)" | grep -v grep | grep -v "b4m_shutdown" || true)
if [ ! -z "$remaining_processes" ]; then
    shutdown_log "WARNING: Some processes may still be running:"
    echo "$remaining_processes" | tee -a "$SHUTDOWN_LOG"
else
    shutdown_log "All robot processes successfully stopped"
fi

# Check Micro-ROS agent status
microros_docker=$(docker ps --filter 'ancestor=microros/micro-ros-agent:humble' --format '{{.ID}}' 2>/dev/null || true)
microros_processes=$(ps aux | grep -E "micro.*ros.*agent" | grep -v grep || true)

if [ "$KEEP_AGENT" = true ]; then
    if [ ! -z "$microros_docker" ] || [ ! -z "$microros_processes" ]; then
        shutdown_log "Micro-ROS agent kept running as requested (--keep-agent)"
        if [ ! -z "$microros_docker" ]; then
            echo "Docker containers: $microros_docker" | tee -a "$SHUTDOWN_LOG"
        fi
        if [ ! -z "$microros_processes" ]; then
            echo "$microros_processes" | tee -a "$SHUTDOWN_LOG"
        fi
    else
        shutdown_log "WARNING: No Micro-ROS agent found running (expected with --keep-agent)"
    fi
else
    if [ ! -z "$microros_docker" ] || [ ! -z "$microros_processes" ]; then
        shutdown_log "WARNING: Micro-ROS agent is still running:"
        if [ ! -z "$microros_docker" ]; then
            echo "Docker containers: $microros_docker" | tee -a "$SHUTDOWN_LOG"
        fi
        if [ ! -z "$microros_processes" ]; then
            echo "$microros_processes" | tee -a "$SHUTDOWN_LOG"
        fi
    else
        shutdown_log "Micro-ROS agent successfully stopped"
    fi
fi

shutdown_log "B4M Robot shutdown script completed successfully"
echo "======================================================"
echo "Shutdown completed successfully!"
if [ "$KEEP_AGENT" = true ]; then
    echo "YB_Car_Node kept running (--keep-agent mode)"
else
    echo "All ROS2 nodes stopped"
fi
echo "Shutdown log saved to: $SHUTDOWN_LOG"
echo "======================================================"

exit 0
