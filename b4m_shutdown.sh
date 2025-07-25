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

# Step 1: Stop all ROS2 nodes gracefully
shutdown_log "Step 1: Stopping all ROS2 nodes gracefully"
ros2 node list 2>/dev/null | while read -r node; do
    if [ ! -z "$node" ]; then
        shutdown_log "Stopping ROS2 node: $node"
    fi
done

# Send SIGTERM to all ROS2 processes
pkill -f "ros2" 2>/dev/null
sleep 3

# Step 2: Force kill remaining ROS2 processes if needed
shutdown_log "Step 2: Force killing any remaining ROS2 processes"
pkill -9 -f "ros2" 2>/dev/null
pkill -9 -f "yahboom" 2>/dev/null
pkill -9 -f "nav2" 2>/dev/null
pkill -9 -f "rviz" 2>/dev/null

# Step 3: Stop waypoint navigation and robot manager processes
shutdown_log "Step 3: Stopping waypoint navigation and robot manager processes"
pkill -f "b4m_waypoint_nav" 2>/dev/null
pkill -f "b4m_robot_manager" 2>/dev/null
sleep 2
pkill -9 -f "b4m_waypoint_nav" 2>/dev/null
pkill -9 -f "b4m_robot_manager" 2>/dev/null

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

# Final status check
shutdown_log "Checking for any remaining robot-related processes:"
remaining_processes=$(ps aux | grep -E "(yahboom|b4m|nav2|rviz)" | grep -v grep | grep -v "b4m_shutdown" || true)
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

shutdown_log "B4M Robot shutdown script completed"
echo "======================================================"
echo "Shutdown completed!"
echo "Shutdown log saved to: $SHUTDOWN_LOG"
echo "======================================================"

exit 0
