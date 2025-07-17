#!/bin/bash
# B4M Shutdown Script
# This script stops all processes from the B4M_launch.md to return the system to a freshly rebooted state

echo "Starting B4M shutdown sequence..."

# Function to check if a process is running
is_running() {
    pgrep -f "$1" > /dev/null
    return $?
}

# Stop all ROS2 nodes
echo "Stopping all ROS2 nodes..."
pkill -f "ros2 launch"
pkill -f "b4m_waypoint_nav.py"
pkill -f "b4m_robot_manager_node.py"

# Give ROS2 nodes time to shutdown gracefully
sleep 2

# Force kill any remaining ROS2 processes if they're still running
if is_running "ros2"; then
    echo "Force killing remaining ROS2 processes..."
    pkill -9 -f "ros2"
fi

if is_running "b4m_waypoint_nav.py"; then
    echo "Force killing waypoint navigation node..."
    pkill -9 -f "b4m_waypoint_nav.py"
fi

if is_running "b4m_robot_manager_node.py"; then
    echo "Force killing robot manager GUI..."
    pkill -9 -f "b4m_robot_manager_node.py"
fi

# Stop Micro-ROS agent Docker containers
echo "Stopping Micro-ROS agent Docker containers..."
MICROROS_CONTAINERS=$(docker ps -q --filter "ancestor=microros/micro-ros-agent:humble")
if [ ! -z "$MICROROS_CONTAINERS" ]; then
    echo "Found running micro-ros agent containers, stopping them..."
    docker stop $MICROROS_CONTAINERS
else
    echo "No micro-ros agent containers running"
fi

# Kill any remaining Python processes related to the B4M system
echo "Checking for remaining Python processes..."
# Get the workspace root directory (where this script is located)
WORKSPACE_ROOT=$(cd "$(dirname "$0")" && pwd)
pkill -f "python3 \"$WORKSPACE_ROOT"

# Kill RViz if it's still running
if is_running "rviz"; then
    echo "Stopping RViz..."
    pkill -f "rviz"
fi

echo "B4M shutdown sequence complete. System is now in a clean state."
echo "To restart the system, follow the steps in B4M_launch.md"
