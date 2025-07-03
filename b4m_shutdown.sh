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
pkill -f "waypoint_manager_node.py"

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

if is_running "waypoint_manager_node.py"; then
    echo "Force killing waypoint manager GUI..."
    pkill -9 -f "waypoint_manager_node.py"
fi

# Stop the Micro-ROS agent Docker container
echo "Stopping Micro-ROS agent Docker container..."
CONTAINER_ID=$(docker ps | grep "microros/micro-ros-agent:humble" | awk '{print $1}')
if [ ! -z "$CONTAINER_ID" ]; then
    docker stop $CONTAINER_ID
    echo "Micro-ROS agent container stopped."
else
    echo "No running Micro-ROS agent container found."
fi

# Kill any remaining Python processes related to the B4M system
echo "Checking for remaining Python processes..."
pkill -f "python3 /home/yahboom/b4m_yahboom"

# Kill RViz if it's still running
if is_running "rviz"; then
    echo "Stopping RViz..."
    pkill -f "rviz"
fi

echo "B4M shutdown sequence complete. System is now in a clean state."
echo "To restart the system, follow the steps in B4M_launch.md"
