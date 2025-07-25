#!/bin/bash

# B4M Robot Manager GUI Shortcut
# This script sets up the environment and launches the Robot Manager GUI

# Get the workspace root directory (where this script is located)
WORKSPACE_ROOT=$(cd "$(dirname "$0")" && pwd)

echo "B4M Robot Manager GUI"
echo "====================="
echo "Setting up environment..."

# Change to workspace directory
cd "$WORKSPACE_ROOT"

# Source the workspace
if [ -f 'install/setup.bash' ]; then
    source install/setup.bash
    echo "✅ Workspace sourced successfully"
else
    echo "❌ ERROR: install/setup.bash not found!"
    echo "Please run 'colcon build' first."
    exit 1
fi

echo "🚀 Launching Robot Manager GUI..."
echo ""

# Launch the Robot Manager GUI
ros2 run b4m_waypoint_nav b4m_robot_manager_node.py