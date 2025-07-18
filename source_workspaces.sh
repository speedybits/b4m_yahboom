#!/bin/bash

# Script to source all workspaces with the correct paths
# This ensures that no external paths are referenced

# Get the workspace root directory (where this script is located)
if [ -n "${BASH_SOURCE[0]}" ]; then
    WORKSPACE_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
else
    WORKSPACE_ROOT=$(pwd)
fi

# Source ROS2 first
source /opt/ros/humble/setup.bash

# Source the main workspace
source "$WORKSPACE_ROOT/install/setup.bash"

# Source the gmapping workspace (if exists)
if [ -f "$WORKSPACE_ROOT/gmapping_ws/install/setup.bash" ]; then
    source "$WORKSPACE_ROOT/gmapping_ws/install/setup.bash"
fi

# Source the IMU workspace (if exists)
if [ -f "$WORKSPACE_ROOT/imu_ws/install/setup.bash" ]; then
    source "$WORKSPACE_ROOT/imu_ws/install/setup.bash"
fi

# Source the uROS workspace (if exists)
if [ -f "$WORKSPACE_ROOT/uros_ws/install/setup.bash" ]; then
    source "$WORKSPACE_ROOT/uros_ws/install/setup.bash"
fi

echo "All workspaces sourced from: $WORKSPACE_ROOT"
echo "You can now use the ROS2 commands with the workspaces."
