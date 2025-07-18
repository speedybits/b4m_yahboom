#!/bin/bash

# Get the workspace root directory (where this script is located)
WORKSPACE_ROOT=$(cd "$(dirname "$0")" && pwd)

# Clear any existing ROS environment variables
unset ROS_DISTRO
unset ROS_DOMAIN_ID
unset ROS_LOCALHOST_ONLY
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset COLCON_PREFIX_PATH
unset PYTHONPATH

# Source ROS2 Humble
source /opt/ros/humble/setup.bash

# Source only our workspace
source "$WORKSPACE_ROOT/install/setup.bash"

# Print confirmation
echo "Environment set up for workspace: $WORKSPACE_ROOT"
echo "ROS_DISTRO: $ROS_DISTRO"
echo "AMENT_PREFIX_PATH: $AMENT_PREFIX_PATH"
