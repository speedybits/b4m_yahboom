#!/bin/bash

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
source /home/yahboom/b4m_yahboom/install/setup.bash

# Print confirmation
echo "Environment set up for /home/yahboom/b4m_yahboom workspace only"
echo "ROS_DISTRO: $ROS_DISTRO"
echo "AMENT_PREFIX_PATH: $AMENT_PREFIX_PATH"
