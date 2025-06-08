#!/bin/bash

# Script to source all workspaces with the correct paths
# This ensures that no external paths are referenced

# Source ROS2 first
source /opt/ros/humble/setup.bash

# Source the main workspace
source /home/yahboom/b4m_yahboom/install/setup.bash

# Source the gmapping workspace
source /home/yahboom/b4m_yahboom/gmapping_ws/install/setup.bash

# Source the IMU workspace
source /home/yahboom/b4m_yahboom/imu_ws/install/setup.bash

# Source the uROS workspace
source /home/yahboom/b4m_yahboom/uros_ws/install/setup.bash

echo "All workspaces sourced with correct paths."
echo "You can now use the ROS2 commands with the moved workspaces."
