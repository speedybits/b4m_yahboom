#!/bin/bash

# Script to rebuild all workspaces with correct paths
# This ensures that no external paths are referenced

echo "Rebuilding all workspaces with correct paths..."

# Set the base directory
BASE_DIR="/home/yahboom/b4m_yahboom"

# Clean and rebuild gmapping_ws
echo "Cleaning gmapping_ws..."
rm -rf $BASE_DIR/gmapping_ws/build $BASE_DIR/gmapping_ws/install $BASE_DIR/gmapping_ws/log

echo "Building gmapping_ws..."
cd $BASE_DIR/gmapping_ws
COLCON_PREFIX_PATH=$BASE_DIR/install colcon build --cmake-args -DCMAKE_INSTALL_PREFIX=$BASE_DIR/gmapping_ws/install

# Clean and rebuild imu_ws
echo "Cleaning imu_ws..."
rm -rf $BASE_DIR/imu_ws/build $BASE_DIR/imu_ws/install $BASE_DIR/imu_ws/log

echo "Building imu_ws..."
cd $BASE_DIR/imu_ws
COLCON_PREFIX_PATH=$BASE_DIR/install colcon build --packages-skip rviz_imu_plugin --cmake-args -DCMAKE_INSTALL_PREFIX=$BASE_DIR/imu_ws/install

# Clean and rebuild uros_ws
echo "Cleaning uros_ws..."
rm -rf $BASE_DIR/uros_ws/build $BASE_DIR/uros_ws/install $BASE_DIR/uros_ws/log

echo "Building uros_ws..."
cd $BASE_DIR/uros_ws
COLCON_PREFIX_PATH=$BASE_DIR/install colcon build --cmake-args -DCMAKE_INSTALL_PREFIX=$BASE_DIR/uros_ws/install

echo "All workspaces rebuilt with correct paths."
