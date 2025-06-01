#!/bin/bash

# This script focuses on debugging the exit code 143 issue

# Create log directory and file
LOG_DIR="/home/yahboom/b4m_yahboom/logs"
LOG_FILE="${LOG_DIR}/debug_$(date +%Y%m%d_%H%M%S).log"
mkdir -p "${LOG_DIR}" 2>/dev/null
touch "${LOG_FILE}"

# Function to log messages
log_message() {
  local message="$1"
  local timestamp=$(date +"%Y-%m-%d %H:%M:%S")
  echo "[${timestamp}] ${message}" | tee -a "${LOG_FILE}"
}

# Function to check if a process is running
process_running() {
  local process_name="$1"
  if pgrep -f "$process_name" >/dev/null; then
    log_message "Process '$process_name' is running"
    return 0
  else
    log_message "Process '$process_name' is not running"
    return 1
  fi
}

# Clean up existing processes - use pkill without -9 first to allow graceful shutdown
log_message "Cleaning up existing processes..."
log_message "Running processes before cleanup:"
ps aux | grep -E 'gazebo|rviz|ros2' | grep -v grep >> "${LOG_FILE}"

# Try gentle shutdown first
log_message "Attempting gentle shutdown of processes..."
pkill -f gazebo || log_message "No gazebo processes found"
pkill -f gzserver || log_message "No gzserver processes found"
pkill -f gzclient || log_message "No gzclient processes found"
pkill -f rviz2 || log_message "No rviz2 processes found"
pkill -f "ros2 run tf2_ros static_transform_publisher" || log_message "No static_transform_publisher processes found"
pkill -f "ros2 launch nav2_bringup" || log_message "No nav2_bringup processes found"
pkill -f "teleop_twist_keyboard" || log_message "No teleop_twist_keyboard processes found"

log_message "Waiting for processes to terminate..."
sleep 2

log_message "Running processes after cleanup:"
ps aux | grep -E 'gazebo|rviz|ros2' | grep -v grep >> "${LOG_FILE}"

# Check system resources
log_message "Checking system resources..."
log_message "Memory usage:"
free -h >> "${LOG_FILE}"
log_message "Disk space:"
df -h >> "${LOG_FILE}"
log_message "CPU load:"
uptime >> "${LOG_FILE}"

# Source ROS workspace
log_message "Sourcing ROS workspace..."
source /opt/ros/humble/setup.bash
source /home/yahboom/b4m_yahboom/install/setup.bash 2>/dev/null || true

# Export Gazebo model path
log_message "Exporting Gazebo model path..."
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/yahboom/b4m_yahboom/yahboomcar_description/models

# Step 1: Start Gazebo server with ROS2 plugins
log_message "Step 1: Starting Gazebo server with ROS2 plugins..."
log_message "Command: gazebo --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so"

# Launch Gazebo in the foreground to see if it's causing the exit code 143
log_message "Launching Gazebo in the foreground to debug exit code..."
gazebo --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so 2>&1 | tee -a "${LOG_FILE}.gazebo" &
GAZEBO_PID=$!
log_message "Gazebo started with PID: $GAZEBO_PID"

log_message "Waiting for Gazebo to start..."
sleep 5

# Check if Gazebo is still running
if kill -0 $GAZEBO_PID 2>/dev/null; then
  log_message "Gazebo is still running with PID: $GAZEBO_PID"
else
  log_message "ERROR: Gazebo exited prematurely. Exit code: $?"
  log_message "Last lines of Gazebo log:"
  tail -n 20 "${LOG_FILE}.gazebo" >> "${LOG_FILE}"
fi

# Wait for user input to continue or exit
log_message "Debug script completed. Check the log file at ${LOG_FILE}"
echo "Debug script completed. Check the log file at ${LOG_FILE}"
echo "Gazebo should be running in the background if it started successfully."
echo "Press Ctrl+C to exit and kill all processes."

# Keep the script running to prevent exit code 143
while true; do
  sleep 1
done
