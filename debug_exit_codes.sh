#!/bin/bash

# This script focuses on debugging exit codes in the Yahboom Gazebo simulation stack
# Specifically targeting exit code 143 (SIGTERM) and 137 (SIGKILL) issues

# Enable bash debugging
set -x

# Create log directory and file
LOG_DIR="/home/yahboom/b4m_yahboom/logs"
LOG_FILE="${LOG_DIR}/exit_code_debug_$(date +%Y%m%d_%H%M%S).log"
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

# Function to monitor a process and log its exit code
monitor_process() {
  local pid=$1
  local name=$2
  local log_file=$3
  
  if [ -z "$pid" ]; then
    log_message "ERROR: No PID provided for monitoring $name"
    return 1
  fi
  
  log_message "Starting to monitor $name process with PID $pid"
  
  # Monitor in background
  (
    # Wait for process to exit
    while kill -0 $pid 2>/dev/null; do
      sleep 1
    done
    
    # Get exit code
    wait $pid
    exit_code=$?
    
    log_message "$name process (PID $pid) exited with code $exit_code"
    
    # Analyze exit code
    case $exit_code in
      0)
        log_message "$name exited normally"
        ;;
      130)
        log_message "$name was terminated by SIGINT (Ctrl+C)"
        ;;
      137)
        log_message "$name was killed by SIGKILL (kill -9) - likely due to out of memory or other system resource issue"
        log_message "Checking system resources at time of kill:"
        free -h >> "$log_file"
        ;;
      143)
        log_message "$name was terminated by SIGTERM (kill) - likely by another process or system service"
        log_message "Checking active processes that might have sent SIGTERM:"
        ps aux | grep -E 'systemd|upstart|init|oom|cgroup' >> "$log_file"
        ;;
      *)
        log_message "$name exited with unexpected code $exit_code"
        ;;
    esac
  ) &
}

# Clean up existing processes
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
sleep 3

log_message "Running processes after cleanup:"
ps aux | grep -E 'gazebo|rviz|ros2' | grep -v grep >> "${LOG_FILE}"

# Check system resources before starting
log_message "Checking system resources before starting..."
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

# Launch Gazebo in the background with output to log file
gazebo --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so > "${LOG_FILE}.gazebo" 2>&1 &
GAZEBO_PID=$!
log_message "Gazebo started with PID: $GAZEBO_PID"

# Monitor Gazebo process
monitor_process $GAZEBO_PID "Gazebo" "${LOG_FILE}.gazebo"

# Wait for Gazebo to start
log_message "Waiting for Gazebo to start..."
sleep 10

# Check if Gazebo is still running
if kill -0 $GAZEBO_PID 2>/dev/null; then
  log_message "Gazebo is still running with PID: $GAZEBO_PID"
else
  log_message "ERROR: Gazebo exited prematurely. Check logs for details."
  exit 1
fi

# Step 2: Create fixed URDF file with Gazebo plugins
log_message "Step 2: Creating fixed URDF file with Gazebo plugins..."
FIXED_URDF="/home/yahboom/b4m_yahboom/yahboomcar_description/urdf/yahboomcar_robot2_gazebo.urdf"

if [ ! -f "$FIXED_URDF" ]; then
  log_message "ERROR: Fixed URDF file not found at $FIXED_URDF"
  exit 1
fi

log_message "Fixed URDF file exists at $FIXED_URDF"

# Step 3: Spawn the robot model in Gazebo
log_message "Step 3: Spawning the robot model in Gazebo..."
log_message "Command: ros2 run gazebo_ros spawn_entity.py -entity robot2 -file $FIXED_URDF"

# Spawn robot in the background with output to log file
ros2 run gazebo_ros spawn_entity.py -entity robot2 -file $FIXED_URDF > "${LOG_FILE}.spawn" 2>&1 &
SPAWN_PID=$!
log_message "Robot spawn started with PID: $SPAWN_PID"

# Monitor spawn process
monitor_process $SPAWN_PID "Robot spawn" "${LOG_FILE}.spawn"

# Wait for robot to spawn
log_message "Waiting for robot to spawn..."
sleep 5

# Check system resources after spawning
log_message "Checking system resources after spawning..."
log_message "Memory usage:"
free -h >> "${LOG_FILE}"
log_message "CPU load:"
uptime >> "${LOG_FILE}"

# Step 4: Add necessary transforms
log_message "Step 4: Adding necessary transforms..."

# Add map to odom transform
log_message "Adding map to odom transform..."
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom > "${LOG_FILE}.map_odom_tf" 2>&1 &
MAP_ODOM_PID=$!
log_message "Map to odom transform started with PID: $MAP_ODOM_PID"

# Monitor transform process
monitor_process $MAP_ODOM_PID "Map to odom transform" "${LOG_FILE}.map_odom_tf"

sleep 2

# Add odom to base_link transform
log_message "Adding odom to base_link transform..."
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom robot2/base_link > "${LOG_FILE}.odom_baselink_tf" 2>&1 &
ODOM_BASELINK_PID=$!
log_message "Odom to base_link transform started with PID: $ODOM_BASELINK_PID"

# Monitor transform process
monitor_process $ODOM_BASELINK_PID "Odom to base_link transform" "${LOG_FILE}.odom_baselink_tf"

sleep 2

# Check if transforms are running
if ! kill -0 $MAP_ODOM_PID 2>/dev/null || ! kill -0 $ODOM_BASELINK_PID 2>/dev/null; then
  log_message "ERROR: One or more transforms exited prematurely. Check logs for details."
fi

# Check system resources after transforms
log_message "Checking system resources after transforms..."
log_message "Memory usage:"
free -h >> "${LOG_FILE}"
log_message "CPU load:"
uptime >> "${LOG_FILE}"

# Step 5: Start RViz for visualization
log_message "Step 5: Starting RViz for visualization..."
RVIZ_CONFIG="/opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz"

if [ ! -f "$RVIZ_CONFIG" ]; then
  log_message "WARNING: Nav2 default view config not found at $RVIZ_CONFIG"
  log_message "Will try to start RViz without config file"
  rviz2 > "${LOG_FILE}.rviz" 2>&1 &
  RVIZ_PID=$!
else
  log_message "Starting RViz with Nav2 default view config"
  rviz2 -d $RVIZ_CONFIG > "${LOG_FILE}.rviz" 2>&1 &
  RVIZ_PID=$!
fi

log_message "RViz started with PID: $RVIZ_PID"

# Monitor RViz process
monitor_process $RVIZ_PID "RViz" "${LOG_FILE}.rviz"

# Wait for RViz to start
log_message "Waiting for RViz to start..."
sleep 5

# Check if RViz is running
if ! kill -0 $RVIZ_PID 2>/dev/null; then
  log_message "ERROR: RViz exited prematurely. Check logs for details."
fi

# Check system resources after RViz
log_message "Checking system resources after RViz..."
log_message "Memory usage:"
free -h >> "${LOG_FILE}"
log_message "CPU load:"
uptime >> "${LOG_FILE}"

# Step 6: Launch Nav2 with AMCL
log_message "Step 6: Launching Nav2 with AMCL..."
log_message "Command: ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true autostart:=true"

# Launch Nav2 in the background with output to log file
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true autostart:=true > "${LOG_FILE}.nav2" 2>&1 &
NAV2_PID=$!
log_message "Nav2 started with PID: $NAV2_PID"

# Monitor Nav2 process
monitor_process $NAV2_PID "Nav2" "${LOG_FILE}.nav2"

# Wait for Nav2 to start
log_message "Waiting for Nav2 to start..."
sleep 10

# Check if Nav2 is running
if ! kill -0 $NAV2_PID 2>/dev/null; then
  log_message "ERROR: Nav2 exited prematurely. Check logs for details."
fi

# Check system resources after Nav2
log_message "Checking system resources after Nav2..."
log_message "Memory usage:"
free -h >> "${LOG_FILE}"
log_message "CPU load:"
uptime >> "${LOG_FILE}"

# Step 7: Remind about setting initial pose in RViz
log_message "Step 7: Reminding about setting initial pose in RViz..."
echo ""
echo "=================================================================================="
echo "CRITICAL STEP: You must set the initial pose in RViz using the 2D Pose Estimate tool."
echo "This is essential for AMCL to start publishing the map->base_link transform."
echo "1. In RViz, click on the '2D Pose Estimate' button in the toolbar"
echo "2. Click and drag on the map to set the robot's position and orientation"
echo "=================================================================================="
echo ""
echo "Press ENTER after you have set the initial pose in RViz..."

# Wait for user confirmation
read
log_message "User confirmed initial pose has been set"

# Step 8: Launch keyboard teleop for robot control
log_message "Step 8: Launching keyboard teleop for robot control..."
log_message "Command: ros2 run teleop_twist_keyboard teleop_twist_keyboard"

# Launch teleop in the background with output to log file
ros2 run teleop_twist_keyboard teleop_twist_keyboard > "${LOG_FILE}.teleop" 2>&1 &
TELEOP_PID=$!
log_message "Teleop started with PID: $TELEOP_PID"

# Monitor teleop process
monitor_process $TELEOP_PID "Teleop" "${LOG_FILE}.teleop"

# Wait for teleop to start
log_message "Waiting for teleop to start..."
sleep 3

# Check if teleop is running
if ! kill -0 $TELEOP_PID 2>/dev/null; then
  log_message "ERROR: Teleop exited prematurely. Check logs for details."
fi

# Check system resources after teleop
log_message "Checking system resources after teleop..."
log_message "Memory usage:"
free -h >> "${LOG_FILE}"
log_message "CPU load:"
uptime >> "${LOG_FILE}"

# Final status check
log_message "Performing final status check..."

# Check if all required components are running
COMPONENTS_OK=true

if ! kill -0 $GAZEBO_PID 2>/dev/null; then
  log_message "ERROR: Gazebo is not running"
  COMPONENTS_OK=false
fi

if ! kill -0 $RVIZ_PID 2>/dev/null; then
  log_message "ERROR: RViz is not running"
  COMPONENTS_OK=false
fi

if ! kill -0 $NAV2_PID 2>/dev/null; then
  log_message "ERROR: Nav2 is not running"
  COMPONENTS_OK=false
fi

if ! kill -0 $TELEOP_PID 2>/dev/null; then
  log_message "ERROR: Teleop is not running"
  COMPONENTS_OK=false
fi

if [ "$COMPONENTS_OK" = true ]; then
  log_message "All components appear to be running correctly"
else
  log_message "WARNING: Some components are not running. Check the log file at ${LOG_FILE}"
fi

# Display final status and instructions
echo "======================================================================"
echo "Yahboom Gazebo simulation is now running!"
echo "======================================================================"
echo "1. Use the keyboard teleop terminal to move the robot:"
echo "   - i/j/k/l: Basic movement controls"
echo "   - u/o/m/.: Diagonal movement controls"
echo "   - Space: Stop the robot"
echo "   - q/z: Increase/decrease max speeds"
echo "   - w/x: Increase/decrease linear speed"
echo "   - e/c: Increase/decrease angular speed"
echo ""
echo "2. Press Ctrl+C to exit and kill all processes."
echo "======================================================================"
echo ""
echo "Log file: ${LOG_FILE}"
echo "Check the log file for detailed debugging information if you encounter issues."
log_message "Script completed successfully"

# Keep the script running to prevent exit code 143
while true; do
  # Check if any component has exited
  if ! kill -0 $GAZEBO_PID 2>/dev/null; then
    log_message "Gazebo has exited. Exit code will be in the log."
  fi
  
  if ! kill -0 $RVIZ_PID 2>/dev/null; then
    log_message "RViz has exited. Exit code will be in the log."
  fi
  
  if ! kill -0 $NAV2_PID 2>/dev/null; then
    log_message "Nav2 has exited. Exit code will be in the log."
  fi
  
  if ! kill -0 $TELEOP_PID 2>/dev/null; then
    log_message "Teleop has exited. Exit code will be in the log."
  fi
  
  # Check system resources periodically
  if (( SECONDS % 30 == 0 )); then
    log_message "Periodic system resource check:"
    log_message "Memory usage:"
    free -h >> "${LOG_FILE}"
    log_message "CPU load:"
    uptime >> "${LOG_FILE}"
  fi
  
  sleep 5
done
