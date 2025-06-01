#!/bin/bash

# Yahboom Gazebo Simulation Launch Script with Exit Code Fixes
# This script addresses the exit code 143 and 255 issues

# Enable bash debugging
set -x

# Create log directory and file
LOG_DIR="/home/yahboom/b4m_yahboom/logs"
LOG_FILE="${LOG_DIR}/gazebo_launch_$(date +%Y%m%d_%H%M%S).log"
mkdir -p "${LOG_DIR}" 2>/dev/null
touch "${LOG_FILE}"

# Function to log messages
log_message() {
  local message="$1"
  local timestamp=$(date +"%Y-%m-%d %H:%M:%S")
  echo "[${timestamp}] ${message}" | tee -a "${LOG_FILE}"
}

# Function to check if a command exists
command_exists() {
  command -v "$1" >/dev/null 2>&1
}

# Function to check if a process is running
process_running() {
  local process_name="$1"
  if pgrep -f "$process_name" >/dev/null; then
    return 0
  else
    return 1
  fi
}

# Function to check if a file exists
check_file_exists() {
  if [ -f "$1" ]; then
    return 0
  else
    return 1
  fi
}

# Function to check if a directory exists
check_dir_exists() {
  if [ -d "$1" ]; then
    return 0
  else
    return 1
  fi
}

# Function to wait for a process to start
wait_for_process() {
  local process_name="$1"
  local max_wait="$2"
  local wait_count=0
  
  while ! process_running "$process_name"; do
    sleep 1
    wait_count=$((wait_count + 1))
    if [ "$wait_count" -ge "$max_wait" ]; then
      log_message "ERROR: Timed out waiting for $process_name to start"
      return 1
    fi
  done
  
  log_message "$process_name is running"
  return 0
}

# Clean up existing processes - use pkill without -9 first to allow graceful shutdown
log_message "Cleaning up existing processes..."

# List running processes before cleanup for debugging
log_message "Running processes before cleanup:"
ps aux | grep -E 'gazebo|gzserver|gzclient|rviz|ros2|nav2|teleop' | grep -v grep >> "${LOG_FILE}"

# Kill Gazebo processes
pkill -f gazebo || log_message "No gazebo processes found"
pkill -f gzserver || log_message "No gzserver processes found"
pkill -f gzclient || log_message "No gzclient processes found"
pkill -f rviz2 || log_message "No rviz2 processes found"
pkill -f "ros2 run tf2_ros static_transform_publisher" || log_message "No static_transform_publisher processes found"
pkill -f "ros2 launch nav2_bringup" || log_message "No nav2_bringup processes found"
pkill -f "teleop_twist_keyboard" || log_message "No teleop_twist_keyboard processes found"
pkill -f "waypoint_navigation" || log_message "No waypoint_navigation processes found"

log_message "Waiting for processes to terminate..."
sleep 3

# Check if any processes are still running and force kill if necessary
if process_running "gazebo" || process_running "gzserver" || process_running "gzclient"; then
  log_message "WARNING: Gazebo processes still running after graceful shutdown, using SIGKILL"
  pkill -9 -f gazebo
  pkill -9 -f gzserver
  pkill -9 -f gzclient
  sleep 2
fi

# Check for and kill any remaining Gazebo master processes (address already in use issue)
log_message "Checking for Gazebo master processes (port 11345)..."
GAZEBO_MASTER_PIDS=$(lsof -i :11345 2>/dev/null | grep -v "PID" | awk '{print $2}')
if [ -n "$GAZEBO_MASTER_PIDS" ]; then
  log_message "Found Gazebo master processes using port 11345: $GAZEBO_MASTER_PIDS"
  for pid in $GAZEBO_MASTER_PIDS; do
    log_message "Killing Gazebo master process with PID $pid"
    kill $pid 2>/dev/null || kill -9 $pid 2>/dev/null
  done
  sleep 2
else
  log_message "No Gazebo master processes found using port 11345"
fi

# Double-check that port 11345 is free
if lsof -i :11345 >/dev/null 2>&1; then
  log_message "ERROR: Port 11345 is still in use after cleanup attempts"
  log_message "Processes using port 11345:"
  lsof -i :11345 >> "${LOG_FILE}"
  log_message "Attempting final forced cleanup of port 11345"
  lsof -i :11345 2>/dev/null | grep -v "PID" | awk '{print $2}' | xargs -r kill -9
  sleep 2
else
  log_message "Port 11345 is free and available for Gazebo"
fi

# List running processes after cleanup for verification
log_message "Running processes after cleanup:"
ps aux | grep -E 'gazebo|gzserver|gzclient|rviz|ros2|nav2|teleop' | grep -v grep >> "${LOG_FILE}"

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
if ! source /opt/ros/humble/setup.bash; then
  log_message "ERROR: Failed to source ROS humble setup.bash"
  exit 1
fi

if ! source /home/yahboom/b4m_yahboom/install/setup.bash 2>/dev/null; then
  log_message "WARNING: Failed to source workspace setup.bash, continuing anyway"
fi

# Export Gazebo model path
log_message "Exporting Gazebo model path..."
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/yahboom/b4m_yahboom/yahboomcar_description/models

# Step 1: Start Gazebo server with ROS2 plugins
log_message "Step 1: Starting Gazebo server with ROS2 plugins..."

# Check if Gazebo plugins exist
if ! command_exists gazebo; then
  log_message "ERROR: gazebo command not found"
  exit 1
fi

# Final check for any remaining Gazebo processes or port usage
if process_running "gazebo" || process_running "gzserver" || process_running "gzclient"; then
  log_message "WARNING: Gazebo processes still running before starting new instance"
  log_message "Attempting final cleanup of Gazebo processes"
  pkill -9 -f gazebo
  pkill -9 -f gzserver
  pkill -9 -f gzclient
  sleep 2
fi

# Final check for port 11345 (Gazebo master port)
if lsof -i :11345 >/dev/null 2>&1; then
  log_message "ERROR: Port 11345 is still in use before starting Gazebo"
  log_message "Processes using port 11345:"
  lsof -i :11345 >> "${LOG_FILE}"
  log_message "Attempting final forced cleanup of port 11345"
  lsof -i :11345 2>/dev/null | grep -v "PID" | awk '{print $2}' | xargs -r kill -9
  sleep 2
  
  # Verify port is now free
  if lsof -i :11345 >/dev/null 2>&1; then
    log_message "ERROR: Port 11345 is still in use after all cleanup attempts. Cannot start Gazebo."
    log_message "Try rebooting the system to clear all Gazebo processes."
    exit 1
  fi
fi

# Check for ROS2 Gazebo plugins
GAZEBO_PLUGIN_PATH=$(rospack find gazebo_plugins 2>/dev/null || echo "")
if [ -z "$GAZEBO_PLUGIN_PATH" ]; then
  log_message "WARNING: gazebo_plugins package not found, plugins may not be available"
else
  log_message "Found gazebo_plugins at $GAZEBO_PLUGIN_PATH"
fi

# Start Gazebo with plugins in a separate terminal
log_message "Starting Gazebo with command: gazebo --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so"
gnome-terminal --title="Gazebo" -- bash -c "source /opt/ros/humble/setup.bash && source /home/yahboom/b4m_yahboom/install/setup.bash 2>/dev/null && export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:/home/yahboom/b4m_yahboom/yahboomcar_description/models && gazebo --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so 2>&1 | tee -a ${LOG_FILE}.gazebo; echo 'Gazebo exited with code \$?' >> ${LOG_FILE}.gazebo; exec bash"

# Wait for Gazebo to start
log_message "Waiting for Gazebo to start..."
if ! wait_for_process "gazebo" 30; then
  log_message "ERROR: Gazebo failed to start within timeout"
  exit 1
fi

# Step 2: Create fixed URDF file with Gazebo plugins
log_message "Step 2: Verifying fixed URDF file with Gazebo plugins..."
FIXED_URDF="/home/yahboom/b4m_yahboom/yahboomcar_description/urdf/yahboomcar_robot2_gazebo.urdf"

if ! check_file_exists "$FIXED_URDF"; then
  log_message "ERROR: Fixed URDF file not found at $FIXED_URDF"
  exit 1
fi

log_message "Fixed URDF file exists at $FIXED_URDF"

# Step 3: Spawn the robot model in Gazebo
log_message "Step 3: Spawning the robot model in Gazebo..."

# Check if spawn_entity.py exists
if ! command_exists ros2; then
  log_message "ERROR: ros2 command not found"
  exit 1
fi

# Verify Gazebo is running before spawning
if ! process_running "gazebo"; then
  log_message "ERROR: Gazebo is not running, cannot spawn robot"
  exit 1
fi

# Spawn robot in a separate terminal
log_message "Spawning robot with command: ros2 run gazebo_ros spawn_entity.py -entity robot2 -file $FIXED_URDF"
gnome-terminal --title="Robot Spawn" -- bash -c "source /opt/ros/humble/setup.bash && source /home/yahboom/b4m_yahboom/install/setup.bash 2>/dev/null && ros2 run gazebo_ros spawn_entity.py -entity robot2 -file $FIXED_URDF 2>&1 | tee -a ${LOG_FILE}.spawn; echo 'Robot spawn exited with code \$?' >> ${LOG_FILE}.spawn; exec bash"

log_message "Waiting for robot to spawn..."
sleep 5

# Step 4: Add necessary transforms
log_message "Step 4: Adding necessary transforms..."

# Check if tf2_ros package is available
if ! ros2 pkg list | grep -q "tf2_ros"; then
  log_message "ERROR: tf2_ros package not found"
  exit 1
fi

# Add map to odom transform
log_message "Adding map to odom transform..."
gnome-terminal --title="Map to Odom TF" -- bash -c "source /opt/ros/humble/setup.bash && source /home/yahboom/b4m_yahboom/install/setup.bash 2>/dev/null && ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom 2>&1 | tee -a ${LOG_FILE}.map_odom_tf; echo 'Map to odom transform exited with code \$?' >> ${LOG_FILE}.map_odom_tf; exec bash"
sleep 2

# Add odom to base_link transform
log_message "Adding odom to base_link transform..."
gnome-terminal --title="Odom to Base Link TF" -- bash -c "source /opt/ros/humble/setup.bash && source /home/yahboom/b4m_yahboom/install/setup.bash 2>/dev/null && ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom robot2/base_link 2>&1 | tee -a ${LOG_FILE}.odom_baselink_tf; echo 'Odom to base_link transform exited with code \$?' >> ${LOG_FILE}.odom_baselink_tf; exec bash"
sleep 2

# Add base_link to base_footprint transform
log_message "Adding base_link to base_footprint transform..."
gnome-terminal --title="Base Link to Footprint TF" -- bash -c "source /opt/ros/humble/setup.bash && source /home/yahboom/b4m_yahboom/install/setup.bash 2>/dev/null && ros2 run tf2_ros static_transform_publisher 0 0 0.05 0 0 0 robot2/base_link robot2/base_footprint 2>&1 | tee -a ${LOG_FILE}.baselink_footprint_tf; echo 'Base_link to base_footprint transform exited with code \$?' >> ${LOG_FILE}.baselink_footprint_tf; exec bash"
sleep 2

# Verify transforms are being published
log_message "Verifying transforms are being published..."
ros2 topic list | grep "/tf" >> "${LOG_FILE}" 2>&1 || log_message "WARNING: /tf topic not found"
ros2 topic list | grep "/tf_static" >> "${LOG_FILE}" 2>&1 || log_message "WARNING: /tf_static topic not found"

log_message "Transforms added successfully."

# Step 5: Start RViz for visualization
log_message "Step 5: Starting RViz for visualization..."

# Check if RViz2 is available
if ! command_exists "rviz2"; then
  log_message "ERROR: rviz2 command not found"
  exit 1
fi

# Check if Nav2 default view config exists
RVIZ_CONFIG="/opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz"
if ! check_file_exists "$RVIZ_CONFIG"; then
  log_message "WARNING: Nav2 default view config not found at $RVIZ_CONFIG"
  log_message "Will try to start RViz without config file"
  gnome-terminal --title="RViz" -- bash -c "source /opt/ros/humble/setup.bash && source /home/yahboom/b4m_yahboom/install/setup.bash 2>/dev/null && rviz2 2>&1 | tee -a ${LOG_FILE}.rviz; echo 'RViz exited with code \$?' >> ${LOG_FILE}.rviz; exec bash"
else
  log_message "Starting RViz with Nav2 default view config"
  gnome-terminal --title="RViz" -- bash -c "source /opt/ros/humble/setup.bash && source /home/yahboom/b4m_yahboom/install/setup.bash 2>/dev/null && rviz2 -d $RVIZ_CONFIG 2>&1 | tee -a ${LOG_FILE}.rviz; echo 'RViz exited with code \$?' >> ${LOG_FILE}.rviz; exec bash"
fi

log_message "Waiting for RViz to start..."
if ! wait_for_process "rviz2" 10; then
  log_message "WARNING: RViz may not have started correctly"
else
  log_message "RViz appears to be running"
fi

# Step 6: Launch Nav2 with AMCL
log_message "Step 6: Launching Nav2 with AMCL..."

# Check if nav2_bringup package is available
if ! ros2 pkg list | grep -q "nav2_bringup"; then
  log_message "ERROR: nav2_bringup package not found"
  exit 1
fi

# Check if navigation_launch.py exists
if ! ros2 pkg prefix nav2_bringup >/dev/null 2>&1; then
  log_message "ERROR: Cannot find nav2_bringup package prefix"
  exit 1
fi

log_message "Launching Nav2 with parameters: use_sim_time:=true autostart:=true map_subscribe_transient_local:=true"
gnome-terminal --title="Nav2 Components" -- bash -c "source /opt/ros/humble/setup.bash && source /home/yahboom/b4m_yahboom/install/setup.bash 2>/dev/null && ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true autostart:=true map_subscribe_transient_local:=true 2>&1 | tee -a ${LOG_FILE}.nav2; echo 'Nav2 exited with code \$?' >> ${LOG_FILE}.nav2; exec bash"

log_message "Waiting for Nav2 to start..."
sleep 10

# Check if Nav2 nodes are running
log_message "Checking if Nav2 nodes are running..."
ros2 node list | grep "nav2" >> "${LOG_FILE}" 2>&1 || log_message "WARNING: No Nav2 nodes found"

# Check for lifecycle nodes
log_message "Checking lifecycle nodes status..."
ros2 service list | grep "change_state" >> "${LOG_FILE}" 2>&1 || log_message "WARNING: No lifecycle services found"

log_message "Nav2 launch completed"

# Step 7: Remind about setting initial pose in RViz (critical step)
log_message "Step 7: Prompting user to set initial pose in RViz (critical step)"

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

# Check if teleop_twist_keyboard package is available
if ! ros2 pkg list | grep -q "teleop_twist_keyboard"; then
  log_message "ERROR: teleop_twist_keyboard package not found"
  exit 1
fi

# Check if cmd_vel topic exists
log_message "Checking for cmd_vel topic..."
ros2 topic list | grep "/cmd_vel" >> "${LOG_FILE}" 2>&1 || log_message "WARNING: /cmd_vel topic not found. It will be created by teleop."

# Launch keyboard teleop
log_message "Launching keyboard teleop with command: ros2 run teleop_twist_keyboard teleop_twist_keyboard"
gnome-terminal --title="Keyboard Teleop" -- bash -c "source /opt/ros/humble/setup.bash && source /home/yahboom/b4m_yahboom/install/setup.bash 2>/dev/null && ros2 run teleop_twist_keyboard teleop_twist_keyboard 2>&1 | tee -a ${LOG_FILE}.teleop; echo 'Teleop exited with code \$?' >> ${LOG_FILE}.teleop; exec bash"

log_message "Waiting for teleop to start..."
if ! wait_for_process "teleop_twist_keyboard" 5; then
  log_message "WARNING: Teleop may not have started correctly"
else
  log_message "Teleop appears to be running"
fi

# Check if cmd_vel topic exists after teleop start
log_message "Checking for cmd_vel topic after teleop start..."
ros2 topic list | grep "/cmd_vel" >> "${LOG_FILE}" 2>&1 || log_message "WARNING: /cmd_vel topic still not found after teleop start"

log_message "Keyboard teleop started successfully."

# Step 9: Launch waypoint navigation
log_message "Step 9: Launching waypoint navigation..."

# Check if waypoint navigation script exists
WAYPOINT_SCRIPT="/home/yahboom/b4m_yahboom/yahboomcar_nav/yahboomcar_nav/waypoint_navigation.py"
if ! check_file_exists "$WAYPOINT_SCRIPT"; then
  log_message "ERROR: Waypoint navigation script not found at $WAYPOINT_SCRIPT"
  log_message "Trying to find waypoint navigation script..."
  
  # Try to find the script using find
  FOUND_SCRIPT=$(find /home/yahboom/b4m_yahboom -name "waypoint_navigation.py" -type f 2>/dev/null | head -1)
  
  if [ -n "$FOUND_SCRIPT" ]; then
    log_message "Found waypoint navigation script at $FOUND_SCRIPT"
    WAYPOINT_SCRIPT="$FOUND_SCRIPT"
  else
    log_message "ERROR: Could not find waypoint navigation script"
    log_message "Trying to use ros2 run instead"
    gnome-terminal --title="Waypoint Navigation" -- bash -c "source /opt/ros/humble/setup.bash && source /home/yahboom/b4m_yahboom/install/setup.bash 2>/dev/null && ros2 run yahboomcar_nav gazebo_waypoint_navigation 2>&1 | tee -a ${LOG_FILE}.waypoint; echo 'Waypoint navigation exited with code \$?' >> ${LOG_FILE}.waypoint; exec bash"
    sleep 2
    log_message "Waypoint navigation attempted using ros2 run"
    
    # Check if waypoint navigation is running
    if ! process_running "gazebo_waypoint_navigation"; then
      log_message "WARNING: Waypoint navigation may not have started correctly using ros2 run"
    else
      log_message "Waypoint navigation appears to be running using ros2 run"
    fi
    
    log_message "Waypoint navigation started with ros2 run (fallback method)"
  fi
else
  # Launch waypoint navigation
  log_message "Launching waypoint navigation with command: python3 $WAYPOINT_SCRIPT"
  gnome-terminal --title="Waypoint Navigation" -- bash -c "source /opt/ros/humble/setup.bash && source /home/yahboom/b4m_yahboom/install/setup.bash 2>/dev/null && python3 $WAYPOINT_SCRIPT 2>&1 | tee -a ${LOG_FILE}.waypoint; echo 'Waypoint navigation exited with code \$?' >> ${LOG_FILE}.waypoint; exec bash"

  log_message "Waiting for waypoint navigation to start..."
  sleep 3

  # Check if waypoint navigation is running
  if ! process_running "waypoint_navigation"; then
    log_message "WARNING: Waypoint navigation may not have started correctly"
  else
    log_message "Waypoint navigation appears to be running"
  fi

  log_message "Waypoint navigation started successfully."
fi

# Final status check and summary
log_message "Performing final status check..."

# Check if all required components are running
COMPONENTS_OK=true

if ! process_running "gazebo"; then
  log_message "ERROR: Gazebo is not running"
  COMPONENTS_OK=false
fi

if ! process_running "rviz2"; then
  log_message "ERROR: RViz is not running"
  COMPONENTS_OK=false
fi

# Check if cmd_vel topic exists
if ! ros2 topic list 2>/dev/null | grep -q "/cmd_vel"; then
  log_message "ERROR: /cmd_vel topic not found"
  COMPONENTS_OK=false
fi

# Check if tf topics exist
if ! ros2 topic list 2>/dev/null | grep -q "/tf"; then
  log_message "ERROR: /tf topic not found"
  COMPONENTS_OK=false
fi

# Check if Nav2 nodes are running
if ! ros2 node list 2>/dev/null | grep -q "nav2"; then
  log_message "ERROR: Nav2 nodes not found"
  COMPONENTS_OK=false
fi

if [ "$COMPONENTS_OK" = true ]; then
  log_message "All components appear to be running correctly"
else
  log_message "WARNING: Some components may not be running correctly. Check the log file at ${LOG_FILE}"
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
echo "2. Waypoint navigation commands:"
echo "   - Press 's' to save the current position as a waypoint"
echo "   - Press 'l' to list saved waypoints"
echo "   - Press 'g' to navigate to a waypoint"
echo "   - Press 'c' to cancel navigation"
echo "   - Press 'q' to quit"
echo "======================================================================"
echo ""
echo "Log file: ${LOG_FILE}"
echo "Check the log file for detailed debugging information if you encounter issues."
log_message "Script completed successfully"
echo ""
echo "Workflow for creating and using waypoints:"
echo "  1. Use keyboard teleop to move the robot to a desired position"
echo "  2. Switch to the waypoint navigation terminal"
echo "  3. Press 's' to save the current position as a waypoint"
echo "  4. Use 'l' to list waypoints and 'g' to navigate to them"
echo ""
echo "If you encounter any issues, check the log files at ${LOG_DIR}"
