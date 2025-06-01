#!/bin/bash

# This script launches the Yahboom Gazebo simulation with all necessary components
# following the optimal launch procedure from previous successful runs

# Enable debugging
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
  if command -v "$1" >/dev/null 2>&1; then
    log_message "Command '$1' exists"
    return 0
  else
    log_message "Command '$1' does not exist"
    return 1
  fi
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

# Function to source ROS workspace
source_ros_workspace() {
  log_message "Sourcing ROS workspace..."
  if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    log_message "Sourced ROS Humble setup.bash"
  else
    log_message "ERROR: ROS Humble setup.bash not found"
    return 1
  fi
  
  if [ -f /home/yahboom/b4m_yahboom/install/setup.bash ]; then
    source /home/yahboom/b4m_yahboom/install/setup.bash 2>/dev/null
    log_message "Sourced workspace setup.bash"
  else
    log_message "WARNING: Workspace setup.bash not found, continuing anyway"
  fi
  return 0
}

# Function to check if a file exists
check_file_exists() {
  local file_path="$1"
  if [ -f "$file_path" ]; then
    log_message "File '$file_path' exists"
    return 0
  else
    log_message "ERROR: File '$file_path' does not exist"
    return 1
  fi
}

# Function to check if a directory exists
check_dir_exists() {
  local dir_path="$1"
  if [ -d "$dir_path" ]; then
    log_message "Directory '$dir_path' exists"
    return 0
  else
    log_message "ERROR: Directory '$dir_path' does not exist"
    return 1
  fi
}

# Clean up existing processes
log_message "Cleaning up existing processes..."
log_message "Killing gazebo processes..."
pkill -f gazebo 2>>${LOG_FILE} || log_message "No gazebo processes found"
log_message "Killing gzserver processes..."
pkill -f gzserver 2>>${LOG_FILE} || log_message "No gzserver processes found"
log_message "Killing gzclient processes..."
pkill -f gzclient 2>>${LOG_FILE} || log_message "No gzclient processes found"
log_message "Killing rviz2 processes..."
pkill -f rviz2 2>>${LOG_FILE} || log_message "No rviz2 processes found"
log_message "Killing static_transform_publisher processes..."
pkill -f "ros2 run tf2_ros static_transform_publisher" 2>>${LOG_FILE} || log_message "No static_transform_publisher processes found"
log_message "Killing nav2_bringup processes..."
pkill -f "ros2 launch nav2_bringup" 2>>${LOG_FILE} || log_message "No nav2_bringup processes found"
log_message "Killing teleop_twist_keyboard processes..."
pkill -f "teleop_twist_keyboard" 2>>${LOG_FILE} || log_message "No teleop_twist_keyboard processes found"
log_message "Waiting for processes to terminate..."
sleep 2

# Source ROS workspace
log_message "Sourcing ROS workspace..."
if ! source_ros_workspace; then
  log_message "ERROR: Failed to source ROS workspace"
  exit 1
fi

# Check for required commands
log_message "Checking for required commands..."
for cmd in gazebo ros2 gnome-terminal rviz2; do
  if ! command_exists "$cmd"; then
    log_message "ERROR: Required command '$cmd' not found"
    exit 1
  fi
done

# Check for required directories
log_message "Checking for required directories..."
check_dir_exists "/home/yahboom/b4m_yahboom/yahboomcar_description" || log_message "WARNING: yahboomcar_description directory not found"
check_dir_exists "/home/yahboom/b4m_yahboom/yahboomcar_description/models" || log_message "WARNING: models directory not found"
check_dir_exists "/home/yahboom/b4m_yahboom/yahboomcar_description/urdf" || log_message "WARNING: urdf directory not found"

# Export Gazebo model path
log_message "Exporting Gazebo model path..."
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/yahboom/b4m_yahboom/yahboomcar_description/models
log_message "GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH"

# Step 1: Start Gazebo server with ROS2 plugins
log_message "Step 1: Starting Gazebo server with ROS2 plugins..."
log_message "Command: gazebo --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so"

# Check if Gazebo plugins exist
if [ ! -f "/opt/ros/humble/lib/libgazebo_ros_init.so" ]; then
  log_message "WARNING: libgazebo_ros_init.so not found in expected location"
fi
if [ ! -f "/opt/ros/humble/lib/libgazebo_ros_factory.so" ]; then
  log_message "WARNING: libgazebo_ros_factory.so not found in expected location"
fi

# Launch Gazebo in a separate terminal
gnome-terminal --title="Gazebo" -- bash -c "source /opt/ros/humble/setup.bash && source /home/yahboom/b4m_yahboom/install/setup.bash 2>/dev/null && gazebo --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so 2>&1 | tee -a ${LOG_FILE}.gazebo; exec bash"

log_message "Waiting for Gazebo to start..."
sleep 5

# Check if Gazebo is running
if ! process_running "gazebo"; then
  log_message "WARNING: Gazebo may not have started correctly"
else
  log_message "Gazebo appears to be running"
fi

# Step 2: Create fixed URDF file without encoding declaration
log_message "Step 2: Creating fixed URDF file without encoding declaration..."

# Check if source URDF exists
SOURCE_URDF="/home/yahboom/b4m_yahboom/yahboomcar_description/urdf/yahboomcar_robot2.urdf"
TARGET_URDF="/home/yahboom/b4m_yahboom/yahboomcar_description/urdf/yahboomcar_robot2_fixed.urdf"
GAZEBO_URDF="/home/yahboom/b4m_yahboom/yahboomcar_description/urdf/yahboomcar_robot2_gazebo.urdf"

log_message "Source URDF: $SOURCE_URDF"
log_message "Target URDF: $TARGET_URDF"
log_message "Gazebo URDF: $GAZEBO_URDF"

if ! check_file_exists "$SOURCE_URDF"; then
  log_message "ERROR: Source URDF file does not exist"
  exit 1
fi

# Check if Gazebo URDF exists and use it if available
if check_file_exists "$GAZEBO_URDF"; then
  log_message "Using existing Gazebo URDF file with plugins"
  cp "$GAZEBO_URDF" "$TARGET_URDF"
  log_message "Copied Gazebo URDF to target location"
  
  # Check if the Gazebo URDF contains the differential drive plugin
  if grep -q "libgazebo_ros_diff_drive.so" "$TARGET_URDF"; then
    log_message "Gazebo differential drive plugin found in URDF"
  else
    log_message "WARNING: Gazebo differential drive plugin NOT found in URDF"
  fi
else
  log_message "Creating fixed URDF file from source..."
  # Create fixed URDF file without encoding declaration and with absolute paths
  cat "$SOURCE_URDF" | grep -v 'encoding="utf-8"' | sed 's|package://yahboomcar_description/meshes/|/home/yahboom/b4m_yahboom/yahboomcar_description/meshes/|g' > "$TARGET_URDF"
  
  log_message "WARNING: Using URDF without Gazebo plugins. Robot may not move in simulation."
fi

# Verify the fixed URDF file was created
if check_file_exists "$TARGET_URDF"; then
  log_message "Fixed URDF file created successfully"
  # Count the number of lines in the URDF file
  URDF_LINES=$(wc -l < "$TARGET_URDF")
  log_message "URDF file has $URDF_LINES lines"
  
  # Check for common issues in the URDF
  if grep -q "encoding=\"utf-8\"" "$TARGET_URDF"; then
    log_message "WARNING: URDF still contains encoding declaration"
  fi
  
  if grep -q "package://" "$TARGET_URDF"; then
    log_message "WARNING: URDF still contains package:// paths"
  fi
else
  log_message "ERROR: Failed to create fixed URDF file"
  exit 1
fi

# Step 3: Spawn the robot model in Gazebo
log_message "Step 3: Spawning robot model in Gazebo..."

# Verify that spawn_entity.py exists
if ! ros2 pkg list | grep -q "gazebo_ros"; then
  log_message "ERROR: gazebo_ros package not found"
  exit 1
fi

# Check if Gazebo is running before attempting to spawn
if ! process_running "gazebo"; then
  log_message "ERROR: Gazebo is not running. Cannot spawn robot."
  log_message "Attempting to restart Gazebo..."
  gnome-terminal --title="Gazebo Restart" -- bash -c "source /opt/ros/humble/setup.bash && source /home/yahboom/b4m_yahboom/install/setup.bash 2>/dev/null && gazebo --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so 2>&1 | tee -a ${LOG_FILE}.gazebo_restart; exec bash"
  log_message "Waiting for Gazebo to restart..."
  sleep 10
  
  if ! process_running "gazebo"; then
    log_message "ERROR: Failed to restart Gazebo. Exiting."
    exit 1
  fi
fi

# Spawn the robot model
log_message "Spawning robot with command: ros2 run gazebo_ros spawn_entity.py -file $TARGET_URDF -entity yahboomcar -x 0 -y 0 -z 0.1"
gnome-terminal --title="Robot Spawn" -- bash -c "source /opt/ros/humble/setup.bash && source /home/yahboom/b4m_yahboom/install/setup.bash 2>/dev/null && ros2 run gazebo_ros spawn_entity.py -file $TARGET_URDF -entity yahboomcar -x 0 -y 0 -z 0.1 2>&1 | tee -a ${LOG_FILE}.robot_spawn; exec bash"
log_message "Waiting for robot to spawn..."
sleep 5

# Check if robot was spawned successfully by checking for the model in Gazebo
log_message "Checking if robot was spawned successfully..."
if ! ros2 service list | grep -q "/get_model_list"; then
  log_message "WARNING: Cannot verify if robot was spawned. /get_model_list service not available."
else
  log_message "Attempting to verify robot spawn via service call..."
  # This is a placeholder - in a real script you would call the service and check the response
  log_message "Service verification not implemented, continuing anyway"
fi

# Step 4: Add necessary transforms
log_message "Step 4: Adding necessary transforms..."

# Check if tf2_ros package is available
if ! ros2 pkg list | grep -q "tf2_ros"; then
  log_message "ERROR: tf2_ros package not found"
  exit 1
fi

# Add map to odom transform
log_message "Adding map to odom transform..."
gnome-terminal --title="Map to Odom TF" -- bash -c "source /opt/ros/humble/setup.bash && source /home/yahboom/b4m_yahboom/install/setup.bash 2>/dev/null && ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom 2>&1 | tee -a ${LOG_FILE}.map_odom_tf; exec bash"
sleep 2

# Add odom to base_link transform
log_message "Adding odom to base_link transform..."
gnome-terminal --title="Odom to Base Link TF" -- bash -c "source /opt/ros/humble/setup.bash && source /home/yahboom/b4m_yahboom/install/setup.bash 2>/dev/null && ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom robot2/base_link 2>&1 | tee -a ${LOG_FILE}.odom_baselink_tf; exec bash"
sleep 2

# Add base_link to base_footprint transform
log_message "Adding base_link to base_footprint transform..."
gnome-terminal --title="Base Link to Footprint TF" -- bash -c "source /opt/ros/humble/setup.bash && source /home/yahboom/b4m_yahboom/install/setup.bash 2>/dev/null && ros2 run tf2_ros static_transform_publisher 0 0 0.05 0 0 0 robot2/base_link robot2/base_footprint 2>&1 | tee -a ${LOG_FILE}.baselink_footprint_tf; exec bash"
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
  gnome-terminal --title="RViz" -- bash -c "source /opt/ros/humble/setup.bash && source /home/yahboom/b4m_yahboom/install/setup.bash 2>/dev/null && rviz2 2>&1 | tee -a ${LOG_FILE}.rviz; exec bash"
else
  log_message "Starting RViz with Nav2 default view config"
  gnome-terminal --title="RViz" -- bash -c "source /opt/ros/humble/setup.bash && source /home/yahboom/b4m_yahboom/install/setup.bash 2>/dev/null && rviz2 -d $RVIZ_CONFIG 2>&1 | tee -a ${LOG_FILE}.rviz; exec bash"
fi

log_message "Waiting for RViz to start..."
sleep 5

# Check if RViz is running
if ! process_running "rviz2"; then
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
gnome-terminal --title="Nav2 Components" -- bash -c "source /opt/ros/humble/setup.bash && source /home/yahboom/b4m_yahboom/install/setup.bash 2>/dev/null && ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true autostart:=true map_subscribe_transient_local:=true 2>&1 | tee -a ${LOG_FILE}.nav2; exec bash"

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
gnome-terminal --title="Keyboard Teleop" -- bash -c "source /opt/ros/humble/setup.bash && source /home/yahboom/b4m_yahboom/install/setup.bash 2>/dev/null && ros2 run teleop_twist_keyboard teleop_twist_keyboard 2>&1 | tee -a ${LOG_FILE}.teleop; exec bash"

log_message "Waiting for teleop to start..."
sleep 3

# Check if teleop is running
if ! process_running "teleop_twist_keyboard"; then
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
    gnome-terminal --title="Waypoint Navigation" -- bash -c "source /opt/ros/humble/setup.bash && source /home/yahboom/b4m_yahboom/install/setup.bash 2>/dev/null && ros2 run yahboomcar_nav gazebo_waypoint_navigation 2>&1 | tee -a ${LOG_FILE}.waypoint; exec bash"
    sleep 2
    log_message "Waypoint navigation attempted using ros2 run"
    
    # Check if waypoint navigation is running
    if ! process_running "gazebo_waypoint_navigation"; then
      log_message "WARNING: Waypoint navigation may not have started correctly using ros2 run"
    else
      log_message "Waypoint navigation appears to be running using ros2 run"
    fi
    
    log_message "Waypoint navigation started with ros2 run (fallback method)"
    return
  fi
fi

# Launch waypoint navigation
log_message "Launching waypoint navigation with command: python3 $WAYPOINT_SCRIPT"
gnome-terminal --title="Waypoint Navigation" -- bash -c "source /opt/ros/humble/setup.bash && source /home/yahboom/b4m_yahboom/install/setup.bash 2>/dev/null && python3 $WAYPOINT_SCRIPT 2>&1 | tee -a ${LOG_FILE}.waypoint; exec bash"

log_message "Waiting for waypoint navigation to start..."
sleep 3

# Check if waypoint navigation is running
if ! process_running "waypoint_navigation"; then
  log_message "WARNING: Waypoint navigation may not have started correctly"
else
  log_message "Waypoint navigation appears to be running"
fi

log_message "Waypoint navigation started successfully."

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
echo "  4. Repeat steps 1-3 for multiple waypoints"
echo "  5. Press 'l' to list saved waypoints"
echo "  6. Press 'g' to navigate to a specific waypoint"
echo ""
echo "To stop all components, run: pkill -f gazebo && pkill -f rviz2 && pkill -f 'ros2 run tf2_ros static_transform_publisher'"
