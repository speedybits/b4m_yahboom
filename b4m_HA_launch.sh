#!/bin/bash

# B4M Robot - Home Assistant MQTT Integration Launch Script
# This script automates the launch process for the B4M Robot with Home Assistant integration
# Each step will be launched in a separate terminal with user confirmation
#
# Usage: ./b4m_HA_launch.sh [--skip-agent]
#   --skip-agent: Skip the Micro-ROS agent launch (Step 1)

# Parse command line arguments
SKIP_AGENT=false
for arg in "$@"; do
    case $arg in
        --skip-agent)
            SKIP_AGENT=true
            shift
            ;;
        -h|--help)
            echo "Usage: $0 [--skip-agent]"
            echo "  --skip-agent: Skip the Micro-ROS agent launch (Step 1)"
            exit 0
            ;;
        *)
            echo "Unknown argument: $arg"
            echo "Use -h or --help for usage information"
            exit 1
            ;;
    esac
done

# Get the workspace root directory (where this script is located)
WORKSPACE_ROOT=$(cd "$(dirname "$0")" && pwd)

# Create logs directory if it doesn't exist
LOGS_DIR="$WORKSPACE_ROOT/logs"
mkdir -p "$LOGS_DIR"

# Generate timestamp for log files
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
MAIN_LOG="$LOGS_DIR/b4m_launch_$TIMESTAMP.log"

# Function to ask for user confirmation
confirm() {
    echo ""
    read -p "Press Enter to continue to the next step or Ctrl+C to exit..."
    echo ""
}

# Function to log messages
log_message() {
    local message=$1
    echo "$(date '+%Y-%m-%d %H:%M:%S') - $message" | tee -a "$MAIN_LOG"
}

# Function to launch a command in a new terminal with logging
launch_in_terminal() {
    local description=$1
    local command=$2
    local step_num=$3
    
    # Create step-specific log file
    local step_log="$LOGS_DIR/step${step_num}_$(echo "$description" | tr ' ' '_' | tr '[:upper:]' '[:lower:]')_$TIMESTAMP.log"
    
    echo "====================================================="
    echo "STEP $step_num: $description"
    echo "====================================================="
    echo "Command to execute:"
    echo "  $command"
    echo ""
    
    # Ask for confirmation before proceeding
    echo "Press ENTER to execute this command, or Ctrl+C to exit..."
    read
    
    echo ""
    echo "📁 Log file location: $step_log"
    echo "🚀 Launching step $step_num in new terminal..."
    echo ""
    
    log_message "STEP $step_num: Starting terminal for $description"
    log_message "Log file: $step_log"
    
    # Create a temporary script for this step
    local temp_script="/tmp/b4m_step${step_num}_$$"
    cat > "$temp_script" << EOF
#!/bin/bash

# Store the command to execute
COMMAND='$command'

# Terminal title
echo -e "\033]0;B4M Step $step_num: $description\007"

echo "======================================="
echo "B4M Robot Launch - Step $step_num"
echo "======================================="
echo "Description: $description"
echo "Log file: $step_log"
echo "Started at: \$(date)"
echo "======================================="
echo ""

# Log the start
echo "Starting: $description" | tee "$step_log"
echo "Command: \$COMMAND" | tee -a "$step_log"
echo "Started at: \$(date)" | tee -a "$step_log"
echo "=====================================" | tee -a "$step_log"

# Source workspace before running command
cd "$WORKSPACE_ROOT"
if [ -f 'install/setup.bash' ]; then
    source install/setup.bash
    echo "✅ Workspace sourced successfully" | tee -a "$step_log"
else
    echo "⚠️  WARNING: install/setup.bash not found!" | tee -a "$step_log"
fi

echo ""
echo "🚀 Executing command..."
echo ""

# Execute the command with better error handling
set +e  # Don't exit on error
eval "\$COMMAND" 2>&1 | tee -a "$step_log"
COMMAND_EXIT_CODE=\$?
set -e

echo ""
echo "=====================================" | tee -a "$step_log"
if [ \$COMMAND_EXIT_CODE -eq 0 ]; then
    echo "✅ Process completed successfully at: \$(date)" | tee -a "$step_log"
else
    echo "❌ Process failed with exit code \$COMMAND_EXIT_CODE at: \$(date)" | tee -a "$step_log"
fi
echo "Log saved to: $step_log" | tee -a "$step_log"
echo "=====================================" | tee -a "$step_log"

echo ""
echo "Step $step_num terminal will remain open for debugging."
echo "Log file: $step_log"
echo ""
echo "Commands you can run:"
echo "  - 'tail -f $step_log' to monitor the log"
echo "  - 'ros2 node list' to check active nodes"
echo "  - 'ros2 topic list' to check active topics"
echo ""
echo "Press Enter to close this terminal, or run additional commands..."

# Keep terminal open
exec bash

EOF
    
    chmod +x "$temp_script"
    
    # Launch the temporary script in a new terminal
    xterm -fn fixed -e "$temp_script" &
    
    # Give some time for the terminal to start
    sleep 2
    
    log_message "STEP $step_num terminal launched"
}

echo "B4M Robot - Home Assistant MQTT Integration Launch Script"
echo "This script will guide you through launching all components of the B4M Robot system."
echo "Each step will open in a separate terminal window."
echo "Logs will be saved to: $LOGS_DIR"
echo "Main log file: $MAIN_LOG"

if [ "$SKIP_AGENT" = true ]; then
    echo ""
    echo "⏭️  Skipping Micro-ROS agent launch (--skip-agent specified)"
fi

echo ""

log_message "B4M Robot launch script started"

# Step 1: Start the Micro-ROS Agent (unless skipped)
if [ "$SKIP_AGENT" = false ]; then
    launch_in_terminal "Starting the Micro-ROS Agent for ESP32 communication" \
        "docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:humble udp4 --port 8090" \
        "1"
else
    log_message "STEP 1: Skipped Micro-ROS agent launch"
fi

# Step 2: Power on the Yahboom Robot
echo "====================================================="
echo "STEP 2: Power on the physical Yahboom Robot"
echo "====================================================="
echo "Manual step required:"
echo "  1. Turn on the physical robot's power switch"
echo "  2. Wait for the robot to boot up and connect to the Micro-ROS agent"
echo "  3. Check for connection messages in the Micro-ROS agent terminal"
echo ""

log_message "STEP 2: Waiting for physical robot power on"

echo "Press ENTER when the robot is powered on and connected..."
read

log_message "STEP 2: Physical robot power on confirmed"

# Step 3: Launch the Car's Underlying Data Processing
launch_in_terminal "Starting the car's underlying data processing for sensor integration" \
    "cd \"$WORKSPACE_ROOT\" && . install/setup.bash && ros2 launch yahboomcar_bringup yahboomcar_bringup_launch.py" \
    "3"

# Step 4: Start RViz for Visualization
launch_in_terminal "Starting RViz for visualization of robot state and environment" \
    "cd \"$WORKSPACE_ROOT\" && . install/setup.bash && ros2 launch yahboomcar_nav display_launch.py" \
    "4"

# Step 5: Launch the Navigation System
launch_in_terminal "Launching the navigation system with pre-built map" \
    "cd \"$WORKSPACE_ROOT\" && . install/setup.bash && ros2 launch yahboomcar_nav waypoint_navigation_launch.py maps:=\"$WORKSPACE_ROOT/yahboomcar_nav/maps/yahboom_map.yaml\"" \
    "5"

# Step 6: Initial Robot Positioning
echo "====================================================="
echo "STEP 6: Initial Robot Positioning"
echo "====================================================="
echo "IMPORTANT: The robot must be placed at the exact same starting position and orientation each time."
echo ""
echo "✅ Prerequisites (should be working now):"
echo "   - Step 3 (bringup) running → EKF publishing odom frame"
echo "   - Step 4 (RViz) running → Visualization ready"
echo "   - Step 5 (navigation) running → AMCL localization active"
echo ""
echo "📍 Positioning steps:"
echo "1. Place the robot at the designated starting position in your environment"
echo "2. In RViz, click the '2D Pose Estimate' button (arrow icon in toolbar)"
echo "3. Click and drag on the map to set robot position and orientation"
echo "4. Verify the red arrow (robot) appears at the correct position on the map"
echo "5. Check that laser scan data aligns with map obstacles"
echo ""
echo "🔧 Troubleshooting:"
echo "   - If 2D Pose Estimate doesn't work: Check Step 3 terminal is still running"
echo "   - If no laser data: Check MicroROS agent connection (Step 1)"
echo "   - If robot doesn't appear: Check transform chain in RViz TF display"
echo ""

log_message "STEP 6: Waiting for robot positioning"
confirm
log_message "STEP 6: Robot positioning confirmed"

# Step 7: Start the B4M Waypoint Navigation Node with MQTT Parameters
launch_in_terminal "Starting the B4M Waypoint Navigation Node with MQTT integration" \
    "cd \"$WORKSPACE_ROOT\" && . install/setup.bash && python3 \"$WORKSPACE_ROOT/b4m_waypoint_nav/b4m_waypoint_nav/b4m_waypoint_nav.py\" --ros-args -p mqtt_broker:=192.168.68.111 -p mqtt_port:=1883 -p mqtt_username:=robot -p mqtt_password:=robot123" \
    "7"

# Step 8: Start the Robot Manager GUI
launch_in_terminal "Starting the B4M Robot Manager GUI for visual control of waypoints" \
    "cd \"$WORKSPACE_ROOT\" && . install/setup.bash && ros2 run b4m_waypoint_nav b4m_robot_manager_node.py" \
    "8"


log_message "B4M Robot launch script completed"
echo "====================================================="
echo "Launch script completed successfully!"
echo "All logs are saved in: $LOGS_DIR"
echo "Main log file: $MAIN_LOG"
echo "====================================================="

exit 0
