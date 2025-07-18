#!/bin/bash

# B4M Robot - Home Assistant MQTT Integration Launch Script
# This script automates the launch process for the B4M Robot with Home Assistant integration
# Each step will be launched in a separate terminal with user confirmation

# Get the workspace root directory (where this script is located)
WORKSPACE_ROOT=$(cd "$(dirname "$0")" && pwd)

# Function to ask for user confirmation
confirm() {
    echo ""
    read -p "Press Enter to continue to the next step or Ctrl+C to exit..."
    echo ""
}

# Function to launch a command in a new terminal
launch_in_terminal() {
    local description=$1
    local command=$2
    
    echo "====================================================="
    echo "STEP: $description"
    echo "====================================================="
    echo "Command to execute: $command"
    echo ""
    
    confirm
    
    # Launch the command in a new terminal using xterm with a basic font
    xterm -fn fixed -e bash -c "$command; echo 'Process completed. Press Enter to close this terminal...'; read" &
    
    # Give some time for the terminal to start
    sleep 2
}

echo "B4M Robot - Home Assistant MQTT Integration Launch Script"
echo "This script will guide you through launching all components of the B4M Robot system."
echo "Each step will open in a separate terminal window."
echo ""

# Step 1: Start the Micro-ROS Agent
launch_in_terminal "Starting the Micro-ROS Agent for ESP32 communication" \
    "docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:humble udp4 --port 8090"

# Step 2: Power on the Yahboom Robot
echo "====================================================="
echo "STEP: Power on the physical Yahboom Robot"
echo "====================================================="
echo "Please turn on the physical robot's power switch."
echo "Wait for the robot to boot up and connect to the Micro-ROS agent."
echo "You should see connection messages in the Micro-ROS agent terminal."
echo ""
confirm

# Step 3: Launch the Car's Underlying Data Processing
launch_in_terminal "Starting the car's underlying data processing for sensor integration" \
    "cd \"$WORKSPACE_ROOT\" && . install/setup.bash && ros2 launch yahboomcar_bringup yahboomcar_bringup_launch.py"

# Step 4: Start RViz for Visualization
launch_in_terminal "Starting RViz for visualization of robot state and environment" \
    "cd \"$WORKSPACE_ROOT\" && . install/setup.bash && ros2 launch yahboomcar_nav display_launch.py"

# Step 5: Launch the Navigation System
launch_in_terminal "Launching the navigation system with pre-built map" \
    "cd \"$WORKSPACE_ROOT\" && . install/setup.bash && ros2 launch yahboomcar_nav waypoint_navigation_launch.py maps:\"$WORKSPACE_ROOT/yahboomcar_nav/maps/yahboom_map.yaml\""

# Step 6: Initial Robot Positioning
echo "====================================================="
echo "STEP: Initial Robot Positioning"
echo "====================================================="
echo "IMPORTANT: The robot must be placed at the exact same starting position and orientation each time."
echo "1. Place the robot at the designated starting position in your environment"
echo "2. Use the '2D Pose Estimate' button in RViz to manually set the robot's initial pose"
echo "3. Verify in RViz that the red arrow (representing the robot) appears at the correct position on the map"
echo ""
confirm

# Step 7: Start the B4M Waypoint Navigation Node with MQTT Parameters
launch_in_terminal "Starting the B4M Waypoint Navigation Node with MQTT integration" \
    "cd \"$WORKSPACE_ROOT\" && . install/setup.bash && python3 \"$WORKSPACE_ROOT/b4m_waypoint_nav/b4m_waypoint_nav/b4m_waypoint_nav.py\" --ros-args -p mqtt_broker:=192.168.68.111 -p mqtt_port:=1883 -p mqtt_username:=robot -p mqtt_password:=robot123"

# Step 8: Start the Robot Manager GUI
echo "===================================================="
echo "STEP: Starting the B4M Robot Manager GUI"
echo "===================================================="
echo "IMPORTANT: This will launch the robot manager GUI for visual control of the robot."
echo ""
confirm

launch_in_terminal "Starting the B4M Robot Manager GUI for visual control of waypoints" \
    "cd \"$WORKSPACE_ROOT\" && . install/setup.bash && ros2 run b4m_waypoint_nav b4m_robot_manager_node.py"


exit 0
