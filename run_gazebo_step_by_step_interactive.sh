#!/bin/bash

# Interactive step-by-step script for launching Gazebo simulation
# This script will launch components one by one with user confirmation

# Source ROS environment and workspace
source /opt/ros/humble/setup.bash
source /home/yahboom/b4m_yahboom/install/setup.bash

# Set Gazebo model path to include Yahboom models
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/yahboom/b4m_yahboom/yahboomcar_description

# Function to prompt user for confirmation
confirm() {
    echo ""
    echo "Press ENTER to continue to the next step or Ctrl+C to exit..."
    read
}

# Clean up existing processes
echo "Step 0: Cleaning up existing processes..."
pkill -9 -f gazebo || true
pkill -9 -f gzserver || true
pkill -9 -f gzclient || true
pkill -9 -f rviz2 || true
pkill -9 -f "ros2 run tf2_ros static_transform_publisher" || true
pkill -9 -f "python3 /home/yahboom/b4m_yahboom/yahboomcar_nav/yahboomcar_nav/waypoint_navigation.py" || true
sleep 2
echo "✓ Cleanup complete"

# Step 1: Start Gazebo
echo ""
echo "Step 1: Start Gazebo"
echo "This will launch the Gazebo simulator."
confirm
gnome-terminal --title="Gazebo" -- bash -c "source /opt/ros/humble/setup.bash && gazebo --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so; exec bash"
echo "✓ Gazebo started"

# Step 2: Start RViz
echo ""
echo "Step 2: Start RViz"
echo "This will launch RViz for visualization."
confirm
gnome-terminal --title="RViz" -- bash -c "source /opt/ros/humble/setup.bash && ros2 launch yahboomcar_nav/launch/display_launch.py; exec bash"
echo "✓ RViz started"

# Step 3: Add necessary transforms
echo ""
echo "Step 3: Add necessary transforms"
echo "This will add the odom->base_link and base_link->base_footprint transforms."
confirm
gnome-terminal --title="TF Publisher (odom->base_link)" -- bash -c "source /opt/ros/humble/setup.bash && ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link; exec bash"
echo "✓ odom->base_link transform added"
sleep 1
gnome-terminal --title="TF Publisher (base_link->base_footprint)" -- bash -c "source /opt/ros/humble/setup.bash && ros2 run tf2_ros static_transform_publisher 0 0 0.05 0 0 0 base_link base_footprint; exec bash"
echo "✓ base_link->base_footprint transform added"

# Step 4: Launch Nav2 with AMCL
echo ""
echo "Step 4: Launch Nav2 with AMCL"
echo "This will start the Navigation2 stack with AMCL for localization."
confirm
gnome-terminal --title="Nav2" -- bash -c "source /opt/ros/humble/setup.bash && ros2 launch nav2_bringup bringup_launch.py map:=/home/yahboom/b4m_yahboom/yahboomcar_nav/maps/yahboom_map.yaml use_sim_time:=true; exec bash"
echo "✓ Nav2 with AMCL started"

# Step 5: Set initial pose in RViz
echo ""
echo "Step 5: Set initial pose in RViz"
echo "IMPORTANT: You must set the initial pose in RViz using the 2D Pose Estimate tool."
echo "This is a critical step for AMCL to start publishing the map->base_link transform."
echo "1. In RViz, click on the '2D Pose Estimate' button in the toolbar"
echo "2. Click and drag on the map to set the robot's position and orientation"
confirm

# Step 6: Launch waypoint navigation
echo ""
echo "Step 6: Launch waypoint navigation"
echo "This will start the waypoint navigation system."
confirm
gnome-terminal --title="Waypoint Navigation" -- bash -c "source /opt/ros/humble/setup.bash && python3 /home/yahboom/b4m_yahboom/yahboomcar_nav/yahboomcar_nav/waypoint_navigation.py; exec bash"
echo "✓ Waypoint navigation started"

# Final instructions
echo ""
echo "=================================================================================="
echo "SETUP COMPLETE! The Gazebo simulation is now running with all components."
echo "=================================================================================="
echo ""
echo "Waypoint Navigation Commands:"
echo "  s - Save current position as a waypoint"
echo "  l - List all saved waypoints"
echo "  g - Go to a waypoint (you'll be prompted for the number)"
echo "  c - Cancel current navigation"
echo "  q - Quit the program"
echo ""
echo "To stop all components, run: pkill -9 -f gazebo && pkill -9 -f rviz2 && pkill -9 -f 'ros2 run tf2_ros static_transform_publisher'"
echo ""
echo "The script has completed. All components are running in separate terminals."
