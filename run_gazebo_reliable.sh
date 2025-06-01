#!/bin/bash

# This script launches the Yahboom Gazebo simulation with all necessary components
# in a reliable manner with proper environment setup.

# Clean up any existing processes - use pkill without -9 first to allow graceful shutdown
echo "Cleaning up existing processes..."
pkill -f gazebo || true
pkill -f gzserver || true
pkill -f gzclient || true
pkill -f rviz2 || true
pkill -f "ros2 run tf2_ros static_transform_publisher" || true
pkill -f "ros2 launch nav2_bringup" || true
pkill -f "teleop_twist_keyboard" || true
sleep 2

# Source ROS2 and workspace
source /opt/ros/humble/setup.bash
source /home/yahboom/b4m_yahboom/install/setup.bash 2>/dev/null

# Export Gazebo model path
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/yahboom/b4m_yahboom/yahboomcar_description/models

# Step 1: Start Gazebo server with ROS2 plugins
echo "Step 1: Starting Gazebo server with ROS2 plugins..."
gnome-terminal --title="Gazebo" -- bash -c "source /opt/ros/humble/setup.bash && source /home/yahboom/b4m_yahboom/install/setup.bash 2>/dev/null && gazebo --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so; exec bash"
sleep 5

# Step 1.4: Create fixed URDF file without encoding declaration and with Gazebo plugins
echo "Step 1.4: Creating fixed URDF file with Gazebo plugins..."
cat /home/yahboom/b4m_yahboom/yahboomcar_description/urdf/yahboomcar_robot2.urdf | grep -v 'encoding="utf-8"' | sed 's|package://yahboomcar_description/meshes/|/home/yahboom/b4m_yahboom/yahboomcar_description/meshes/|g' > /home/yahboom/b4m_yahboom/yahboomcar_description/urdf/yahboomcar_robot2_fixed.urdf

# Step 1.5: Spawn the robot model in Gazebo
echo "Step 1.5: Spawning robot model in Gazebo..."
gnome-terminal --title="Robot Spawn" -- bash -c "source /opt/ros/humble/setup.bash && source /home/yahboom/b4m_yahboom/install/setup.bash 2>/dev/null && ros2 run gazebo_ros spawn_entity.py -file /home/yahboom/b4m_yahboom/yahboomcar_description/urdf/yahboomcar_robot2_gazebo.urdf -entity yahboomcar -x 0 -y 0 -z 0.1; exec bash"
sleep 3

# Step 2: Add necessary transforms
echo "Step 2: Adding necessary transforms..."
gnome-terminal --title="Map to Odom TF" -- bash -c "source /opt/ros/humble/setup.bash && source /home/yahboom/b4m_yahboom/install/setup.bash 2>/dev/null && ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom; exec bash"
sleep 1
gnome-terminal --title="Odom to Base Link TF" -- bash -c "source /opt/ros/humble/setup.bash && source /home/yahboom/b4m_yahboom/install/setup.bash 2>/dev/null && ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom robot2/base_link; exec bash"
sleep 1
gnome-terminal --title="Base Link to Footprint TF" -- bash -c "source /opt/ros/humble/setup.bash && source /home/yahboom/b4m_yahboom/install/setup.bash 2>/dev/null && ros2 run tf2_ros static_transform_publisher 0 0 0.05 0 0 0 robot2/base_link robot2/base_footprint; exec bash"
sleep 1

# Step 3: Start RViz for visualization
echo "Step 3: Starting RViz for visualization..."
gnome-terminal --title="RViz" -- bash -c "source /opt/ros/humble/setup.bash && source /home/yahboom/b4m_yahboom/install/setup.bash 2>/dev/null && rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz; exec bash"
sleep 3

# Step 4: Launch Nav2 with AMCL
echo "Step 4: Launching Nav2 with AMCL..."
gnome-terminal --title="Nav2 Components" -- bash -c "source /opt/ros/humble/setup.bash && source /home/yahboom/b4m_yahboom/install/setup.bash 2>/dev/null && ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true map_subscribe_transient_local:=true autostart:=true; exec bash"
sleep 5

# Step 5: Remind about setting initial pose in RViz (critical step)
echo ""
echo "=================================================================================="
echo "CRITICAL STEP: You must set the initial pose in RViz using the 2D Pose Estimate tool."
echo "This is essential for AMCL to start publishing the map->base_link transform."
echo "1. In RViz, click on the '2D Pose Estimate' button in the toolbar"
echo "2. Click and drag on the map to set the robot's position and orientation"
echo "=================================================================================="
echo ""
echo "Press ENTER after you have set the initial pose in RViz..."
read

# Step 6: Launch keyboard teleop for robot control
echo "Step 6: Launching keyboard teleop for robot control..."
gnome-terminal --title="Keyboard Teleop" -- bash -c "source /opt/ros/humble/setup.bash && source /home/yahboom/b4m_yahboom/install/setup.bash 2>/dev/null && ros2 run teleop_twist_keyboard teleop_twist_keyboard; exec bash"
sleep 2

# Step 7: Launch waypoint navigation
echo "Step 7: Launching waypoint navigation..."
gnome-terminal --title="Waypoint Navigation" -- bash -c "source /opt/ros/humble/setup.bash && source /home/yahboom/b4m_yahboom/install/setup.bash 2>/dev/null && python3 /home/yahboom/b4m_yahboom/yahboomcar_nav/yahboomcar_nav/waypoint_navigation.py; exec bash"

echo "====================================================================="
echo "Yahboom Gazebo simulation is now running!"
echo "====================================================================="
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
echo "====================================================================="

echo "=================================================================================="
echo ""
echo "=================================================================================="
echo "SETUP COMPLETE! The Gazebo simulation is now running with all components."
echo "=================================================================================="
echo ""
echo "Keyboard Teleop Controls (make sure the Keyboard Teleop terminal is active):"
echo "  w - Move forward"
echo "  s - Move backward"
echo "  a - Turn left"
echo "  d - Turn right"
echo "  q/z - Turn left while moving forward"
echo "  e/c - Turn right while moving forward"
echo "  i/j/k/l - Alternative movement controls"
echo "  u/o/m/. - Alternative turning controls"
echo "  Space - Stop the robot"
echo "  b - Toggle emergency brake"
echo "  x - Exit keyboard control"
echo ""
echo "Waypoint Navigation Commands (make sure the Waypoint Navigation terminal is active):"
echo "  s - Save current position as a waypoint"
echo "  l - List all saved waypoints"
echo "  g - Go to a waypoint (you'll be prompted for the number)"
echo "  c - Cancel current navigation"
echo "  q - Quit the program"
echo ""
echo "Workflow for creating and using waypoints:"
echo "  1. Use keyboard teleop to move the robot to a desired position"
echo "  2. Switch to the waypoint navigation terminal"
echo "  3. Press 's' to save the current position as a waypoint"
echo "  4. Repeat steps 1-3 for multiple waypoints"
echo "  5. Press 'l' to list saved waypoints"
echo "  6. Press 'g' to navigate to a specific waypoint"
echo ""
echo "To stop all components, run: pkill -9 -f gazebo && pkill -9 -f rviz2 && pkill -9 -f 'ros2 run tf2_ros static_transform_publisher'"
echo ""
echo "The script has completed. All components are running in separate terminals."
