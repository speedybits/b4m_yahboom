#!/bin/bash

# This script provides step-by-step instructions for launching the Yahboom Gazebo simulation
# You will need to press ENTER after each step to proceed to the next one

# Function to wait for user confirmation
wait_for_user() {
  echo ""
  echo "Press ENTER to continue to the next step..."
  read
}

# Clean up existing processes
echo "Step 0: Cleaning up existing processes..."
pkill -f gazebo || true
pkill -f gzserver || true
pkill -f gzclient || true
pkill -f rviz2 || true
pkill -f "ros2 run tf2_ros static_transform_publisher" || true
pkill -f "ros2 launch nav2_bringup" || true
pkill -f "teleop_twist_keyboard" || true
sleep 2
echo "Cleanup complete."
wait_for_user

# Source ROS workspace
echo "Step 1: Sourcing ROS workspace..."
source /opt/ros/humble/setup.bash
source /home/yahboom/b4m_yahboom/install/setup.bash 2>/dev/null || true
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/yahboom/b4m_yahboom/yahboomcar_description/models
echo "ROS workspace sourced successfully."
wait_for_user

# Start Gazebo server
echo "Step 2: Starting Gazebo server with ROS2 plugins..."
echo "Running command: gazebo --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so"
echo "Opening a new terminal window for Gazebo..."
gnome-terminal --title="Gazebo" -- bash -c "source /opt/ros/humble/setup.bash && source /home/yahboom/b4m_yahboom/install/setup.bash 2>/dev/null && gazebo --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so; exec bash"
echo "Waiting for Gazebo to start..."
sleep 5
echo "Gazebo should now be running. If you see any errors in the Gazebo terminal, please address them before continuing."
wait_for_user

# Create fixed URDF file
echo "Step 3: Creating fixed URDF file without encoding declaration..."
cat /home/yahboom/b4m_yahboom/yahboomcar_description/urdf/yahboomcar_robot2.urdf | grep -v 'encoding="utf-8"' | sed 's|package://yahboomcar_description/meshes/|/home/yahboom/b4m_yahboom/yahboomcar_description/meshes/|g' > /home/yahboom/b4m_yahboom/yahboomcar_description/urdf/yahboomcar_robot2_fixed.urdf
echo "Fixed URDF file created successfully at: /home/yahboom/b4m_yahboom/yahboomcar_description/urdf/yahboomcar_robot2_fixed.urdf"
wait_for_user

# Spawn the robot model
echo "Step 4: Spawning robot model in Gazebo..."
echo "Running command: ros2 run gazebo_ros spawn_entity.py -file /home/yahboom/b4m_yahboom/yahboomcar_description/urdf/yahboomcar_robot2_fixed.urdf -entity yahboomcar -x 0 -y 0 -z 0.1"
echo "Opening a new terminal window for robot spawning..."
gnome-terminal --title="Robot Spawn" -- bash -c "source /opt/ros/humble/setup.bash && source /home/yahboom/b4m_yahboom/install/setup.bash 2>/dev/null && ros2 run gazebo_ros spawn_entity.py -file /home/yahboom/b4m_yahboom/yahboomcar_description/urdf/yahboomcar_robot2_fixed.urdf -entity yahboomcar -x 0 -y 0 -z 0.1; exec bash"
echo "Waiting for robot to spawn..."
sleep 3
echo "Robot should now be visible in Gazebo. If not, check the terminal for errors."
wait_for_user

# Add necessary transforms
echo "Step 5: Adding necessary transforms..."
echo "Running command for map->odom transform: ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom"
echo "Opening a new terminal window for map->odom transform..."
gnome-terminal --title="Map to Odom TF" -- bash -c "source /opt/ros/humble/setup.bash && source /home/yahboom/b4m_yahboom/install/setup.bash 2>/dev/null && ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom; exec bash"
sleep 1
echo "Running command for odom->base_link transform: ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom robot2/base_link"
echo "Opening a new terminal window for odom->base_link transform..."
gnome-terminal --title="Odom to Base Link TF" -- bash -c "source /opt/ros/humble/setup.bash && source /home/yahboom/b4m_yahboom/install/setup.bash 2>/dev/null && ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom robot2/base_link; exec bash"
sleep 1
echo "Transforms added successfully."
wait_for_user

# Start RViz
echo "Step 6: Starting RViz for visualization..."
echo "Running command: rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz"
echo "Opening a new terminal window for RViz..."
gnome-terminal --title="RViz" -- bash -c "source /opt/ros/humble/setup.bash && source /home/yahboom/b4m_yahboom/install/setup.bash 2>/dev/null && rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz; exec bash"
echo "Waiting for RViz to start..."
sleep 3
echo "RViz should now be running. If you see any errors in the RViz terminal, please address them before continuing."
wait_for_user

# Launch Nav2
echo "Step 7: Launching Nav2 with AMCL..."
echo "Running command: ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true autostart:=true"
echo "Opening a new terminal window for Nav2..."
gnome-terminal --title="Nav2 Components" -- bash -c "source /opt/ros/humble/setup.bash && source /home/yahboom/b4m_yahboom/install/setup.bash 2>/dev/null && ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true autostart:=true; exec bash"
echo "Waiting for Nav2 to start..."
sleep 5
echo "Nav2 should now be running. If you see any lifecycle errors in the Nav2 terminal, please address them before continuing."
wait_for_user

# Remind about setting initial pose
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

# Launch keyboard teleop
echo "Step 8: Launching keyboard teleop for robot control..."
echo "Running command: ros2 run teleop_twist_keyboard teleop_twist_keyboard"
echo "Opening a new terminal window for keyboard teleop..."
gnome-terminal --title="Keyboard Teleop" -- bash -c "source /opt/ros/humble/setup.bash && source /home/yahboom/b4m_yahboom/install/setup.bash 2>/dev/null && ros2 run teleop_twist_keyboard teleop_twist_keyboard; exec bash"
echo "Keyboard teleop started successfully."
sleep 2
wait_for_user

# Launch waypoint navigation
echo "Step 9: Launching waypoint navigation..."
echo "Running command: python3 /home/yahboom/b4m_yahboom/yahboomcar_nav/yahboomcar_nav/waypoint_navigation.py"
echo "Opening a new terminal window for waypoint navigation..."
gnome-terminal --title="Waypoint Navigation" -- bash -c "source /opt/ros/humble/setup.bash && source /home/yahboom/b4m_yahboom/install/setup.bash 2>/dev/null && python3 /home/yahboom/b4m_yahboom/yahboomcar_nav/yahboomcar_nav/waypoint_navigation.py; exec bash"
echo "Waypoint navigation started successfully."

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
echo "Workflow for creating and using waypoints:"
echo "  1. Use keyboard teleop to move the robot to a desired position"
echo "  2. Switch to the waypoint navigation terminal"
echo "  3. Press 's' to save the current position as a waypoint"
echo "  4. Repeat steps 1-3 for multiple waypoints"
echo "  5. Press 'l' to list saved waypoints"
echo "  6. Press 'g' to navigate to a specific waypoint"
echo ""
echo "To stop all components, run: pkill -f gazebo && pkill -f rviz2 && pkill -f 'ros2 run tf2_ros static_transform_publisher'"
