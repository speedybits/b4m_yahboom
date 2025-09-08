#!/bin/bash

echo "EMERGENCY STOP - Attempting multiple stop methods..."

# Method 1: Send zero velocity with higher rate
echo "[1/4] Sending continuous zero velocity commands..."
timeout 2 ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' --rate 10 &

# Method 2: Try canceling navigation goals
echo "[2/4] Canceling navigation goals..."
ros2 action send_goal /cancel_navigation nav2_msgs/action/NavigateToPose '{}' 2>/dev/null &

# Method 3: Cancel through navigate_to_pose
echo "[3/4] Canceling navigate_to_pose action..."
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose '{}' 2>/dev/null &

# Method 4: Send emergency stop to base controller if available
echo "[4/4] Sending emergency stop to base controller..."
ros2 topic pub --once /emergency_stop std_msgs/msg/Bool '{data: true}' 2>/dev/null

# Wait for commands to take effect
sleep 1

# Final single zero velocity command
echo "Sending final stop command..."
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

# Kill any remaining pub processes
pkill -f "ros2 topic pub /cmd_vel"

echo "EMERGENCY STOP COMPLETE - Robot should be stopped"
echo "If robot is still moving, try: ./b4m_shutdown.sh --keep-agent"