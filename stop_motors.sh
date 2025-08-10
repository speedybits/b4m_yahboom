#!/bin/bash

# Script to emergency stop robot motors
# This script sends a zero velocity command to stop the robot immediately

echo "Emergency stopping robot motors..."

# Send zero velocity command to stop the robot
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

echo "Stop command sent to robot motors."
echo "Robot should now be stopped."

# Also try to stop any active navigation goals
echo "Canceling any active navigation goals..."
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{}" --feedback &
sleep 0.5
pkill -f "ros2 action send_goal"

echo "Motor stop sequence complete."