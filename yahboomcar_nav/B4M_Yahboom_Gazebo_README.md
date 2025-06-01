# B4M Yahboom Multi-Waypoint Navigation in Gazebo

## Overview

This package provides a multi-waypoint navigation system for the Yahboom robot in a Gazebo simulation environment using ROS2 Navigation2. The system allows you to:

1. Store multiple waypoints with custom names in the simulated environment
2. Select waypoints via keyboard input
3. Navigate to selected waypoints with path planning and obstacle avoidance

## Features

- **Gazebo Simulation**: Run the Yahboom robot in a simulated environment
- **Waypoint Storage**: Save the robot's current position as a named waypoint
- **Waypoint Selection**: List and select waypoints using keyboard commands
- **Path Planning**: Navigate to selected waypoints using ROS2 Navigation2 with obstacle avoidance
- **Interactive Interface**: Simple keyboard-based interface for controlling the robot

## Requirements

- ROS2 Humble
- Gazebo
- Navigation2 package
- Yahboom robot URDF model

## Installation

The code is already integrated into the yahboomcar_nav package. To build it:

```bash
cd ~/yahboomcar_ws
colcon build --packages-select yahboomcar_nav
source install/setup.bash
```

## Usage

### 1. Start the Gazebo Simulation with Navigation

Launch the Gazebo simulation with the Yahboom robot and Navigation2:

```bash
ros2 launch yahboomcar_nav gazebo_waypoint_navigation_launch.py
```

This single command will:
- Start the Gazebo simulation server
- Spawn the Yahboom robot model in Gazebo
- Launch the Navigation2 stack with the map
- Start the waypoint navigation node

### 2. Set Initial Pose

In the RViz interface that appears, use the **2D Pose Estimate** tool to set the initial pose of the robot.

### 3. Using Keyboard Commands

The system provides the following keyboard commands:

- `s` - Save current position as a waypoint (you'll be prompted for a name)
- `l` - List all saved waypoints
- `g` - Go to a waypoint (you'll select from the list)
- `c` - Cancel current navigation
- `q` - Quit the program

## How It Works

1. **Gazebo Simulation**: The system launches a Gazebo simulation with the Yahboom robot model.

2. **Initialization**: The system initializes the Navigation2 action client and sets up keyboard input handling.

3. **Waypoint Storage**: When you press 's', the system saves the current position of the simulated robot as a waypoint with a name you provide.

4. **Waypoint Navigation**: When you press 'g' and select a waypoint, the system:
   - Sends the waypoint as a goal to the Navigation2 action server
   - The simulated robot plans a path to the goal while avoiding obstacles
   - The robot follows the path to reach the goal
   - You can cancel navigation at any time by pressing 'c'

5. **Shutdown**: When you press 'q', the system stops the robot and shuts down cleanly.

## Implementation Details

The system consists of the following components:

1. **gazebo_waypoint_navigation.py**: The main ROS2 node that implements:
   - TF2 integration for getting the robot's position in the simulated environment
   - Waypoint storage and management
   - Keyboard input handling
   - Navigation2 action client for path planning and execution

2. **gazebo_waypoint_navigation_launch.py**: A launch file that starts:
   - The Gazebo simulation server
   - The Yahboom robot model in Gazebo
   - The Navigation2 stack with map and parameters
   - The waypoint navigation node
   - Required TF transformations

## Differences from Real Robot Implementation

The Gazebo implementation differs from the real robot implementation in the following ways:

1. **Robot Position**: Uses TF2 to get the robot's position in the simulated environment instead of odometry.
2. **Simulation Clock**: Uses the Gazebo simulation clock instead of the real clock.
3. **Launch Process**: Launches Gazebo and spawns the robot model in addition to the navigation stack.

## Troubleshooting

- If the robot doesn't respond to navigation commands, ensure that:
  - The initial pose is set correctly in RViz
  - The map is loaded properly
  - The simulated laser scanner is functioning correctly

- If keyboard commands don't work, make sure:
  - The terminal window running the waypoint navigation node is in focus
  - The terminal is in raw mode (this should happen automatically)

## License

This software is part of the Yahboom robot package and follows its licensing terms.

## Credits

This implementation is based on the Navigation2 framework and extends the functionality described in the B4M_Yahboom_Navigation2 navigation avoid documentation.
