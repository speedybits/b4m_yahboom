# B4M Yahboom Multi-Waypoint Navigation System

## Overview

This package provides a multi-waypoint navigation system for the Yahboom robot using ROS2 Navigation2. The system allows you to:

1. Store multiple waypoints with custom names
2. Select waypoints via keyboard input
3. Navigate to selected waypoints with path planning and obstacle avoidance

## Features

- **Waypoint Storage**: Save the robot's current position as a named waypoint
- **Waypoint Selection**: List and select waypoints using keyboard commands
- **Path Planning**: Navigate to selected waypoints using ROS2 Navigation2 with obstacle avoidance
- **Interactive Interface**: Simple keyboard-based interface for controlling the robot

## Requirements

- ROS2 Humble
- Navigation2 package
- Yahboom robot with laser scanner

## Installation

The code is already integrated into the yahboomcar_nav package. To build it:

```bash
cd ~/yahboomcar_ws
colcon build --packages-select yahboomcar_nav
source install/setup.bash
```

## Usage

### 1. Start the Robot Base

First, start the car to process the underlying data:

```bash
ros2 launch yahboomcar_bringup yahboomcar_bringup_launch.py
```

### 2. Start RViz Visualization

In a new terminal, start RViz for visualization:

```bash
ros2 launch yahboomcar_nav display_launch.py
```

### 3. Launch the Waypoint Navigation System

In another terminal, launch the waypoint navigation system:

```bash
ros2 launch yahboomcar_nav waypoint_navigation_launch.py
```

### 4. Set Initial Pose

In the RViz interface, use the **2D Pose Estimate** tool to give the initial pose of the car.

### 5. Using Keyboard Commands

The system provides the following keyboard commands:

- `s` - Save current position as a waypoint (you'll be prompted for a name)
- `l` - List all saved waypoints
- `g` - Go to a waypoint (you'll select from the list)
- `c` - Cancel current navigation
- `q` - Quit the program

## How It Works

1. **Initialization**: The system initializes the Navigation2 action client and sets up keyboard input handling.

2. **Waypoint Storage**: When you press 's', the system saves the current position as a waypoint with a name you provide.

3. **Waypoint Navigation**: When you press 'g' and select a waypoint, the system:
   - Sends the waypoint as a goal to the Navigation2 action server
   - The robot plans a path to the goal while avoiding obstacles
   - The robot follows the path to reach the goal
   - You can cancel navigation at any time by pressing 'c'

4. **Shutdown**: When you press 'q', the system stops the robot and shuts down cleanly.

## Implementation Details

The system consists of the following components:

1. **waypoint_navigation.py**: The main ROS2 node that implements:
   - Waypoint storage and management
   - Keyboard input handling
   - Navigation2 action client for path planning and execution

2. **waypoint_navigation_launch.py**: A launch file that starts:
   - The Navigation2 stack with map and parameters
   - The waypoint navigation node
   - Required TF transformations

## Troubleshooting

- If the robot doesn't respond to navigation commands, ensure that:
  - The initial pose is set correctly in RViz
  - The map is loaded properly
  - The laser scanner is functioning correctly

- If keyboard commands don't work, make sure:
  - The terminal window running the waypoint navigation node is in focus
  - The terminal is in raw mode (this should happen automatically)

## License

This software is part of the Yahboom robot package and follows its licensing terms.

## Credits

This implementation is based on the Navigation2 framework and extends the functionality described in the B4M_Yahboom_Navigation2 navigation avoid documentation.
