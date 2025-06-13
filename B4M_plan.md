# b4m_waypoint_nav: Waypoint Navigation with Orientation Plan

## Overview

This document outlines the plan for implementing a waypoint navigation system with orientation for the Yahboom robot. The system will allow users to:

1. Drive the robot using keyboard controls
2. Store waypoints with orientation at the robot's current position
3. Name and manage stored waypoints
4. Navigate to stored waypoints by name, with proper orientation upon arrival

The implementation will build upon the existing Navigation2 framework as described in the Lidar course documentation.

## System Architecture

### Components

1. **Waypoint Management System**
   - Store waypoints with position (x, y) and orientation (quaternion)
   - Persist waypoints to file for reuse across sessions
   - Provide interface for adding, listing, and selecting waypoints

2. **Keyboard Control Interface**
   - Drive the robot using keyboard commands based on the VM Remote control scheme:
     - 'i': Go forward
     - ',': Move back
     - 'l': Right rotation
     - 'j': Left rotation
     - 'u': Turn left
     - 'o': Turn right
     - 'm': Reverse left
     - '.': Reverse right
   - Trigger waypoint storage with a specific key ('s')
   - Select waypoints for navigation via keyboard input ('g')
   - List available waypoints ('p')
   - Delete waypoints ('d')

3. **Navigation Integration**
   - Leverage Navigation2 for path planning and obstacle avoidance
   - Send goal poses (position + orientation) to the navigation stack
   - Monitor navigation status and provide feedback
   - Visualize waypoints in RViz using marker arrays

### Data Structure

Each waypoint will be stored with the following information:
```
{
  "name": "string",       // User-defined name for the waypoint
  "position": {
    "x": float,           // X coordinate in map frame
    "y": float            // Y coordinate in map frame
  },
  "orientation": {
    "x": float,           // Quaternion x component
    "y": float,           // Quaternion y component
    "z": float,           // Quaternion z component
    "w": float            // Quaternion w component
  },
  "timestamp": string,    // When the waypoint was created
  "visualization": {      // Properties for RViz visualization
    "color": {           // RGB color for the marker
      "r": float,        // Red component (0.0-1.0)
      "g": float,        // Green component (0.0-1.0)
      "b": float         // Blue component (0.0-1.0)
    },
    "scale": float       // Size of the waypoint marker
  }
}
```

## Implementation Plan

### 1. Create Waypoint Navigation Node

Develop a ROS2 node (`b4m_waypoint_nav.py`) that will:

- Subscribe to robot pose topics to get current position and orientation
- Publish goal poses to the Navigation2 stack
- Implement waypoint management logic
- Handle keyboard input for robot control and waypoint operations

## Usage Instructions

1. **Start the System**
   ```bash
   # Terminal 1: Start the car's base functionality
   ros2 launch yahboomcar_bringup yahboomcar_bringup_launch.py
   
   # Terminal 2: Start the waypoint navigation system
   ros2 launch b4m_waypoint_nav b4m_waypoint_nav_launch.py
   ```

2. **Control the Robot**
   - Keyboard control of the robot will be closely based on the @04.VM Remote control course-1. VM keyboard remote control.txt implementation
   - Use the following keys to drive the robot:
     - 'i': Go forward
     - ',': Move back
     - 'l': Right rotation
     - 'j': Left rotation
     - 'u': Turn left
     - 'o': Turn right
     - 'm': Reverse left
     - '.': Reverse right
     - ' ' (spacebar): Stop the robot
   - Speed adjustment:
     - 'q': Increase both linear and angular speed
     - 'z': Decrease both linear and angular speed
     - 'w': Increase linear speed only
     - 'x': Decrease linear speed only
     - 'e': Increase angular speed only
     - 'c': Decrease angular speed only
   - Press 's' to store a waypoint at the current position:
     - Enter a name for the waypoint when prompted
   - Press 'p' to list all stored waypoints
   - Press 'g' to navigate to a waypoint:
     - Enter the name of the waypoint to navigate to when prompted
   - Press 'c' to cancel navigation
   - Press 'd' to delete a waypoint:
     - Enter the name of the waypoint to delete when prompted

3. **Navigation Process**
   - The robot will plan a path to the selected waypoint
   - Navigation2 will handle obstacle avoidance
   - Upon arrival, the robot will orient itself according to the stored orientation
   - MQTT messages will be published for navigation events (start, success, failure)
   - Home Assistant integration for monitoring navigation status

4. **Waypoint Visualization**
   - Waypoints are visualized in RViz as text labels with their names
   - Arrows show the orientation of each waypoint
   - Each waypoint has a unique color for easy identification

## Future Enhancements

1. **Waypoint Management UI**
   - Develop a graphical interface for waypoint management
   - Visualize waypoints on a map
   - Allow drag-and-drop waypoint creation and editing

2. **Simulation Integration**
   - Adapt the system to work in Gazebo simulation
   - Share waypoints between physical robot and simulation
   - Use the same waypoint file for both environments
   - Add configuration option to specify different waypoint files if needed


## Waypoint Manager Implementation

Based on the requirements and existing implementation, the Waypoint Manager will be a standalone application with optional integration with the running robot or simulation:

### Approach Selection

The **Waypoint Manager** will be implemented as a standalone Python-based GUI application that can operate independently of the robot or simulation for the following reasons:

1. It allows for waypoint management without the overhead of running the full robot system or simulation
2. Users can plan and organize waypoints at any time, improving workflow efficiency
3. The application can still integrate with the running robot when needed for live operations

### Implementation Outline

The implementation will consist of:

1. **Python GUI using PyQt5**
   - PyQt5 is selected for its performance, comprehensive widget set, and ease of development
   - No existing GUI framework is currently used in the project that would be better suited
   - The application will be optimized for responsiveness and intuitive interaction

2. **Map and Waypoint Management**
   - Load maps from `/home/yahboom/b4m_yahboom/yahboomcar_nav/maps/` directory
   - Display waypoints with their names, positions, and orientations
   - Provide intuitive controls for adding, editing, and deleting waypoints
   - Support waypoint reordering and organization

3. **UI Layout**
   - Left side panel containing waypoint list and control options
   - Right side showing the map with waypoints visualized
   - Clean, technical interface without unnecessary clutter
   - Click-to-place mechanism for adding new waypoints on the map
   - Click-to-select for existing waypoints with action buttons in the left panel

4. **Data Storage**
   - Use the existing JSON waypoint format stored at repository root (`/home/yahboom/b4m_yahboom/waypoints.json`)
   - Current format includes: name, position (x,y), orientation (quaternion x,y,z,w), timestamp, and visualization properties (color, scale)
   - Potential enhancements: waypoint categories, sequence ordering, navigation parameters
   - Maintain backward compatibility with existing waypoint navigation system

### Primary Operation Mode: Map-Based Waypoint Management

The Waypoint Manager will operate primarily as a standalone application that doesn't require the robot or simulation to be running:

1. **Map-Based Waypoint Editing (Priority 1)**
   - Load and display saved maps directly from the filesystem
   - Create, edit, and organize waypoints on the loaded map
   - Save waypoints to the shared JSON file that will be used by the robot/simulation

2. **Map Loading Capabilities**
   - Load map files (.pgm and .yaml) from the same location used by existing applications
   - Display the map with proper scaling and orientation
   - Support for multiple maps with configuration options

3. **Waypoint Persistence**
   - All waypoint changes are saved to the same waypoint file used by the robot
   - Option to create map-specific waypoint files for different environments
   - Import/export functionality for waypoint configurations

4. **Path Planning Preview (Future Enhancement)**
   - Visualize potential navigation paths between waypoints
   - Estimate travel times and distances
   - Highlight potential navigation challenges based on map features

5. **Implementation Approach**
   - The GUI will use a lightweight ROS2 node that doesn't require the full navigation stack
   - Map rendering will use Qt's graphics capabilities rather than requiring RViz
   - Live robot position will be displayed when connected to a running robot/simulation

6. **Connected Mode (Optional)**
   - When the robot or simulation is running, the UI automatically detects and connects to it
   - Live position updates are shown on the map
   - Direct waypoint commands can be sent to the robot
   - Real-time feedback during navigation

### ROS2 Dependencies

The Waypoint Manager will require the following ROS2 dependencies:

1. **Core ROS2 Dependencies**
   - `rclpy`: Python client library for ROS2
   - `geometry_msgs`: For pose and transform messages
   - `nav2_msgs`: For navigation action interfaces
   - `visualization_msgs`: For marker visualization
   - `std_msgs`: For standard message types

2. **PyQt5 Dependencies**
   - `python3-pyqt5`: Main GUI framework
   - `python3-pyqt5.qtsvg`: For SVG rendering support

3. **Additional Python Dependencies**
   - `numpy`: For numerical operations
   - `pillow`: For image processing (map rendering)

### Launch File

A dedicated launch file will be created for the Waypoint Manager at `b4m_waypoint_nav/launch/waypoint_manager_launch.py` with the following features:

1. **Standalone Mode (Default)**
   - Launch only the Waypoint Manager without requiring other ROS2 nodes
   - Load map and waypoint data from filesystem

2. **Connected Mode (Optional)**
   - Parameter to enable connection to running robot/simulation
   - Auto-discovery of running ROS2 navigation stack

### Repository Structure

The Waypoint Manager will be implemented within the existing repository structure for several important reasons:

1. **Consistency with project structure**: The waypoint navigation functionality is already part of this repository (in the b4m_waypoint_nav directory), so adding the GUI component there maintains a clean organization.

2. **Dependency management**: The GUI will directly interact with the existing B4MWaypointNav node and share the same waypoint storage mechanism.

3. **Version control benefits**: Having the GUI in the same repository makes it easier to track changes and maintain compatibility between the waypoint navigation functionality and its UI.

4. **Simplified deployment**: Users won't need to clone multiple repositories to get the full functionality.

5. **Shared resources**: The GUI will use the same configuration files, waypoint storage, and RViz integration as the existing implementation.

The GUI component will be added as:
- `b4m_waypoint_nav/b4m_waypoint_nav/waypoint_gui.py` for the GUI implementation
- `b4m_waypoint_nav/launch/waypoint_gui_launch.py` for launching the GUI

This structure maintains the organization of the codebase while adding the new functionality in a logical location.

### Development Steps

1. **Create the GUI Framework**
   - Set up a basic PyQt5 application with ROS2 integration
   - Design the layout with a waypoint list and control buttons
   - Implement map preview widget

2. **Implement Waypoint Management Features**
   - Add/edit/delete waypoints
   - Rename waypoints
   - Adjust waypoint orientation
   - Save/load waypoints from file

3. **Integrate with RViz**
   - Create custom RViz panels if needed
   - Set up communication between GUI and RViz
   - Implement click-to-create waypoint functionality

4. **Testing and Refinement**
   - Test with simulated robot in Gazebo
   - Test with physical Yahboom robot
   - Refine UI based on user feedback

This implementation will provide a user-friendly interface for managing waypoints while leveraging the existing ROS2 infrastructure and visualization capabilities.

## Conclusion

This plan provides a comprehensive approach to implementing waypoint navigation with orientation for the Yahboom robot. By leveraging the Navigation2 framework and adding custom waypoint management features, we can create a flexible and user-friendly system for robot navigation.
