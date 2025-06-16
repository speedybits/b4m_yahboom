# b4m_waypoint_nav: Waypoint Navigation with Orientation Plan

## Overview

This document outlines the plan for implementing a waypoint navigation system with orientation for the Yahboom robot. The system will allow users to:

1. Manage waypoints through a dedicated GUI application
2. Store waypoints with orientation at specific positions on the map
3. Name and manage stored waypoints
4. Navigate to stored waypoints by name, with proper orientation upon arrival
5. Send navigation commands via MQTT for external control

The implementation will build upon the existing Navigation2 framework as described in the Lidar course documentation and utilize MQTT for communication with external systems.

## System Architecture

### Components

1. **Waypoint Management System**
   - Store waypoints with position (x, y) and orientation (quaternion)
   - Persist waypoints to file for reuse across sessions
   - Provide GUI interface for adding, listing, and selecting waypoints
   - Support map-based waypoint placement and management

2. **Waypoint Manager GUI**
   - Central dashboard for all waypoint operations
   - Map visualization with waypoint overlay
   - Add, edit, delete, and rename waypoints
   - Select and navigate to waypoints with a single click
   - Send navigation commands via MQTT

3. **Navigation Integration**
   - Leverage Navigation2 for path planning and obstacle avoidance
   - Send goal poses (position + orientation) to the navigation stack
   - Monitor navigation status and provide feedback
   - Visualize waypoints in RViz using marker arrays
   
4. **MQTT Communication**
   - Publish navigation commands to MQTT topics
   - Subscribe to robot status updates via MQTT
   - Enable external systems to trigger waypoint navigation
   - Provide standardized message format for waypoint commands
   
   MQTT Topic Structure:
   - `yahboom/navigation/command`: Topic for sending navigation commands
   - `yahboom/navigation/status`: Topic for receiving navigation status updates
   
   MQTT Message Format:
   - Navigation Command: `{"command": "goto", "waypoint_id": "waypoint_name"}`
   - Navigation Status: `{"status": "[in_progress|completed|failed]", "waypoint_id": "waypoint_name", "message": "status message"}`

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
- ~~Handle keyboard input for robot control and waypoint operations~~ (Deprecated: Keyboard functionality will be removed as the waypoint manager GUI becomes the central dashboard for waypoint operations)
- Subscribe to MQTT topics for receiving navigation commands

## Usage Instructions

1. **Start the System**
   ```bash
   # Terminal 1: Start the car's base functionality
   ros2 launch yahboomcar_bringup yahboomcar_bringup_launch.py
   
   # Terminal 2: Start the waypoint navigation system with GUI
   ros2 launch b4m_waypoint_nav waypoint_manager_launch.py
   ```

2. **Using the Waypoint Manager GUI**
   - The GUI serves as the central dashboard for all waypoint operations
   - Map area:
     - Click on the map to place new waypoints
     - Click on existing waypoints to select them
     - Right-click on waypoints for quick actions
   - Waypoint list panel:
     - View all waypoints for the current map
     - Select waypoints from the list to edit properties
     - Use buttons to add, edit, or delete waypoints
   - Navigation controls:
     - 'Go to Selected Waypoint' button sends navigation command via MQTT
     - Cancel navigation button to stop current navigation
     - Status indicators show current robot state and navigation progress

3. **Navigation Process**
   - The robot will plan a path to the selected waypoint
   - Navigation2 will handle obstacle avoidance
   - Upon arrival, the robot will orient itself according to the stored orientation
   - All navigation commands are transmitted via MQTT
   - External systems can trigger navigation by publishing to the appropriate MQTT topics
   - MQTT messages will be published for navigation events (start, success, failure)
   - Home Assistant integration for monitoring navigation status

4. **Waypoint Visualization**
   - Waypoints are visualized in RViz as text labels with their names
   - Arrows show the orientation of each waypoint
   - Each waypoint has a unique color for easy identification

## Future Enhancements



. **Simulation Integration**
   - Adapt the system to work in Gazebo simulation
   - Share waypoints between physical robot and simulation
   - Use the same waypoint file for both environments
   - Add configuration option to specify different waypoint files if needed


## Waypoint Manager Implementation

Based on the requirements and existing implementation, the Waypoint Manager will be a standalone application with optional integration with the running robot or simulation:

### Approach Selection

The **Waypoint Manager** has been implemented as a standalone Python-based GUI application that operates independently of the robot or simulation for the following reasons:

1. It allows for waypoint management without the overhead of running the full robot system or simulation
2. Users can plan and organize waypoints at any time, improving workflow efficiency
3. The application integrates with the running robot when needed for live operations

### Implementation Details

The implementation consists of:

1. **Python GUI using PyQt5**
   - PyQt5 was selected for its performance, comprehensive widget set, and ease of development
   - The application is optimized for responsiveness with immediate visual feedback
   - Robust error handling and debug logging have been added for better maintainability

2. **Map and Waypoint Management**
   - Loads maps from the `/home/yahboom/b4m_yahboom/install/b4m_waypoint_nav/maps/` directory
   - Displays waypoints with their names, positions, and orientations using ROS2 visualization markers
   - Provides intuitive controls for adding, editing, and deleting waypoints with immediate visual updates
   - Supports interactive map zooming and panning with mouse wheel and drag events

3. **UI Layout**
   - Left side panel containing waypoint list, map selection dropdown, and control buttons
   - Right side showing the map with waypoints visualized as colored markers
   - Clean, technical interface with status bar feedback for user actions
   - Click-to-place mechanism for adding new waypoints directly on the map
   - Click-to-select for existing waypoints with edit/delete options in the left panel

4. **Data Storage**
   - Uses JSON waypoint format stored at `/home/yahboom/b4m_yahboom/install/b4m_waypoint_nav/waypoints.json`
   - Each waypoint stores name, position, orientation, timestamp, and visualization properties
   - Changes are saved automatically and can be used immediately by the navigation system

5. **Bug Fixes and Improvements**
   - Fixed "TypeError: unhashable type: set" error in waypoint data handling
   - Resolved map drawing issues by converting float coordinates to integers
   - Added immediate map view updates after waypoint add/edit/delete operations
   - Enhanced error handling in scroll wheel zoom functionality
   - Implemented proper waypoint selection and highlighting
   - Current format includes: name, position (x,y), orientation (quaternion x,y,z,w), timestamp, and visualization properties (color, scale)
   - Enhance with map-specific waypoint sets (multi-map support)
   - Maintain backward compatibility with existing waypoint navigation system

5. **User Experience Features**
   - Enforce unique waypoint names with auto-naming suggestions (e.g., "Waypoint 1", "Waypoint 2")
   - Map selection dropdown to switch between available maps
   - Persistent settings between sessions (window size, last used map, UI preferences)
   - Technical interface designed for ROS2 developers
   - Disable waypoint editing features when in connected mode (gray out buttons)

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
   - Direct waypoint commands can be sent to the robot via MQTT
   - 'Go to Selected Waypoint' button becomes active when connected to a running robot
   - MQTT messages use the waypoint ID only, with the robot retrieving full position data from its waypoint storage
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

### Error Handling

The Waypoint Manager will implement the following error handling approach:

1. **Terminal-Based Error Logging**
   - All errors will be logged to the Linux terminal for debugging purposes
   - Use ROS2 logging mechanisms (get_logger().error, get_logger().warn)
   - No complex error dialogs in the UI to keep the interface clean

2. **Common Error Scenarios**
   - Missing map files: Display warning in terminal and UI status bar
   - Corrupted waypoint data: Attempt to load valid entries, log errors for invalid ones
   - Connection failures: Show disconnected status in UI, continue in standalone mode

### Launch File

A dedicated launch file will be created for the Waypoint Manager at `b4m_waypoint_nav/launch/waypoint_manager_launch.py` with the following features:

1. **Standalone Mode (Default)**
   - Launch only the Waypoint Manager without requiring other ROS2 nodes
   - Load map and waypoint data from filesystem

2. **Connected Mode (Optional)**
   - Parameter to enable connection to running robot/simulation
   - Auto-discovery of running ROS2 navigation stack
   - MQTT broker connection parameters:
     - `mqtt_broker`: MQTT broker address (default: 'localhost')
     - `mqtt_port`: MQTT broker port (default: 1883)
     - `mqtt_topic_prefix`: Prefix for all MQTT topics (default: 'yahboom')

### Repository Structure and Component Interaction

The Waypoint Manager will be implemented within the existing repository structure for several important reasons:

1. **Consistency with project structure**: The waypoint navigation functionality is already part of this repository (in the b4m_waypoint_nav directory), so adding the GUI component there maintains a clean organization.

2. **Dependency management**: The GUI will directly interact with the existing B4MWaypointNav node and share the same waypoint storage mechanism.

3. **Version control benefits**: Having the GUI in the same repository makes it easier to track changes and maintain compatibility between the waypoint navigation functionality and its UI.

4. **Simplified deployment**: Users won't need to clone multiple repositories to get the full functionality.

5. **Shared resources**: The GUI will use the same configuration files, waypoint storage, and RViz integration as the existing implementation.

#### Key Components and Their Roles

1. **waypoint_manager.py**
   - Provides the GUI interface for waypoint management
   - Allows users to create, edit, and delete waypoints visually on a map
   - Stores waypoints in a shared JSON format
   - Will be enhanced with a "Go to Selected Waypoint" button
   - Will publish MQTT messages to trigger navigation

2. **b4m_waypoint_nav.py**
   - Core navigation node that interfaces with ROS2 Navigation2
   - Handles the actual robot movement and navigation
   - Enhanced with MQTT client for bidirectional communication
   - Subscribes to MQTT navigation commands on `yahboom/navigation/command` topic
   - Executes the navigation to waypoints when requested via MQTT
   - Publishes status messages to `mqtt_status` topic
   - No longer uses keyboard control (deprecated in favor of GUI control)

#### Communication Flow

1. **User Interaction**
   - User selects a waypoint in the waypoint_manager GUI
   - User clicks "Go to Selected Waypoint" button

2. **MQTT Communication**
   - waypoint_manager.py publishes a message to `yahboom/navigation/command` topic
   - Message contains the waypoint ID to navigate to

3. **Navigation Execution**
   - b4m_waypoint_nav.py receives the MQTT message
   - Looks up the waypoint coordinates from its waypoint storage
   - Sends the navigation goal to the Navigation2 stack
   - Reports status back via the `yahboom/navigation/status` topic

4. **Status Updates**
   - Navigation progress and completion status are published back to MQTT
   - External systems (like Home Assistant) can monitor these status messages

The GUI component will be added as:
- `b4m_waypoint_nav/b4m_waypoint_nav/waypoint_manager.py` for the GUI implementation
- `b4m_waypoint_nav/launch/waypoint_manager_launch.py` for launching the GUI

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

4. **Implement MQTT Integration**
   - Set up MQTT client in the waypoint manager using paho-mqtt library
   - Define message formats for waypoint navigation commands
   - Implement 'Go to Selected Waypoint' functionality
   - Create subscriber for robot status updates
   - Handle connection failures gracefully with appropriate user feedback
   - Support both local testing with Mosquitto broker and integration with Home Assistant

5. **Testing and Refinement**
   - Test with simulated robot in Gazebo
   - Test with physical Yahboom robot
   - Test MQTT communication with external systems
   - Refine UI based on user feedback

This implementation will provide a user-friendly interface for managing waypoints while leveraging the existing ROS2 infrastructure and visualization capabilities.

## Conclusion

This plan provides a comprehensive approach to implementing waypoint navigation with orientation for the Yahboom robot. By leveraging the Navigation2 framework, adding a central GUI dashboard for waypoint management, and implementing MQTT communication for external control, we can create a flexible and user-friendly system for robot navigation that can be easily integrated with other systems.
