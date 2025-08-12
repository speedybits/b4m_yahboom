This document is divided into two main sections:
1. **Robot Manager GUI**: Describes the standalone GUI application for managing waypoints
2. **MQTT-Based Navigation**: Explains how the waypoint manager receives coordinates via MQTT and navigates the robot

# SECTION 1: ROBOT MANAGER GUI

## Overview

The Robot Manager GUI is a standalone application that allows users to create, edit, and manage waypoints for the Yahboom robot. It operates independently of the robot or simulation for efficient workflow, with optional integration when the robot is running.

## Implementation Approach

The Robot Manager has been implemented as a standalone Python-based GUI application for the following reasons:

1. It allows for waypoint management without the overhead of running the full robot system or simulation
2. Users can plan and organize waypoints at any time, improving workflow efficiency
3. The application integrates with the running robot when needed for live operations

## Technical Implementation

### Core Components

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
   - Left side panel containing:
     - **Micro ROS Agent Control**: Start Agent and Stop All buttons at the top
     - **System Control Panel**: Rebuild, Start, and Stop buttons for robot system management
     - **Configuration Management**: Save/Load configuration buttons
     - **Map Information**: Current map display
     - **Waypoint Management**: List, add, edit, delete waypoint controls
     - **Navigation Parameters**: Collapsible parameter adjustment section
   - Right side showing the map with waypoints visualized as colored markers
   - Clean, technical interface with status displays and feedback for user actions
   - Click-to-place mechanism for adding new waypoints directly on the map
   - Click-to-select for existing waypoints with edit/delete options in the left panel

4. **Data Storage**
   - Uses JSON waypoint format stored at `/home/yahboom/b4m_yahboom/install/b4m_waypoint_nav/waypoints.json`
   - Each waypoint stores name, position, orientation, timestamp, and visualization properties
   - Changes are saved automatically and can be used immediately by the navigation system

### User Interface Features

1. **Map-Based Waypoint Editing**
   - Load and display saved maps directly from the filesystem
   - Create, edit, and organize waypoints on the loaded map
   - Save waypoints to the shared JSON file that will be used by the robot/simulation

2. **Waypoint Management**
   - Add new waypoints by clicking on the map
   - Edit existing waypoints by selecting them from the list or map
   - Delete waypoints that are no longer needed
   - Rename waypoints with descriptive names
   - Adjust waypoint orientation for proper robot positioning

3. **User Experience Enhancements**
   - Enforce unique waypoint names with auto-naming suggestions (e.g., "Waypoint 1", "Waypoint 2")
   - Map selection dropdown to switch between available maps
   - Persistent settings between sessions (window size, last used map, UI preferences)
   - Technical interface designed for ROS2 developers
   - Waypoint list displays the name of each waypoint along with its X/Y coordinates (formatted to 2 decimal places in parentheses)

4. **Connected Mode Features**
   - When the robot or simulation is running, the UI automatically detects and connects to it
   - Live position updates are shown on the map
   - Direct waypoint commands can be sent to the robot via MQTT
   - 'Go to Selected Waypoint' button becomes active when connected to a running robot

5. **Central Launch Control System**
   - The B4M Robot Manager serves as the central application for complete system lifecycle management
   - Provides an integrated interface for starting, monitoring, and stopping all robot system components
   - Eliminates the need for manual script execution and terminal management

### Central Launch Control Features

1. **Micro ROS Agent Management**
   - **Start Agent** button: Launches the Micro-ROS Agent Docker container for ESP32 communication
   - **Stop All** button: Terminates all processes including the Micro-ROS Agent and resets system to clean state
   - Agent status display shows current state with color-coded background (red=stopped, green=running)

2. **Robot System Control**
   - **Rebuild** button: Executes `colcon build --symlink-install` when parameter or map changes require rebuilding
   - **Start** button: Initiates the automated launch sequence for all robot system components
   - **Stop** button: Gracefully shuts down robot system components while preserving the Micro-ROS Agent
   - System status display shows current launch step and overall system state

3. **Automated Launch Sequence**
   The Start button executes the following sequence (derived from b4m_launch.sh):
   - Step 1: Start sensor integration (`yahboomcar_bringup_launch.py`)
   - Step 2: Start RViz visualization (`display_launch.py`)
   - Step 3: Start navigation system (`waypoint_navigation_launch.py`)
   - Step 4: Manual robot positioning (popup dialog for 2D Pose Estimate)
   - Step 5: Start waypoint navigation with MQTT integration (`b4m_waypoint_nav.py`)
   
4. **Smart Button State Management**
   All control buttons are intelligently enabled/disabled based on system state to prevent invalid operations:

   **Micro ROS Agent Control:**
   - **Start Agent**: Enabled when agent is stopped
   - **Stop All**: Enabled when agent is running

   **System Control (depends on agent state):**
   - **Rebuild**: 
     - Enabled when: Agent running AND parameters modified OR map changed
     - Grayed when: Agent stopped OR no changes requiring rebuild
   - **Start**: 
     - Enabled when: Agent running AND system stopped
     - Grayed when: Agent stopped OR system starting/running
   - **Stop**: 
     - Enabled when: Agent running AND (system starting OR running)
     - Grayed when: Agent stopped OR system already stopped

   **Navigation Parameters:**
   - **Editable**: When agent running AND system stopped
   - **Read-only (grayed)**: When system starting OR running
   - **Disabled**: When agent stopped

5. **Process Management**
   - Each launch step runs in a separate terminal window for visibility and monitoring
   - Process tracking maintains references to launched components
   - Graceful shutdown attempts normal termination before force-killing processes
   - Clear separation between agent-level shutdown (Stop All) and system-level shutdown (Stop)

6. **Manual Step Integration**
   - **2D Pose Estimate Dialog**: Appears during launch sequence with clear instructions
   - **User Confirmation**: OK button allows user to proceed when manual positioning is complete
   - **Process Continuity**: Launch sequence automatically continues after manual steps

### Button State Dependencies and Behavior

The GUI implements a hierarchical dependency system where the Micro-ROS Agent serves as the foundational layer:

1. **Agent Dependency Hierarchy**
   ```
   Micro-ROS Agent (foundational)
   └── Robot System Components
       ├── Sensor Integration
       ├── RViz Visualization  
       ├── Navigation System
       ├── Manual Positioning
       └── Waypoint Navigation
   ```

2. **State Transition Logic**
   - **Agent Stopped** → All system controls disabled (grayed)
   - **Agent Starting** → Agent controls update, system controls remain disabled
   - **Agent Running** → System controls become available based on system state
   - **System Starting** → Start button disabled, Stop button enabled, parameters read-only
   - **System Running** → Only Stop button enabled, parameters read-only
   - **System Stopped** (with agent running) → Start and Rebuild (if needed) enabled, parameters editable

3. **Visual Feedback System**
   - **Red background**: Agent stopped, system unavailable
   - **Green background**: Agent running, system ready
   - **Grayed buttons**: Action not available in current state
   - **Enabled buttons**: Action available and safe to execute
   - **Status text**: Clear indication of current state and next possible actions

4. **Parameter Editing States**
   - **Full Edit Mode**: Agent running, system stopped - all parameters editable
   - **Read-Only Mode**: System active - parameters visible but grayed and non-editable
   - **Disabled Mode**: Agent stopped - parameter section completely disabled

5. **Rebuild Detection**
   - Automatically enabled when navigation parameters are modified
   - Placeholder for future map changing functionality (currently documented for future implementation)
   - Only available when agent is running and system is in appropriate state

### Waypoint Data Structure

Each waypoint is stored with the following information:
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

### Single Map Approach

The Robot Manager GUI is simplified to work with a single map (yahboom_map):

1. **Map Display**
   - The GUI displays the yahboom_map by default
   - No map selection interface is needed as only one map is supported
   - The map is loaded from `/home/yahboom/b4m_yahboom/yahboomcar_nav/maps/yahboom_map.yaml`

2. **Waypoint Management**
   - All waypoints are associated with the yahboom_map
   - The waypoint list displays all waypoints in the yahboom_map section of the JSON file
   - The GUI clearly indicates that it's working with yahboom_map

### Central Launch Control Implementation

1. **Docker Integration**
   - Uses Docker commands to manage Micro-ROS Agent container lifecycle
   - Monitors container status using `docker ps` with filtering
   - Graceful container shutdown with `docker stop` and `docker rm`

2. **Process Management**
   - Threaded execution prevents GUI blocking during long operations
   - Terminal windows launched using `gnome-terminal` for process visibility
   - Process tracking with subprocess.Popen references
   - Graceful shutdown with escalation to force-kill if needed

3. **State Management Architecture**
   - Dual-state system: agent_state ('stopped'/'running') and system_state ('stopped'/'starting'/'running')
   - Centralized button state updates through `updateControlButtonStates()` method
   - Event-driven parameter change detection with automatic rebuild flagging

4. **Launch Sequence Execution**
   - Step-by-step execution with 3-second delays between steps
   - QTimer-based non-blocking progression through launch sequence
   - Manual step integration with modal dialogs
   - Error handling and status reporting at each step

### Dependencies

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
   - `threading`: For non-blocking operations
   - `subprocess`: For process management and Docker integration

4. **System Dependencies**
   - **Docker**: Required for Micro-ROS Agent container management
   - **gnome-terminal**: For launching processes in visible terminal windows
   - **colcon**: For building ROS2 packages when parameters change

### Error Handling

1. **Terminal-Based Error Logging**
   - All errors are logged to the Linux terminal for debugging purposes
   - Use ROS2 logging mechanisms (get_logger().error, get_logger().warn)
   - No complex error dialogs in the UI to keep the interface clean

2. **Common Error Scenarios**
   - Missing map files: Display warning in terminal and UI status bar
   - Corrupted waypoint data: Attempt to load valid entries, log errors for invalid ones
   - Connection failures: Show disconnected status in UI, continue in standalone mode

# SECTION 2: MQTT-BASED NAVIGATION

## Overview

This section describes how the waypoint manager receives coordinates in MQTT format (from either HomeAssistant or the Robot Manager GUI) and causes the robot to navigate to the specified waypoint.

## MQTT Communication Architecture

### Topic Structure

The waypoint navigation system uses the following MQTT topics:

- `yahboom/navigation/command`: Topic for sending navigation commands
- `yahboom/navigation/status`: Topic for receiving navigation status updates
- `yahboom/navigation/info`: Topic for receiving information about command formats and system capabilities

### Message Format

The system supports two command formats:

1. **Waypoint-based navigation** (deprecated):
   ```json
   {"command": "goto", "waypoint_id": "waypoint_name"}
   ```

2. **Coordinate-based navigation** (preferred):
   ```json
   {"command": "goto", "waypoint_id": "optional_name", "position": {"x": float, "y": float}, "orientation": {"x": float, "y": float, "z": float, "w": float}}
   ```

Status messages are published in the following format:
```json
{"status": "[in_progress|completed|failed]", "waypoint_id": "waypoint_name", "message": "status message"}
```

## Navigation Process

### Command Reception

1. **MQTT Subscription**
   - The `b4m_waypoint_nav.py` node subscribes to the `yahboom/navigation/command` topic
   - It receives JSON-formatted navigation commands from either HomeAssistant or the Robot Manager GUI
   - The node validates the received command format and extracts the necessary information

2. **Command Processing**
   - For coordinate-based commands, the node directly extracts position and orientation values
   - The waypoint_id field is used only for debugging and logging purposes
   - The node does not need to perform any waypoint lookup as the coordinates are provided directly

### Navigation Execution

1. **Goal Creation**
   - The node creates a Navigation2 goal pose from the received coordinates
   - The goal includes both position (x, y) and orientation (quaternion)

2. **Navigation Request**
   - The goal is sent to the Navigation2 action server
   - The node monitors the navigation progress through action feedback

3. **Status Updates**
   - Navigation status is published to the `yahboom/navigation/status` topic
   - Status updates include: navigation started, in progress, completed, or failed
   - The waypoint_id from the original command is included in status messages for tracking

4. **Navigation Completion**
   - Upon successful arrival, the robot orients itself according to the specified orientation
   - A final status message is published indicating completion
   - The node returns to an idle state, ready to receive the next command

## Dynamic Waypoint Management

The system supports dynamic waypoint management without requiring navigation node restarts:

1. **Coordinate-Based Navigation**
   - The Robot Manager GUI **MUST** send coordinate-based navigation commands
   - The GUI maintains its own internal mapping of waypoint names to coordinates
   - When a user selects a waypoint in the GUI, it looks up the coordinates and sends those
   - The waypoint name is included in the MQTT message for debugging purposes only
   - The navigation node processes these coordinates directly without any waypoint lookup

2. **Benefits**
   - Waypoints can be added, edited, or removed without restarting the navigation node
   - Changes to waypoints take effect immediately
   - Improved reliability as the navigation node doesn't need to maintain waypoint state
   - The navigation node is decoupled from waypoint management

## Implementation Details

### Core Navigation Node

The `b4m_waypoint_nav.py` node is the central component that handles the actual robot movement:

1. **ROS2 Integration**
   - Interfaces with the Navigation2 stack for path planning and execution
   - Subscribes to robot pose topics to get current position and orientation
   - Publishes goal poses to the Navigation2 action server

2. **MQTT Client**
   - Implements a bidirectional MQTT client using the paho-mqtt library
   - Subscribes to command topics and publishes status updates
   - Handles connection management and reconnection logic

3. **Command Execution**
   - Parses and validates incoming MQTT commands
   - Creates and sends navigation goals to Navigation2
   - Monitors navigation progress and reports status

4. **Error Handling**
   - Detects and reports navigation failures
   - Handles invalid commands gracefully
   - Provides detailed error messages in status updates

### Communication Flow

1. **User Interaction**
   - User selects a waypoint in the Robot Manager GUI or HomeAssistant
   - User triggers navigation (clicks "Go to Selected Waypoint" button)

2. **MQTT Communication**
   - The waypoint manager publishes a message to `yahboom/navigation/command` topic
   - Message contains the waypoint coordinates and orientation

3. **Navigation Execution**
   - b4m_waypoint_nav.py receives the MQTT message
   - It sends the navigation goal to the Navigation2 stack
   - Reports status back via the `yahboom/navigation/status` topic

4. **Status Updates**
   - Navigation progress and completion status are published back to MQTT
   - External systems (like Home Assistant) can monitor these status messages

## Waypoint JSON Structure

The waypoints.json file uses a simplified structure with only one map (yahboom_map):

```json
{
  "yahboom_map": {
    "Waypoint1": {
      "position": { "x": 1.0, "y": 1.0 },
      "orientation": { "x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0 }
    },
    "Door": {
      "position": { "x": 2.0, "y": 2.0 },
      "orientation": { "x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0 }
    },
    "Kitchen": {
      "position": { "x": 3.0, "y": 1.5 },
      "orientation": { "x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0 }
    }
  }
}
```

## Best Practices

1. **Use descriptive waypoint names**: Choose clear, meaningful names for waypoints that describe their location or purpose.

2. **Verify waypoint creation**: After creating a waypoint, test navigation to ensure it works correctly.

3. **Check logs for errors**: If navigation fails, check the waypoint_nav.log file for error messages.

4. **Regular backups**: Periodically back up the waypoints.json file, especially after making significant changes.

5. **Consistent positioning**: Ensure the robot is properly localized before saving waypoints to maintain accuracy.

## Conclusion

By leveraging the Navigation2 framework, adding a central GUI dashboard for waypoint management, implementing MQTT communication for external control, and integrating comprehensive system lifecycle management, we have created a flexible and user-friendly system for robot navigation that can be easily integrated with other systems.

### Key Improvements with Central Launch Control

1. **Simplified Operations**
   - Single application manages entire robot system lifecycle
   - Eliminates need for manual script execution and terminal management
   - Automated launch sequence reduces human error and setup time

2. **Enhanced User Experience**
   - Intelligent button states prevent invalid operations
   - Clear visual feedback about system state and available actions
   - Integrated status displays for both agent and system components

3. **Robust State Management**
   - Hierarchical dependency system ensures proper startup/shutdown order
   - Graceful error handling and recovery mechanisms
   - Process isolation with individual terminal windows for monitoring

4. **Development Efficiency**
   - Parameter modification automatically triggers rebuild detection
   - Real-time system state awareness prevents configuration conflicts
   - Streamlined workflow from development to deployment

5. **System Reliability**
   - Proper process lifecycle management with graceful shutdowns
   - Clear separation between agent-level and system-level operations
   - Comprehensive error logging and status reporting

The supporting scripts for map selection, Docker container management, and the integrated launch control system ensure smooth operation and maintenance of the complete robotic system from a single, unified interface.


