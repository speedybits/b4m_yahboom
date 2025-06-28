# B4M Waypoint Navigation System Launch Guide

This document provides step-by-step instructions for launching and testing the B4M Waypoint Navigation system, including the Waypoint Manager GUI and MQTT functionality.

## Prerequisites

- Yahboom robot with ROS2 Humble
- Docker installed for the Micro-ROS agent
- B4M Waypoint Navigation package built and installed

## Launch Sequence

Follow these steps in order to properly launch the waypoint navigation system:

### 1. Start the Micro-ROS Agent

```bash
docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:humble udp4 --port 8090
```

### 2. Power on the Yahboom Robot

- Turn on the physical robot's power switch
- Wait for the robot to boot up and connect to the Micro-ROS agent
- You should see connection messages in the Micro-ROS agent terminal

### 3. Launch the Car's Underlying Data Processing

```bash
cd /home/yahboom/b4m_yahboom
. install/setup.bash
ros2 launch yahboomcar_bringup yahboomcar_bringup_launch.py
```

### 4. Start RViz for Visualization (Optional)

```bash
cd /home/yahboom/b4m_yahboom
. install/setup.bash
ros2 launch yahboomcar_nav display_launch.py
```

### 5. Launch the Navigation System

```bash
cd /home/yahboom/b4m_yahboom
. install/setup.bash
ros2 launch yahboomcar_nav waypoint_navigation_launch.py maps:=/home/yahboom/b4m_yahboom/yahboomcar_nav/maps/yahboom_map.yaml
```

> **NOTE**: Explicitly specifying the map file path ensures that the correct map is loaded. If you see the wrong map in RViz or get a "Frame [map] does not exist" error, this command should fix the issue by loading the correct map file.

### 6. Verify Map Display in RViz

> **IMPORTANT**: After RViz starts, verify that the correct map is displayed:
> 1. Check that the Fixed Frame in the Global Options panel is set to "map"
> 2. Verify that the Map display is enabled and showing the correct map file (yahboom_map.pgm)
> 3. If the wrong map appears, try the following:
>    - Click on the Map display in the left panel
>    - Change the Topic to "/map" if it's not already set
>    - Set "Draw Behind" to true
>    - If the map still doesn't appear correctly, restart RViz and try again


### 7. Initial Robot Positioning

**IMPORTANT**: The robot must be placed at the exact same starting position and orientation each time.

1. Place the robot at the designated starting position in your environment
2. Use the "2D Pose Estimate" button in RViz to manually set the robot's initial pose
3. Verify in RViz that the red arrow (representing the robot) appears at the correct position on the map

> **Note**: Manual pose setting in RViz is required each time you start the system. Make sure to set the pose accurately to ensure proper navigation.

### 8. Start the B4M Waypoint Navigation Node

```bash
cd /home/yahboom/b4m_yahboom
. install/setup.bash
python3 /home/yahboom/b4m_yahboom/b4m_waypoint_nav/b4m_waypoint_nav/b4m_waypoint_nav.py
```

> **IMPORTANT**: This node processes MQTT messages from the Waypoint Manager GUI and sends navigation commands to the robot. Without this node running, navigation commands from the GUI will not be executed.

### 9. Start the Waypoint Manager GUI

**IMPORTANT**: Always stop ALL existing waypoint manager GUI instances before starting a new one to ensure the latest code changes are used and to prevent conflicts.

```bash
# First, stop ALL existing waypoint manager GUI instances
pkill -9 -f "waypoint_manager_node.py"

# Verify all instances are stopped
ps aux | grep -i waypoint_manager_node.py

# Then start the waypoint manager GUI
cd /home/yahboom/b4m_yahboom
. install/setup.bash
ros2 run b4m_waypoint_nav waypoint_manager_node.py
```

> **IMPORTANT**: The waypoint manager GUI sends coordinate-based navigation commands in JSON format. The navigation node only accepts commands in the format: `{"command": "goto", "waypoint_id": "debug_name", "position": {"x": float, "y": float}, "orientation": {"x": float, "y": float, "z": float, "w": float}}`. Legacy plain text waypoint names are no longer supported.

## Testing the Waypoint Manager GUI

1. **Set and Verify Initial Pose**:
   - After starting the navigation system, use the "2D Pose Estimate" button in RViz to manually set the robot's initial pose
   - Set the pose to match the robot's actual position (X=-0.4019, Y=0.5530, orientation Z=0.0456, W=0.9989)
   - Verify that the red arrow (representing the robot) appears at the correct location on the map

2. **Create Waypoints**:
   - Select a map from the dropdown menu
   - Click "Add Waypoint" and provide a name
   - Use the "Set Current Pose" button to set the waypoint at the robot's current position
   - Alternatively, use the "Set Custom Pose" to manually enter coordinates

2. **Save Waypoints**:
   - Click "Save Waypoints" to store the waypoints to the waypoints.json file
   - Waypoints are saved per map, so you can have different waypoints for different maps

3. **Navigate to Waypoints**:
   - Select a waypoint from the list
   - Click "Navigate to Waypoint" to send the robot to that location
   - The robot will plan a path and navigate to the waypoint

## Testing MQTT Functionality

The B4M Waypoint Navigation system supports MQTT for remote control. To test this functionality:

1. **MQTT Connection**:
   - The system automatically connects to the MQTT broker specified in the parameters
   - Default broker is localhost:1883
   - Connection status is shown in the GUI status bar

2. **Send Navigation Commands via MQTT**:
   - Publish a message to the topic `yahboom/navigation/command` with the waypoint name as the payload
   - Example using mosquitto_pub:
     ```bash
     mosquitto_pub -h localhost -t "yahboom/navigation/command" -m "Living Room"
     ```

3. **Receive Status Updates**:
   - Subscribe to the topic `yahboom/navigation/status` to receive status updates
   - Example using mosquitto_sub:
     ```bash
     mosquitto_sub -h localhost -t "yahboom/navigation/status"
     ```

## Troubleshooting

### Map Not Found
- Ensure maps are in the correct directory: `/home/yahboom/b4m_yahboom/src/yahboomcar_nav/maps`
- Maps should have both .pgm and .yaml files
- The default map is yahboom_map

### Waypoints Not Found
- Check that the waypoints.json file exists in the repository root
- Verify that waypoints have been created for the current map
- Check the ROS2 logs for any error messages

### MQTT Connection Issues
- Verify that the MQTT broker is running
- Check the broker address and port in the parameters
- Use mosquitto_sub to verify that topics are being published correctly

## File Locations

- **Maps**: `/home/yahboom/b4m_yahboom/src/yahboomcar_nav/maps`
- **Waypoints**: `/home/yahboom/b4m_yahboom/waypoints.json`
- **B4M Waypoint Nav Code**: `/home/yahboom/b4m_yahboom/b4m_waypoint_nav/b4m_waypoint_nav/`
