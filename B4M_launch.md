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
ros2 launch yahboomcar_nav waypoint_navigation_launch.py
```

### 6. Set Initial Pose Estimate in RViz

**CRITICAL STEP**: The robot must be properly localized before navigation commands will be accepted.

1. In the RViz window, click on the "2D Pose Estimate" button in the top toolbar
2. Click and drag on the map to set the initial pose:
   - Click where the robot is currently located on the map
   - Drag in the direction the robot is facing
3. Verify that the red arrow (representing the robot) appears at the correct position on the map

> **Note**: Without setting the initial pose, all navigation commands will be rejected by the navigation stack because it doesn't know the robot's starting position.

### 7. Start the Waypoint Manager GUI

```bash
cd /home/yahboom/b4m_yahboom
. install/setup.bash
ros2 run b4m_waypoint_nav waypoint_manager_node.py
```

## Testing the Waypoint Manager GUI

1. **Create Waypoints**:
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
