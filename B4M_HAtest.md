# B4M Robot - Home Assistant MQTT Integration

## Quick Start Guide

Follow these steps in order to properly launch the robot navigation system with Home Assistant MQTT integration:

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

### 4. Start RViz for Visualization

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

### 8. Start the B4M Waypoint Navigation Node with MQTT Parameters

```bash
cd /home/yahboom/b4m_yahboom
. install/setup.bash
python3 /home/yahboom/b4m_yahboom/b4m_waypoint_nav/b4m_waypoint_nav/b4m_waypoint_nav.py --ros-args -p mqtt_broker:=192.168.68.111 -p mqtt_port:=1883 -p mqtt_username:=robot -p mqtt_password:=robot123
```

> **IMPORTANT**: This node processes MQTT messages from Home Assistant and sends navigation commands to the robot. Without this node running, navigation commands from Home Assistant will not be executed.

### 9. Configure Home Assistant

Add to your configuration.yaml:

```yaml
button:
  - platform: mqtt
    name: "Robot to Kitchen"
    command_topic: "yahboom/navigation/command"
    payload_press: '{"command": "goto", "waypoint_id": "kitchen", "position": {"x": 1.0, "y": 1.5}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}'

  - platform: mqtt
    name: "Robot to Living Room"
    command_topic: "yahboom/navigation/command"
    payload_press: '{"command": "goto", "waypoint_id": "living_room", "position": {"x": -1.0, "y": 0.5}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.707, "w": 0.707}}'
```

### 10. Test Navigation

1. Restart Home Assistant
2. Add buttons to your dashboard
3. Click buttons to send robot to different locations

## Testing MQTT Functionality

The B4M Waypoint Navigation system supports MQTT for remote control via Home Assistant. To test this functionality:

### 1. MQTT Connection

- The system automatically connects to the MQTT broker specified in the parameters
- Default broker is 192.168.68.111:1883 with username 'robot' and password 'robot123'
- To verify MQTT connection is working:

```bash
cd /home/yahboom/b4m_yahboom
python3 mqtt_test.py
```

Successful output should show:
```
Connected to MQTT broker at 192.168.68.111:1883 with result code 0
Message published
Subscribed to topic
Received message on topic test/connection: Hello from MQTT test client
```

### 2. Send Navigation Commands via Home Assistant

- Use the buttons configured in Home Assistant to send commands
- Commands are published to the topic `yahboom/navigation/command`
- The payload contains the waypoint information in JSON format

### 3. Receive Status Updates

- Subscribe to the topic `yahboom/navigation/status` to receive status updates
- Example using mosquitto_sub:
  ```bash
  mosquitto_sub -h 192.168.68.111 -t "yahboom/navigation/status" -u robot -P robot123
  ```

## Troubleshooting

### Map Not Found
- Ensure maps are in the correct directory: `/home/yahboom/b4m_yahboom/yahboomcar_nav/maps`
- Maps should have both .pgm and .yaml files
- The default map is yahboom_map

### Waypoints Not Found
- Check that the waypoints.json file exists in `/home/yahboom/b4m_yahboom`
- Verify that waypoints have been created for the current map
- Check the ROS2 logs for any error messages

### MQTT Connection Issues
- Verify that the MQTT broker is running in Home Assistant
- Check the broker address (192.168.68.111) and port (1883)
- Ensure username (robot) and password (robot123) are correct
- Check that robot and Home Assistant are on same network
- Use mosquitto_sub to verify that topics are being published correctly

### Navigation Issues
- Make sure robot is properly localized in RViz first
- Verify waypoint coordinates match your environment
- Check waypoint navigation node logs for errors

## File Locations

- **Maps**: `/home/yahboom/b4m_yahboom/yahboomcar_nav/maps`
- **Waypoints**: `/home/yahboom/b4m_yahboom/waypoints.json`
- **B4M Waypoint Nav Code**: `/home/yahboom/b4m_yahboom/b4m_waypoint_nav/b4m_waypoint_nav/`
- **MQTT Test Script**: `/home/yahboom/b4m_yahboom/mqtt_test.py`
