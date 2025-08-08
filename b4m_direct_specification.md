# B4M Direct Vision-Based Navigation Specification

## Overview

This specification describes the integration of the B4M API with the Yahboom robot's camera vision system to enable intelligent, vision-based autonomous navigation. The system captures visual information from the robot's ESP32 camera, describes what it sees to the B4M AI API, and uses the AI's response to make navigation decisions.

## Development Approach

### Initial Development in Simulation

The B4M Direct system will be initially developed and tested entirely in the Gazebo Classic simulation environment before deployment to the physical robot. This approach ensures:
- Safe testing without risk to hardware
- Repeatable test scenarios
- Faster development iteration
- Consistent debugging environment

#### Launch Command
```bash
./b4m_HA_launch.sh --b4m-direct --simulation
```

This command will:
1. Launch Gazebo Classic simulation with virtual robot
2. Start all navigation and sensor systems in simulation mode
3. Initialize the B4M Direct vision navigation system
4. Provide simulated camera feed from Gazebo

#### Simulation-Specific Configuration
In simulation mode, the system will:
- Use Gazebo's simulated RGB camera plugin with 120° field of view
- Camera topic: `/camera/image_raw` (simulation) instead of `/espRos/esp32camera`
- Camera mounted at front of robot, 0.2m height, 0° tilt
- Simulated laser scanner at `/scan`
- Virtual odometry from Gazebo's differential drive plugin
- Use `use_sim_time:=true` for all ROS2 nodes

## System Architecture

### Components

1. **Vision Input**
   - **Physical Robot**: ESP32 camera module provides compressed images
     - Topic: `/espRos/esp32camera` (CompressedImage)
     - Processed images available on: `/esp32_img` (Image)
   - **Simulation**: Gazebo camera sensor plugin
     - Topic: `/camera/image_raw` (Image)
     - Direct RGB images from simulated environment
   - Resolution: 640x480

2. **B4M API Integration**
   - Endpoint: `https://app.bike4mind.com/api/chat`
   - API Key: `b4m_live_c491719bd23cc716e2db2c5182f4f900`
   - Model: `gpt-4o-mini`
   - Purpose: Interpret visual scenes and provide navigation guidance

3. **Movement Control**
   - Output topic: `/cmd_vel` (Twist)
   - Linear and angular velocity commands
   - Integration with existing obstacle avoidance (laser scan)

4. **Safety Layer**
   - Laser scan topic: `/scan` (LaserScan)
   - Ensures obstacle avoidance overrides vision-based commands
   - Stop distance: 30.48cm (1 foot)

## Vision-Based Navigation Pipeline

### 1. Image Capture and Processing
```python
# Subscribe to camera feed
/espRos/esp32camera -> CompressedImage
# Convert to OpenCV format
cv_image = bridge.compressed_imgmsg_to_cv2(msg)
# Resize to standard resolution
cv_image = cv.resize(cv_image, (640, 480))
```

### 2. Scene Analysis
The system will generate simple, descriptive scene descriptions:
- Basic object identification: "Human 2 meters ahead", "Wall on right"
- Path availability: "Clear path to the left", "Open space ahead"
- Obstacle detection: "Obstacle blocking path", "Object 1 meter ahead"
- General observations: "Narrow passage", "Wide open area"

Initial implementation will use simple computer vision techniques:
- Color-based segmentation for basic object detection
- Edge detection for identifying walls and boundaries
- Motion detection for identifying humans or moving objects
- Distance estimation based on object size in frame

### 3. API Request Format
```json
{
  "message": "I am a mobile robot. I see: [scene description]. Where should I go? Respond with simple navigation commands like 'move forward', 'turn left', 'turn right', or 'stop'.",
  "model": "gpt-4o-mini",
  "temperature": 0.3,
  "max_tokens": 100
}
```

Scene description should include:
- Main objects visible in the center of view
- Clear paths or openings
- Obstacles or barriers
- Any text or signs visible

### 4. API Response Processing
Currently, the API returns free-text responses. The system will parse these for navigation commands.

**Planned enhancement**: API will be modified to return structured JSON:
```json
{
  "response": "turn left",
  "confidence": 0.85,
  "reasoning": "Open hallway visible on the left"
}
```

**Current implementation**: Parse free-text for keywords like "forward", "left", "right", "stop"

### 5. Command Mapping

| API Response | Linear Velocity (m/s) | Angular Velocity (rad/s) | Duration (s) |
|-------------|----------------------|-------------------------|--------------|
| "move forward" | 0.08 | 0.0 | continuous |
| "turn left" | 0.0 | 0.3 | 2.0 |
| "turn right" | 0.0 | -0.3 | 2.0 |
| "stop" | 0.0 | 0.0 | until cleared |
| "move slowly" | 0.04 | 0.0 | continuous |
| "sharp left" | 0.0 | 0.5 | 3.0 |
| "sharp right" | 0.0 | -0.5 | 3.0 |
| "backup" | -0.04 | 0.0 | 2.0 |

## ROS2 Node Structure

### Node: `b4m_direct_navigator`

#### Publishers
- `/cmd_vel` (geometry_msgs/Twist) - Movement commands
- `/b4m_direct/status` (std_msgs/String) - System status
- `/b4m_direct/debug_image` (sensor_msgs/Image) - Annotated images

#### Subscribers
- `/espRos/esp32camera` (sensor_msgs/CompressedImage) - Camera input
- `/scan` (sensor_msgs/LaserScan) - Obstacle detection
- `/b4m_direct/command` (std_msgs/String) - Manual override commands

#### Services
- `/b4m_direct/enable` - Enable/disable vision navigation
- `/b4m_direct/set_mode` - Switch between exploration/navigation modes

### Configuration Parameters

```yaml
b4m_direct_navigator:
  ros__parameters:
    # API Configuration
    api_endpoint: "https://app.bike4mind.com/api/chat"
    api_key: "b4m_live_c491719bd23cc716e2db2c5182f4f900"
    model: "gpt-4o-mini"
    temperature: 0.3
    max_tokens: 100
    
    # Vision Processing
    image_rate: 1.0  # Hz - how often to process images
    image_resize: [640, 480]
    
    # Movement Parameters
    linear_speed_default: 0.08  # m/s
    angular_speed_default: 0.3  # rad/s
    stop_distance: 0.3048  # meters (1 foot)
    
    # Behavior
    enable_obstacle_override: true
    exploration_mode: false
    debug_mode: false
```

## Implementation Phases

### Phase 0: Simulation Setup and Validation
- Configure Gazebo Classic world with test obstacles
- Verify camera sensor plugin integration
- Test image capture from simulated camera
- Validate movement commands in simulation
- Establish baseline performance metrics

### Phase 1: Basic Integration (Simulation)
- Camera image capture from Gazebo
- Simple scene description generation
- API communication testing
- Basic movement command execution in simulation
- Verify obstacle avoidance with simulated laser

### Phase 2: Enhanced Vision Processing (Simulation)
- Object detection using OpenCV on simulated images
- Path detection algorithms for Gazebo environments
- Scene segmentation testing
- Improved scene descriptions with simulated landmarks

### Phase 3: Real Robot Transition
- Adapt system for physical ESP32 camera
- Calibrate for real-world lighting conditions
- Tune movement parameters for physical robot
- Validate safety mechanisms on hardware

### Phase 4: Intelligent Navigation
- Context-aware navigation decisions
- Memory of previous locations
- Goal-oriented navigation
- Learning from navigation success/failure

### Phase 5: Multi-Modal Integration
- Combine vision with laser scan data
- Integrate with existing SLAM mapping
- Waypoint navigation enhancement
- MQTT command interface

## Safety Considerations

1. **Obstacle Override**
   - Laser scan always takes precedence
   - Stop immediately if obstacle detected < 30.48cm
   - Reduce speed in cluttered environments

2. **API Failure Handling**
   - Default to safe stop on API timeout
   - Stop robot on unclear or unparseable API responses
   - No fallback navigation - robot stops and waits
   - Rate limiting to prevent API overuse (1 Hz max)

3. **Movement Validation**
   - Verify commands are within safe limits
   - Smooth acceleration/deceleration
   - Emergency stop capability

## Testing Procedures

### Simulation Tests (Phase 0-2)
1. Gazebo world setup validation
2. Camera feed acquisition from simulation
3. API integration with simulated scenes
4. Movement command execution in Gazebo
5. Obstacle detection and avoidance
6. SLAM integration in simulation

### Unit Tests
1. Image capture and processing
2. API request/response handling
3. Command mapping accuracy
4. Safety override mechanisms

### Integration Tests (Simulation First)
1. End-to-end vision to movement pipeline in Gazebo
2. Obstacle avoidance during vision navigation
3. Mode switching and manual override
4. Performance with various simulated environments

### Hardware Tests (Phase 3+)
1. ESP32 camera integration
2. Real-world lighting adaptation
3. Physical robot movement calibration
4. Safety validation on hardware

### System Tests
1. Navigation in known environments
2. Exploration of unknown spaces
3. Long-duration operation stability
4. Multi-robot coordination (future)

## Performance Metrics

- Image processing latency: < 100ms
- API response time: < 500ms
- Navigation decision rate: 1-2 Hz
- Obstacle detection response: < 50ms
- Success rate for navigation tasks: > 80%

## Future Enhancements

1. **Advanced Vision Models**
   - Local vision models for faster processing
   - Semantic segmentation
   - 3D scene understanding

2. **Learning Capabilities**
   - Reinforcement learning for navigation
   - Scene memory and recognition
   - Personalized navigation preferences

3. **Multi-Robot Coordination**
   - Shared vision information
   - Collaborative exploration
   - Distributed decision making

## Example Usage

### Simulation Mode Launch
```bash
# Launch complete system with B4M Direct in simulation
# This will automatically handle all launch steps, skipping Robot Manager GUI
./b4m_HA_launch.sh --b4m-direct --simulation

# Or launch individual components for testing:
# 1. Start Gazebo Classic simulation
ros2 launch yahboomcar_nav gazebo_classic_nav_launch.py

# 2. Launch B4M Direct navigator in simulation mode
ros2 run b4m_direct b4m_direct_navigator --ros-args -p simulation_mode:=true

# 3. Enable vision navigation
ros2 service call /b4m_direct/enable std_srvs/srv/SetBool "{data: true}"
```

### Physical Robot Launch
```bash
# Launch with real robot (after simulation testing)
./b4m_HA_launch.sh --b4m-direct --skip-agent

# Launch the vision navigator
ros2 run b4m_direct b4m_direct_navigator

# Enable vision navigation
ros2 service call /b4m_direct/enable std_srvs/srv/SetBool "{data: true}"
```

### Monitoring and Debugging
```bash
# Monitor status
ros2 topic echo /b4m_direct/status

# View camera feed (simulation)
ros2 run rqt_image_view rqt_image_view /camera/image_raw

# View camera feed (real robot)
ros2 run rqt_image_view rqt_image_view /esp32_img

# View debug images with API annotations
ros2 run rqt_image_view rqt_image_view /b4m_direct/debug_image
```

## Dependencies

- ROS2 Humble
- OpenCV 4.x
- Python packages:
  - requests
  - numpy
  - cv_bridge
  - Pillow
- Hardware:
  - ESP32 camera module
  - Laser scanner
  - Computing platform with network access

## Notes

- Initial development and testing will be done entirely in Gazebo Classic simulation
- The `--b4m-direct` flag for `b4m_HA_launch.sh` will need to be implemented to support this mode
  - This flag will automatically handle all launch steps
  - Robot Manager GUI and other non-essential steps will be skipped
  - System will proceed directly to vision-based navigation mode
- Simulation provides a safe, repeatable environment for API integration testing
- Camera configuration: RGB only, 120° field of view, front-mounted at 0.2m height
- Camera topics differ between simulation (`/camera/image_raw`) and real robot (`/espRos/esp32camera`)
- Fallback behavior: Robot stops on unclear/failed API responses (no autonomous fallback)
- Scene descriptions will be simple and descriptive (e.g., "Human 2 meters ahead", "Clear path to the left")
- API response parsing will initially handle free-text, later structured JSON
- Vision processing rate fixed at 1 Hz regardless of robot speed
- No learning/training mode - system operates with fixed algorithms
- The system is designed to work in both simulation (Gazebo Classic) and real robot modes
- API calls are rate-limited to prevent excessive usage
- All navigation decisions can be overridden by manual control
- System logs all API interactions for debugging and analysis
- Gazebo simulation allows testing without physical robot hardware
- Performance metrics from simulation will establish baselines for real robot deployment