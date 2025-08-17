# B4M LLM Navigation System Documentation

## Overview

The B4M LLM Navigation System enables natural language communication between Large Language Models (LLMs) and the Yahboom robot's SLAM-based navigation system. This system translates occupancy grid maps and sensor data into text descriptions that LLMs can understand, and converts LLM decisions into robot movement commands.

## System Architecture

### Core Components

#### 1. Spatial Map Interpreter Node (`b4m_spatial_interpreter`)
Converts ROS2 navigation data into LLM-understandable text descriptions.

**Subscriptions:**
- `/map` (nav_msgs/OccupancyGrid) - SLAM-generated occupancy grid
- `/amcl_pose` (geometry_msgs/PoseWithCovarianceStamped) - Robot's current position
- `/scan` (sensor_msgs/LaserScan) - LiDAR sensor data

**Functionality:**
- Converts occupancy grid to text-based spatial descriptions
- Generates relative spatial context (forward/left/right/back sectors)
- Analyzes obstacle patterns and free space
- Provides spatial state via ROS service for API queries

#### 2. LLM Navigation API Server (`b4m_llm_nav_api`)
REST API server providing the interface between LLM services and the robot.

**Technology:** FastAPI with async support
**Default Port:** 8080

**Endpoints:**
- `GET /spatial_context` - Returns current spatial situation in text
- `POST /move` - Execute movement command from LLM
- `GET /scan_data` - Returns processed LiDAR information
- `POST /navigate_to` - Navigate to specific coordinates
- `GET /status` - Current navigation and robot status

#### 3. Navigation Controller Node (`b4m_llm_controller`)
Bridges API commands to ROS2 navigation stack.

**Publishers:**
- `/cmd_vel` (geometry_msgs/Twist) - Direct velocity commands
- `/navigate_to_pose` (geometry_msgs/PoseStamped) - Navigation goals

**Functionality:**
- Translates simple movement commands (forward/left/right/back)
- Implements safety validation before executing commands
- Monitors navigation progress and reports status

## API Specification

### GET /spatial_context
Returns the robot's current spatial understanding in text format.

**Response:**
```json
{
  "position": {
    "x": 2.5,
    "y": 3.2,
    "heading": 1.57
  },
  "text_description": "You are in an open area. Forward: clear path for 4.5m. Left: wall at 0.8m. Right: doorway at 2.1m. Behind: open space extends 3m.",
  "obstacles": [
    {
      "direction": "forward-left",
      "distance": 1.5,
      "description": "table-sized object"
    },
    {
      "direction": "right",
      "distance": 2.0,
      "description": "small obstacle"
    }
  ],
  "map_stats": {
    "explored_percentage": 65,
    "current_zone": "central_area"
  }
}
```

### POST /move
Execute a movement command.

**Request:**
```json
{
  "command": "move",
  "direction": "forward",  // Options: "forward", "left", "right", "back", "stop"
  "distance": 2.0,          // Optional, in meters
  "speed": "normal"         // Optional: "slow", "normal", "fast"
}
```

**Response:**
```json
{
  "status": "executing",
  "estimated_duration": 4.5,
  "message": "Moving forward 2.0 meters"
}
```

### POST /navigate_to
Navigate to specific map coordinates.

**Request:**
```json
{
  "target": {
    "x": 5.2,
    "y": 3.8,
    "orientation": 0.785  // Optional, in radians
  },
  "label": "kitchen"      // Optional, for logging
}
```

**Response:**
```json
{
  "status": "navigating",
  "distance_to_target": 3.4,
  "estimated_time": 15.2
}
```

### GET /scan_data
Returns processed LiDAR scan information.

**Response:**
```json
{
  "timestamp": "2024-01-15T10:30:00Z",
  "sectors": {
    "front": {"min_distance": 2.5, "clear": true},
    "left": {"min_distance": 0.8, "clear": false},
    "right": {"min_distance": 3.2, "clear": true},
    "back": {"min_distance": 1.5, "clear": true}
  },
  "obstacle_count": 3,
  "scan_quality": "good"
}
```

### GET /status
Returns current robot and navigation status.

**Response:**
```json
{
  "robot_state": "idle",  // Options: "idle", "moving", "navigating", "stopped"
  "battery_level": 85,
  "last_command": "move forward",
  "navigation_status": {
    "active": false,
    "goal": null,
    "progress": 0
  },
  "safety_status": {
    "emergency_stop": false,
    "obstacles_detected": false
  }
}
```

## Communication Flow

```
1. LLM requests spatial context
   LLM → GET /spatial_context → Spatial Interpreter → Text Description

2. LLM makes movement decision
   LLM → POST /move → Navigation Controller → Robot Movement

3. LLM monitors progress
   LLM → GET /status → Current State → LLM Feedback Loop
```

## Text-Based Spatial Representation

The system converts occupancy grids into natural language descriptions:

### Grid Symbol Mapping
- `.` - Free space
- `#` - Obstacle/wall
- `?` - Unknown/unexplored area
- `@` - Robot position
- `*` - Target/goal position

### Example Text Description
```
"You are in a medium-sized room approximately 5m x 4m. There is a clear path forward 
extending 3.2m before reaching a wall. To your left at 0.8m is a solid wall running 
parallel to your current heading. On your right at 2.1m is an opening that appears 
to be a doorway leading to another area. Behind you is open space for about 2m before 
reaching scattered furniture-like obstacles."
```

### Relative Direction System
- **Forward**: 0° (robot's current heading)
- **Left**: 90° counter-clockwise
- **Right**: 90° clockwise
- **Back**: 180° from current heading

## Implementation Structure

```
b4m_llm_nav_api/
├── b4m_llm_nav_api/
│   ├── __init__.py
│   ├── api_server.py           # FastAPI server implementation
│   ├── spatial_interpreter.py   # Map to text conversion
│   ├── nav_controller_node.py  # ROS2 navigation bridge
│   └── utils/
│       ├── map_utils.py        # Occupancy grid processing
│       └── text_generator.py   # Natural language generation
├── launch/
│   └── llm_nav_api_launch.py   # ROS2 launch file
├── config/
│   └── llm_nav_api.yaml        # Configuration parameters
├── tests/
│   ├── test_api_endpoints.py   # API unit tests
│   ├── test_spatial_interpreter.py
│   └── test_navigation.py
├── package.xml
└── setup.py
```

## Configuration Parameters

```yaml
# config/llm_nav_api.yaml
api_server:
  host: "0.0.0.0"
  port: 8080
  cors_enabled: true
  max_request_size: 1048576  # 1MB
  
spatial_interpreter:
  grid_resolution: 0.5        # meters per grid cell for text generation
  max_range: 10.0            # maximum distance to report in descriptions
  sector_angle: 30           # degrees for sector analysis
  obstacle_threshold: 0.65   # occupancy probability for obstacle
  
navigation:
  linear_speed_slow: 0.1     # m/s
  linear_speed_normal: 0.2   # m/s
  linear_speed_fast: 0.3     # m/s
  angular_speed: 0.5         # rad/s
  safety_distance: 0.3       # minimum obstacle distance in meters
  timeout: 30.0              # navigation timeout in seconds
  
text_generation:
  verbosity: "normal"        # Options: "minimal", "normal", "detailed"
  include_distances: true
  include_dimensions: true
  coordinate_format: "relative"  # Options: "relative", "absolute"
```

## Testing Strategy

### Unit Tests
- Spatial interpretation accuracy
- Text generation consistency
- API endpoint validation
- Safety boundary checks

### Integration Tests
- Gazebo simulation environment tests
- SLAM map interpretation tests
- Navigation command execution
- LiDAR data processing

### System Tests
- End-to-end LLM interaction scenarios
- Multi-step navigation sequences
- Error recovery testing
- Performance benchmarks

### Test Commands
```bash
# Run unit tests
colcon test --packages-select b4m_llm_nav_api

# Run integration tests with simulation
./b4m_launch.sh --simulation
ros2 run b4m_llm_nav_api test_integration

# Run full system test
./tests/test_llm_navigation_system.sh
```

## Example LLM Interactions

### Example 1: Basic Exploration
```python
# LLM requests current situation
response = requests.get("http://robot:8080/spatial_context")
# Response: "You are in an open area. Forward: clear path for 4.5m..."

# LLM decides to explore forward
command = {"command": "move", "direction": "forward", "distance": 2.0}
response = requests.post("http://robot:8080/move", json=command)

# LLM checks status after movement
status = requests.get("http://robot:8080/status")
```

### Example 2: Obstacle Avoidance
```python
# LLM detects obstacle in spatial context
context = requests.get("http://robot:8080/spatial_context").json()
if "obstacle" in context["text_description"]:
    # Choose alternate direction
    if "left" in context["text_description"] and "clear" in context["text_description"]:
        command = {"command": "move", "direction": "left"}
        requests.post("http://robot:8080/move", json=command)
```

### Example 3: Coordinate Navigation
```python
# LLM has stored a location and wants to return
stored_location = {"x": 2.5, "y": 3.2, "orientation": 0.0}
navigation_request = {
    "target": stored_location,
    "label": "starting_position"
}
response = requests.post("http://robot:8080/navigate_to", json=navigation_request)

# Monitor navigation progress
while True:
    status = requests.get("http://robot:8080/status").json()
    if status["robot_state"] != "navigating":
        break
    time.sleep(1)
```

## Safety Considerations

1. **Command Validation**: All movement commands are validated against safety boundaries
2. **Obstacle Detection**: Continuous monitoring of LiDAR data prevents collisions
3. **Emergency Stop**: System supports immediate stop commands
4. **Timeout Protection**: All navigation goals have configurable timeouts
5. **Speed Limits**: Maximum speeds are enforced regardless of LLM requests

## Integration with Existing System

The LLM navigation system integrates seamlessly with the existing B4M Yahboom infrastructure:

- Uses the same `/map` and `/amcl_pose` topics as current navigation
- Compatible with both Gmapping and Cartographer SLAM
- Preserves existing waypoint navigation functionality
- Leverages the established safety systems and sensors
- Can be enabled/disabled without affecting core navigation

## Future Enhancements

1. **Visual descriptions**: Integration with camera data for richer spatial context
2. **3D spatial understanding**: Incorporate height and vertical obstacles
3. **Semantic mapping**: Object recognition and labeling
4. **Multi-robot coordination**: LLM controlling multiple robots
5. **Learning from exploration**: Building knowledge base over time

## Troubleshooting

### Common Issues

1. **No spatial context returned**
   - Check that SLAM is running and publishing to `/map`
   - Verify AMCL localization is active

2. **Movement commands not executing**
   - Ensure navigation stack is properly launched
   - Check safety distance parameters

3. **API connection refused**
   - Verify API server is running: `ros2 run b4m_llm_nav_api api_server`
   - Check firewall settings for port 8080

## License and Attribution

This system is part of the B4M Yahboom robot project.
Developed for integration with SLAM-based navigation and LLM decision making.