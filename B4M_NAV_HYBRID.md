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
   Option A: Incremental Movement
   LLM → POST /move → Navigation Controller → /cmd_vel → Robot Movement
   
   Option B: Navigation2 Goal Pose
   LLM → POST /navigate_to → Nav2 Action Server → Path Planner → Robot Movement

3. LLM monitors progress
   LLM → GET /status → Current State → LLM Feedback Loop
```

## Navigation2 Integration with Grid System

### How Nav2 Enhances Grid-Based Navigation

#### 1. Cost Maps vs Simple Grid
While the LLM sees a simplified grid (`.`, `#`, `?`, `@`, `*`), Navigation2 maintains sophisticated cost maps:

```
LLM Grid View:          Nav2 Cost Map (0-254 values):
#  #  #  #  #          254 254 254 254 254  (Lethal obstacle)
#  .  .  .  #          254  50  25  50 254  (Inflation around walls)
#  .  @  .  #          254  25   0  25 254  (Free space gradients)
#  .  .  .  #          254  50  25  50 254  (Safety margins)
#  #  #  #  #          254 254 254 254 254  (Walls)
```

#### 2. Path Planning Algorithms
Nav2 provides multiple planners the LLM can leverage:
- **NavFn/A***: Global path planning for optimal routes
- **DWB/TEB**: Local planning for smooth trajectories
- **Theta***: Any-angle path planning for natural movement

#### 3. Grid to Nav2 Coordinate Transformation
```python
def grid_to_nav2_transform(grid_pos, grid_info):
    """
    Transform grid coordinates to Nav2 world coordinates
    Accounts for map origin and resolution
    """
    # Grid uses top-left origin, Nav2 uses bottom-left
    world_x = grid_info["origin"]["x"] + (grid_pos[0] * grid_info["resolution"])
    world_y = grid_info["origin"]["y"] + (grid_pos[1] * grid_info["resolution"])
    
    return {"x": world_x, "y": world_y}
```

#### 4. Dynamic Replanning
When LLM sets a Nav2 goal, the system continuously:
- Updates path if new obstacles detected (grid `#` appears)
- Adjusts for moving obstacles
- Triggers recovery behaviors if stuck

### Navigation Mode Decision Matrix

| Scenario | Grid State | Navigation Method | Reason |
|----------|------------|-------------------|---------|
| Exploration | >40% `?` symbols | Incremental | Need careful exploration |
| Known path | <20% `?`, clear `.` path | Nav2 Goal | Efficient path planning |
| Obstacle maze | Many `#` symbols | Nav2 Goal | Better obstacle avoidance |
| Precise positioning | Target within 1m | Incremental | Fine control needed |
| Long distance | Target >5m away | Nav2 Goal | Optimal path planning |
| Doorway navigation | Single `.` between `#` | Hybrid | Nav2 to door, incremental through |

## Text-Based Spatial Representation

The system converts occupancy grids into natural language descriptions:

### Grid Symbol Mapping
- `.` - Free space
- `#` - Obstacle/wall
- `?` - Unknown/unexplored area
- `@` - Robot position
- `*` - Target/goal position

### Detailed Grid Symbol Usage Example

#### Example Occupancy Grid (5m x 5m area, 0.5m resolution)
```
    0   1   2   3   4   5   6   7   8   9
0   #   #   #   #   #   #   #   #   #   #
1   #   .   .   .   .   #   ?   ?   ?   #
2   #   .   .   .   .   #   ?   ?   ?   #
3   #   .   .   @   .   .   .   .   ?   #
4   #   .   .   .   .   #   .   .   ?   #
5   #   #   #   .   #   #   .   .   ?   #
6   #   .   .   .   .   .   .   .   ?   #
7   #   .   .   *   .   #   #   #   #   #
8   #   .   .   .   .   #   ?   ?   ?   #
9   #   #   #   #   #   #   #   #   #   #
```

**Symbol Locations:**
- `@` at (3,3): Robot's current position
- `*` at (3,7): Target/goal position
- `#`: Walls and obstacles detected by SLAM
- `.`: Free navigable space
- `?`: Unexplored areas (no sensor data yet)

#### Grid-to-Text Conversion
The Spatial Map Interpreter analyzes this grid and generates:

```json
{
  "text_description": "You are located at coordinates (3,3) in a partially mapped room. 
    Forward (heading south): Clear path for 2.0m until a doorway at (3,5). Beyond the 
    doorway is another room extending 2.5m with your target at (3,7).
    Left (heading east): Clear space for 1.5m, then unexplored area.
    Right (heading west): Immediate wall at 0.5m.
    Behind (heading north): Clear path for 1.0m to wall.
    Unexplored regions detected to the east (right side of map).",
    
  "grid_analysis": {
    "robot_symbol": "@",
    "robot_grid_position": [3, 3],
    "target_symbol": "*", 
    "target_grid_position": [3, 7],
    "path_exists": true,
    "obstacles_between": ["doorway at [3,5]"],
    "unexplored_percentage": 30
  }
}
```

#### LLM Navigation Decision Process

1. **Path Analysis**: LLM examines symbols between @ and *
   - Path from (3,3) to (3,7): @ → . → . → doorway → . → . → *
   - All symbols are "." (free space) except for the doorway

2. **Navigation Strategy Selection**:
   
   **Option A: Navigation2 Goal Pose (Preferred for known areas)**
   ```python
   # LLM analyzes grid and sets direct 2D goal pose
   if path_is_clear and all_areas_mapped:
       # Use Nav2 to handle path planning automatically
       navigation_request = {
           "target": {
               "x": 3.5,  # Grid position 7 * 0.5m resolution
               "y": 1.5,  # Grid position 3 * 0.5m resolution  
               "orientation": 3.14  # Face south (π radians)
           },
           "label": "target_location"
       }
       # Nav2 handles obstacle avoidance and path planning
       response = requests.post("http://robot:8080/navigate_to", json=navigation_request)
   ```
   
   **Option B: Incremental Movement (For exploration or complex obstacles)**
   ```
   Initial:          After Analysis:     LLM Commands:
   #   #   #   #     #   #   #   #      1. Move right (1.0m)
   #   .   @   #     #   .   →   #      2. Move forward (2.0m)
   #   #   .   #     #   #   ↓   #      3. Move left (1.0m)
   #   .   .   #     #   ←   .   #      4. Move forward to *
   #   .   *   #     #   .   *   #
   ```

3. **Hybrid Navigation Approach**:
   ```python
   # LLM Decision Tree
   def select_navigation_method(grid_analysis, target):
       if grid_analysis["unexplored_percentage"] > 40:
           # Too much unknown - use incremental exploration
           return "incremental"
       elif grid_analysis["path_exists"] and not grid_analysis["obstacles_between"]:
           # Clear path - use Nav2 direct goal
           return "nav2_goal"
       elif grid_analysis["obstacles_between"]:
           # Complex obstacles - let Nav2 plan around them
           return "nav2_goal"  # Nav2 handles obstacle avoidance
       else:
           # Default to incremental for fine control
           return "incremental"
   ```

4. **Navigation2 Integration Benefits**:
   - **Automatic Path Planning**: Nav2 uses A* or other algorithms to find optimal path
   - **Dynamic Obstacle Avoidance**: Real-time adjustments for moving obstacles
   - **Recovery Behaviors**: Built-in recovery when stuck or path blocked
   - **Cost Maps**: Considers traversal costs, not just free/occupied

5. **Pattern Recognition**:
   - **Corridor**: Sequential '.' symbols between walls → "hallway"
   - **Doorway**: Single '.' between '#' symbols → "doorway"
   - **Room**: Enclosed area of '.' symbols → "room"

6. **Exploration Strategy**:
   - '?' symbols indicate unexplored areas
   - LLM prioritizes exploring unknown regions
   - Updates path plans as '?' becomes '.' or '#'

#### Real-time Grid Updates

**Before movement:**
```
#   .   .   ?   ?
#   @   .   ?   ?    30% unexplored
#   .   .   ?   ?
```

**After exploring right:**
```
#   .   .   .   #
#   .   @   .   #    10% unexplored
#   .   .   .   #    New obstacles discovered
```

#### Configuration Impact
With `grid_resolution: 0.5` (from config):
- Each grid cell = 0.5m × 0.5m
- 10×10 grid = 5m × 5m physical space
- Cells >65% occupancy → '#' (obstacle)
- Cells <65% occupancy → '.' (free)

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

### Example 4: Navigation2 Grid-Based Goal Setting
```python
# LLM receives grid and converts to Nav2 goal
def grid_to_nav2_goal(grid_analysis, target_grid_pos, grid_resolution=0.5):
    """
    Convert grid position to Navigation2 2D Goal Pose
    Grid origin is top-left, Nav2 uses bottom-left with meters
    """
    # Convert grid coordinates to meters
    target_x = target_grid_pos[0] * grid_resolution
    target_y = target_grid_pos[1] * grid_resolution
    
    # Calculate orientation towards target
    robot_pos = grid_analysis["robot_grid_position"]
    dx = target_grid_pos[0] - robot_pos[0]
    dy = target_grid_pos[1] - robot_pos[1]
    target_orientation = math.atan2(dy, dx)
    
    return {
        "x": target_x,
        "y": target_y,
        "orientation": target_orientation
    }

# Example: LLM analyzes grid and navigates to unexplored area
context = requests.get("http://robot:8080/spatial_context").json()
grid_analysis = context["grid_analysis"]

# Find nearest unexplored area (? symbol)
if grid_analysis["unexplored_percentage"] > 10:
    # Convert grid position of unexplored area to Nav2 goal
    unexplored_pos = [6, 2]  # Example: found '?' at grid position (6,2)
    nav2_goal = grid_to_nav2_goal(grid_analysis, unexplored_pos)
    
    # Send Nav2 goal - let Navigation2 handle path planning
    navigation_request = {
        "target": nav2_goal,
        "label": "explore_unknown_area",
        "use_nav2": True  # Explicitly use Navigation2 stack
    }
    response = requests.post("http://robot:8080/navigate_to", json=navigation_request)
    
    # Nav2 automatically:
    # - Plans optimal path using A* or DWB
    # - Avoids obstacles dynamically
    # - Handles recovery if stuck
    # - Updates path if new obstacles detected
```

### Example 5: Intelligent Navigation Mode Selection
```python
def llm_navigate_to_target(spatial_context, target_symbol="*"):
    """
    LLM intelligently selects between Nav2 goal pose or incremental movement
    """
    grid = spatial_context["grid_analysis"]
    
    # Check if target exists in grid
    if not grid.get("target_grid_position"):
        return {"error": "No target found in grid"}
    
    target_pos = grid["target_grid_position"]
    robot_pos = grid["robot_grid_position"]
    
    # Decision criteria for navigation method
    distance = math.sqrt((target_pos[0]-robot_pos[0])**2 + 
                        (target_pos[1]-robot_pos[1])**2)
    
    if distance < 2.0:  # Close range - use incremental
        return {
            "method": "incremental",
            "reason": "Target very close, need precise control",
            "commands": [
                {"command": "move", "direction": "forward", "distance": 0.5},
                {"command": "move", "direction": "left", "distance": 0.3}
            ]
        }
    elif grid["unexplored_percentage"] < 20:  # Well-mapped area
        # Use Nav2 for efficient path planning
        nav2_goal = grid_to_nav2_goal(grid, target_pos)
        return {
            "method": "nav2_goal", 
            "reason": "Area well-mapped, Nav2 can plan optimal path",
            "navigation_request": {
                "target": nav2_goal,
                "label": "direct_to_target"
            }
        }
    else:  # Partially explored - hybrid approach
        # Navigate to nearest known area first
        intermediate_pos = find_nearest_explored_point(grid, target_pos)
        nav2_goal = grid_to_nav2_goal(grid, intermediate_pos)
        return {
            "method": "hybrid",
            "reason": "Partially explored - navigate to known area first",
            "navigation_request": {
                "target": nav2_goal,
                "label": "intermediate_waypoint"
            },
            "follow_up": "incremental_exploration"
        }
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