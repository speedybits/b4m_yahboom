# B4M LLM Navigation System Documentation

## Overview

The B4M LLM Navigation System enables natural language communication between Large Language Models (LLMs) and the Yahboom robot's SLAM-based navigation system. This system translates occupancy grid maps and sensor data into text descriptions that LLMs can understand, and converts LLM decisions into robot movement commands.

**Important:** The LLM communicates with the robot exclusively through JSON over HTTP. The LLM does not run any code on the robot - it only sends navigation requests as JSON and receives status updates as JSON. All ROS2 and Navigation2 integration happens on the robot side via the API server.

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
- `GET /spatial_context` - Returns current spatial situation in text and grid
- `POST /navigate_to` - Navigate using delta movement or absolute coordinates
- `GET /status` - Current navigation and robot status with detailed Nav2 feedback
- `POST /emergency_stop` - Immediately stop robot and cancel navigation goals
- `POST /enable_simulated_llm` - Enable/disable simulated LLM test mode
- `GET /simulated_llm_decision` - Get next simulated LLM navigation decision

#### 3. Navigation Controller Node (`b4m_llm_controller`)
Bridges API commands to ROS2 Navigation2 stack.

**Publishers:**
- `/navigate_to_pose` (geometry_msgs/PoseStamped) - Navigation2 goals

**Functionality:**
- Converts LLM grid analysis to Navigation2 2D pose estimates
- Validates goal poses before sending to Nav2
- Monitors navigation progress and reports status

## API Specification

### GET /spatial_context
Returns the robot's current spatial understanding in grid and text format.

**Response:**
```json
{
  "position": {
    "x": 1.5,
    "y": 1.5,
    "heading": 1.57
  },
  "grid_view": [
    ["#", "#", "#", "#", "#", "#", "#", "#", "#", "#"],
    ["#", ".", ".", ".", ".", "#", "?", "?", "?", "#"],
    ["#", ".", ".", ".", ".", "#", "?", "?", "?", "#"],
    ["#", ".", ".", "@", ".", ".", ".", ".", "?", "#"],
    ["#", ".", ".", ".", ".", "#", ".", ".", "?", "#"],
    ["#", "#", "#", ".", "#", "#", ".", ".", "?", "#"],
    ["#", ".", ".", ".", ".", ".", ".", ".", "?", "#"],
    ["#", ".", ".", "*", ".", "#", "#", "#", "#", "#"],
    ["#", ".", ".", ".", ".", "#", "?", "?", "?", "#"],
    ["#", "#", "#", "#", "#", "#", "#", "#", "#", "#"]
  ],
  "grid_analysis": {
    "robot_symbol": "@",
    "robot_grid_position": [3, 3],
    "target_symbol": "*",
    "target_grid_position": [3, 7],
    "unexplored_positions": [[6,1], [7,1], [8,1], [6,2], [7,2], [8,2], [8,3], [8,4], [8,5], [8,6], [6,8], [7,8], [8,8]],
    "unexplored_percentage": 30,
    "grid_resolution": 0.5,
    "grid_origin": {"x": 0.0, "y": 0.0}
  },
  "text_description": "You are at grid position (3,3) in a partially mapped room. Target (*) is at (3,7), 2.0m north. Clear path detected through doorway at (3,5). Unexplored areas (30%) to the east.",
  "navigable_goals": [
    {
      "label": "target",
      "grid_position": [3, 7],
      "world_position": {"x": 1.5, "y": 3.5},
      "distance": 2.0
    },
    {
      "label": "nearest_unexplored",
      "grid_position": [6, 1],
      "world_position": {"x": 3.0, "y": 0.5},
      "distance": 3.16
    }
  ]
}
```

### POST /navigate_to
Navigate using relative movement (delta) from current position.

**Request Option 1 - Delta Movement (Recommended):**
```json
{
  "delta": {
    "x": 1.5,             // meters east from current position
    "y": -1.0,            // meters south from current position
    "orientation": 0.785  // Optional: absolute orientation in radians
  },
  "label": "explore_east"  // Optional, for logging
}
```

**Request Option 2 - Absolute Coordinates (Legacy):**
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
  "estimated_time": 15.2,
  "current_position": {
    "x": 3.7,
    "y": 4.8
  },
  "target_position": {
    "x": 5.2,
    "y": 3.8
  }
}
```

### GET /status
Returns current robot and navigation status with detailed Navigation2 feedback.

**Response:**
```json
{
  "robot_state": "navigating",  // Options: "idle", "navigating", "stopped", "error"
  "battery_level": 85,
  "last_command": "delta movement east",
  "position": {
    "x": 3.7,
    "y": 4.8,
    "heading": 1.57
  },
  "navigation_status": {
    "active": true,
    "state": "following_path",  // Options: "planning", "following_path", "replanning", "recovery_active", "path_blocked", "goal_reached", "failed"
    "goal": {
      "x": 5.2,
      "y": 3.8,
      "orientation": 0.785
    },
    "progress": 65,              // Percentage complete (0-100)
    "distance_remaining": 1.2,   // meters
    "estimated_remaining_time": 8.5,  // seconds
    "recovery_attempts": 0,      // Number of recovery behaviors attempted
    "replanning_count": 1        // Number of times path was replanned
  },
  "safety_status": {
    "emergency_stop": false,
    "emergency_stop_triggered": false,  // NEW: Was emergency stop just triggered?
    "obstacles_detected": true,
    "obstacle_clearance": 0.8,   // meters to nearest obstacle
    "path_clear": true
  },
  "nav2_diagnostics": {
    "global_planner_active": true,
    "local_planner_active": true,
    "costmap_updates": true,
    "localization_quality": "good"  // Options: "good", "poor", "lost"
  },
  "simulated_llm_active": false
}
```

### POST /emergency_stop
Immediately stop the robot and cancel all active navigation goals.

**Request:**
```json
{
  "reason": "obstacle_detected"  // Optional: reason for emergency stop
}
```

**Response:**
```json
{
  "status": "stopped",
  "message": "Emergency stop activated - all navigation goals cancelled",
  "goals_cancelled": 1,
  "timestamp": "2024-01-15T10:30:00Z"
}
```

### POST /enable_simulated_llm
Enable or disable the simulated LLM test mode.

**Request:**
```json
{
  "enabled": true,
  "strategy": "nearest_first"  // Optional: override default strategy
}
```

**Response:**
```json
{
  "status": "enabled",
  "message": "Simulated LLM activated with strategy: nearest_first",
  "configuration": {
    "strategy": "nearest_first",
    "decision_delay": 2.0,
    "safety_margin": 1.0
  }
}
```

### GET /simulated_llm_decision
Get the next navigation decision from the simulated LLM.

**Response:**
```json
{
  "decision_available": true,
  "reasoning": "Found unexplored area at grid position [6,2]. Distance: 3.16m. Strategy: nearest_first exploration.",
  "navigation_goal": {
    "target": {
      "x": 3.0,
      "y": 1.0,
      "orientation": 0.523
    },
    "label": "simulated_exploration_6_2",
    "source": "simulated_llm"
  },
  "confidence": 0.85,
  "estimated_completion_time": 12.5
}
```

## Communication Flow

**Normal Operation (Real LLM):**
```
1. LLM requests spatial context
   LLM → GET /spatial_context → Spatial Interpreter → Grid + Text Description

2. LLM analyzes grid and sets navigation goal
   LLM → POST /navigate_to → Nav2 Action Server → Path Planner → Robot Movement

3. LLM monitors progress
   LLM → GET /status → Current State → LLM Feedback Loop
```

**Test Mode (Simulated LLM):**
```
1. Enable simulated LLM
   Test System → POST /enable_simulated_llm → Simulated LLM Activated

2. Robot provides spatial context internally
   Spatial Interpreter → Simulated LLM → Grid Analysis

3. Simulated LLM makes decision
   Simulated LLM → POST /navigate_to → Nav2 Action Server → Robot Movement

4. Monitor simulated decisions
   Test System → GET /simulated_llm_decision → Decision Reasoning
```

## LLM vs Robot-Side Processing

### What the LLM Does:
- **Receives**: JSON data with grid symbols and analysis
- **Analyzes**: Grid patterns using natural language understanding
- **Calculates**: Simple conversions (grid position × resolution = meters)
- **Decides**: Where to navigate based on goals and exploration needs
- **Sends**: JSON requests with target coordinates

### What the Robot API Server Does:
- **Converts**: JSON requests to ROS2 messages
- **Transforms**: Coordinates to Navigation2 format
- **Publishes**: Goals to Nav2 action servers
- **Monitors**: Navigation progress via ROS2 topics
- **Returns**: Status updates as JSON to LLM

### Clear Separation:
- **LLM**: High-level decision making via HTTP/JSON
- **Robot**: Low-level execution via ROS2/Navigation2
- **Interface**: REST API bridges the two systems

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

#### 3. Simplified Grid to Nav2 Coordinate Mapping
```python
def grid_to_world_coordinates(grid_pos, grid_resolution):
    """
    Direct mapping from grid coordinates to world coordinates
    Both use bottom-left origin - no transformation needed
    """
    world_x = grid_pos[0] * grid_resolution
    world_y = grid_pos[1] * grid_resolution
    
    return {"x": world_x, "y": world_y}

def delta_to_nav2_goal(current_position, delta):
    """
    Convert delta movement to absolute Nav2 goal position
    """
    target_x = current_position["x"] + delta["x"]
    target_y = current_position["y"] + delta["y"]
    target_orientation = delta.get("orientation", current_position["heading"])
    
    return {
        "x": target_x,
        "y": target_y, 
        "orientation": target_orientation
    }
```

#### 4. Dynamic Replanning
When LLM sets a Nav2 goal, the system continuously:
- Updates path if new obstacles detected (grid `#` appears)
- Adjusts for moving obstacles
- Triggers recovery behaviors if stuck

### Navigation Goal Selection Strategy

| Scenario | Grid State | Nav2 Goal Target | Configuration Requirement | Reason |
|----------|------------|------------------|---------------------------|---------|
| Exploration | >40% `?` symbols | Nearest `?` position | `allow_unknown: true` | Map unknown areas |
| Target navigation | `*` symbol visible | `*` position | Standard | Reach designated goal |
| Obstacle maze | Many `#` symbols | Best `.` path node | Standard | Nav2 handles path planning |
| Doorway navigation | Single `.` between `#` | Center of doorway `.` | Standard | Navigate through opening |
| Open area | Large `.` region | Center of region | Standard | Maximize visibility |
| Edge exploration | `?` at map boundary | Edge `?` position | `allow_unknown: true` + `track_unknown_space: true` | Expand map coverage |
| Fallback mode | Nav2 unknown disabled | Best `.` position only | `allow_unknown: false` | Safe navigation in known areas |

## Text-Based Spatial Representation

The system converts occupancy grids into natural language descriptions:

### Grid Symbol Mapping
- `.` - Free space (confirmed navigable)
- `#` - Obstacle/wall (confirmed blocked)
- `?` - Unknown/unexplored area (navigable if Navigation2 configured properly)
- `@` - Robot position (current location)
- `*` - Target/goal position (designated objective)

### Grid Coordinate System
The grid uses a **bottom-left origin** coordinate system, matching Navigation2's world coordinates:
- **Origin (0,0)**: Bottom-left corner of the grid
- **+X axis**: Points right (east)
- **+Y axis**: Points up (north)
- **Grid position [x,y]**: Direct correspondence to world coordinates (x*resolution, y*resolution)

This eliminates coordinate transformation complexity and provides direct mapping between grid and Nav2 coordinates.

### Detailed Grid Symbol Usage Example

#### Example Occupancy Grid (5m x 5m area, 0.5m resolution)
```
    0   1   2   3   4   5   6   7   8   9
9   #   #   #   #   #   #   #   #   #   #
8   #   .   .   .   .   #   ?   ?   ?   #
7   #   .   .   *   .   #   #   #   #   #
6   #   .   .   .   .   .   .   .   ?   #
5   #   #   #   .   #   #   .   .   ?   #
4   #   .   .   .   .   #   .   .   ?   #
3   #   .   .   @   .   .   .   .   ?   #
2   #   .   .   .   .   #   ?   ?   ?   #
1   #   .   .   .   .   #   ?   ?   ?   #
0   #   #   #   #   #   #   #   #   #   #
```

**Symbol Locations:**
- `@` at (3,3): Robot's current position
- `*` at (3,7): Target/goal position
- `#`: Walls and obstacles detected by SLAM
- `.`: Free navigable space (confirmed safe)
- `?`: Unexplored areas (navigable depending on Navigation2 configuration)

**Navigation2 Behavior with `?` Symbols:**
- **With `allow_unknown: true`**: Robot can navigate to and through `?` areas
- **With `allow_unknown: false`**: Robot cannot plan paths to `?` positions
- **With `track_unknown_space: false`**: `?` areas treated as obstacles
- **SLAM Integration**: `?` areas convert to `.` or `#` as robot explores

#### Grid-to-Text Conversion
The Spatial Map Interpreter analyzes this grid and generates:

```json
{
  "text_description": "You are located at coordinates (3,3) in a partially mapped room. 
    Forward (heading north): Clear path for 2.0m until a doorway at (3,5). Beyond the 
    doorway is another room extending 1.0m with your target at (3,7).
    Left (heading west): Immediate wall at 0.5m.
    Right (heading east): Clear space for 1.5m, then unexplored area.
    Behind (heading south): Clear path for 1.0m to wall.
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

1. **Grid Analysis**: LLM examines the grid to understand environment
   - Current position: @ at (3,3)
   - Target position: * at (3,7)
   - Path viability: Check for obstacles (#) and unknown areas (?)
   - Distance calculation: Grid cells × resolution (0.5m)

2. **LLM Decision → Delta Movement Request**:
   
   **What the LLM does:**
   - Analyzes grid: "Robot is at [3,3], target is at [3,7]"
   - Calculates delta: "Need to move 0 cells east, 4 cells north = (0m, 2.0m)"
   - Decides: "Send delta movement to target"
   - Sends JSON request:
   ```json
   {
       "delta": {
           "x": 0.0,
           "y": 2.0,
           "orientation": 1.57
       },
       "label": "reach_target"
   }
   ```
   
   **Alternative - Grid-based thinking:**
   ```json
   {
       "delta": {
           "x": 0.0,        // No east/west movement needed
           "y": 2.0,        // Move 2 meters north (4 grid cells × 0.5m)
           "orientation": 1.57  // Face north
       },
       "label": "move_to_target"
   }
   ```

3. **Robot API Server Processing** (runs on robot, not in LLM):
   ```python
   # This code runs on the robot's b4m_llm_nav_api server
   # NOT in the LLM - shown here for implementation reference
   def handle_navigate_request(request_json):
       current_pos = get_current_robot_position()  # Get from /amcl_pose
       
       if "delta" in request_json:
           # Delta-based movement (preferred)
           delta = request_json["delta"]
           target_x = current_pos["x"] + delta["x"]
           target_y = current_pos["y"] + delta["y"]
           target_orientation = delta.get("orientation", current_pos["heading"])
       elif "target" in request_json:
           # Absolute coordinates (legacy support)
           target = request_json["target"]
           target_x = target["x"]
           target_y = target["y"] 
           target_orientation = target.get("orientation", current_pos["heading"])
       else:
           return {"status": "error", "message": "No delta or target specified"}
       
       # Convert to ROS2 Navigation2 goal
       goal = PoseStamped()
       goal.pose.position.x = target_x
       goal.pose.position.y = target_y
       goal.pose.orientation = quaternion_from_euler(0, 0, target_orientation)
       
       # Send to Nav2 - let Nav2 handle path planning and failures
       try:
           nav2_client.send_goal(goal)
           return {
               "status": "navigating", 
               "target_position": {"x": target_x, "y": target_y},
               "estimated_time": calculate_travel_time(current_pos, {"x": target_x, "y": target_y})
           }
       except Exception as e:
           # Let Navigation2 handle the failure and report back to LLM
           return {"status": "nav2_error", "message": str(e)}
   ```

3. **Pattern Recognition for Goal Selection**:
   - **Exploration**: Target nearest '?' symbol for mapping
   - **Navigation**: Target '*' symbol for goal achievement  
   - **Doorway**: Target center of single '.' between walls
   - **Open Area**: Target center of large '.' regions

4. **Navigation2 Handles Complexity**:
   - Path planning around obstacles
   - Dynamic replanning if new obstacles detected
   - Recovery behaviors if stuck
   - Smooth trajectory generation

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
- Grid size automatically matches SLAM map size (no limits)
- Cells >65% occupancy → '#' (obstacle)
- Cells <65% occupancy → '.' (free)
- Bottom-left origin provides direct Nav2 coordinate mapping

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
│   ├── simulated_llm.py        # Simulated LLM for testing
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
│   ├── test_simulated_llm.py   # Simulated LLM tests
│   └── test_navigation.py
├── package.xml
└── setup.py
```

### Simulated LLM Implementation Details

The `simulated_llm.py` module provides automated decision-making for testing:

```python
class SimulatedLLM:
    """
    Simulates LLM behavior for testing robot-to-LLM API
    Accounts for Navigation2 unknown space configuration
    """
    
    def __init__(self, nav2_config):
        self.allow_unknown = nav2_config.get("allow_unknown", False)
        self.track_unknown_space = nav2_config.get("track_unknown_space", False)
        
    def analyze_grid(self, spatial_context):
        """
        Analyzes grid like an LLM would, returns navigation decision
        """
        grid = spatial_context["grid_analysis"]
        
        # Check Navigation2 capability for unknown space
        can_navigate_unknown = self.allow_unknown and self.track_unknown_space
        
        robot_pos = grid["robot_grid_position"]
        
        # Priority 1: Navigate to unexplored areas (if Nav2 supports it)
        if grid["unexplored_positions"] and can_navigate_unknown:
            target = self.find_best_unexplored(grid)
            return self.create_navigation_goal(target, robot_pos, "exploration", 
                                             "Navigating to unexplored area")
        
        # Priority 2: Navigate to target marker if present
        if grid.get("target_grid_position"):
            target = grid["target_grid_position"]
            return self.create_navigation_goal(target, robot_pos, "target",
                                             "Navigating to marked target")
        
        # Priority 3: Navigate to center of largest clear area
        clear_area = self.find_largest_clear_area(grid)
        fallback_reason = "No targets available" if can_navigate_unknown else \
                         "Nav2 unknown space disabled - using known areas only"
        return self.create_navigation_goal(clear_area, robot_pos, "open_area", fallback_reason)
    
    def find_best_unexplored(self, grid):
        """
        Selects best unexplored position based on strategy
        Only called when Navigation2 supports unknown space
        """
        robot_pos = grid["robot_grid_position"]
        unexplored = grid["unexplored_positions"]
        
        # Find nearest unexplored that's safely accessible
        best = None
        min_dist = float('inf')
        
        for pos in unexplored:
            if self.is_safely_accessible(pos, grid):
                dist = self.calculate_distance(robot_pos, pos)
                if dist < min_dist:
                    min_dist = dist
                    best = pos
        
        return best if best else unexplored[0]
    
    def create_navigation_goal(self, target_grid_pos, current_grid_pos, goal_type, reasoning=""):
        """
        Converts target grid position to delta movement JSON with reasoning
        """
        # Calculate delta movement from current to target
        delta_x = (target_grid_pos[0] - current_grid_pos[0]) * 0.5  # Grid cells to meters
        delta_y = (target_grid_pos[1] - current_grid_pos[1]) * 0.5
        
        return {
            "delta": {
                "x": delta_x,
                "y": delta_y,
                "orientation": self.calculate_approach_angle(target_grid_pos, current_grid_pos)
            },
            "label": f"simulated_{goal_type}_{target_grid_pos[0]}_{target_grid_pos[1]}",
            "source": "simulated_llm",
            "reasoning": reasoning,
            "nav2_config_check": {
                "allow_unknown": self.allow_unknown,
                "track_unknown_space": self.track_unknown_space
            }
        }
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

simulated_llm:
  enabled: false             # Set to true to use simulated LLM
  strategy: "nearest_first"  # Options: "nearest_first", "systematic", "random"
  decision_delay: 2.0        # Seconds to wait before making decision
  exploration_radius: 5.0    # Maximum distance to explore from current position
  safety_margin: 1.0         # Minimum distance from obstacles (meters)
  completion_threshold: 95   # Percentage of map explored before stopping
  verbose_logging: true      # Log simulated LLM decisions
```

## Navigation2 Configuration for Unknown Space

**Critical:** Navigation2 must be properly configured to allow navigation to unexplored areas (marked with `?` in the grid). The following parameters are required:

```yaml
# Navigation2 planner configuration
planner_server:
  ros__parameters:
    planner_plugins: ['GridBased']
    GridBased:
      plugin: 'nav2_navfn_planner::NavfnPlanner'
      tolerance: 0.5
      use_astar: true
      allow_unknown: true    # REQUIRED: Allows planning through unexplored areas

# Navigation2 costmap configuration  
global_costmap:
  global_costmap:
    ros__parameters:
      track_unknown_space: true  # REQUIRED: Differentiates unknown from free space
      unknown_cost_value: 255    # Cost value for unknown cells
      
local_costmap:
  local_costmap:
    ros__parameters:
      track_unknown_space: true  # REQUIRED: Local costmap must also track unknown space
```

### Key Parameters Explained:

- **`allow_unknown: true`**: Enables path planning through areas marked as `?` (unexplored)
- **`track_unknown_space: true`**: Required for costmaps to properly handle unknown areas
- **`unknown_cost_value`**: Determines how unknown space is treated (255 = unknown, 0 = free)

### Without Proper Configuration:
- Navigation2 will fail to plan paths to `?` positions
- Robot will be limited to only known areas (`.` symbols)
- LLM exploration strategies will be severely limited

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

### Simulated LLM Test Mode

The system includes a built-in simulated LLM mode for testing the robot-to-LLM API without requiring a real LLM service. This mode mimics LLM behavior by automatically analyzing the grid and selecting appropriate navigation goals.

#### Enabling Simulated LLM Mode

**Launch with test flag:**
```bash
ros2 launch b4m_llm_nav_api llm_nav_api_launch.py use_simulated_llm:=true
```

**Or set in configuration:**
```yaml
# config/llm_nav_api.yaml
test_mode:
  use_simulated_llm: true
  exploration_strategy: "nearest_first"  # Options: "nearest_first", "systematic", "random"
  target_selection_delay: 2.0  # Seconds before selecting new goal
  prefer_unexplored: true  # Prioritize unexplored areas
  safety_margin: 1.0  # Meters from obstacles
```

#### Simulated LLM Behavior

The simulated LLM follows this decision logic, accounting for Navigation2's capabilities:

1. **Receives spatial context** (same as real LLM would)
2. **Analyzes grid** for clear areas and obstacles
3. **Validates Navigation2 configuration** for unknown space support
4. **Selects navigation goal** based on strategy:
   - **Priority 1**: If unexplored areas exist (`?`) AND `allow_unknown: true` → Navigate to nearest `?`
   - **Priority 2**: If target marker exists (`*`) → Navigate to `*`
   - **Priority 3**: Navigate to center of largest clear area (`.`)
   - **Fallback**: If Nav2 doesn't support unknown space, avoid `?` positions
5. **Validates goal safety** against obstacles and unknown space policy
6. **Sends navigation command** with calculated 2D pose

#### Example Simulated LLM Response

**When robot provides spatial context:**
```json
GET /spatial_context returns:
{
  "grid_analysis": {
    "robot_grid_position": [3, 3],
    "unexplored_positions": [[6, 2], [7, 3]],
    "clear_areas": [
      {"center": [5, 5], "radius": 2.0}
    ]
  }
}
```

**Simulated LLM automatically responds:**
```json
POST /navigate_to with:
{
  "target": {
    "x": 3.0,
    "y": 1.0,
    "orientation": 0.523
  },
  "label": "simulated_exploration_6_2",
  "source": "simulated_llm"
}
```

#### Test Scenarios

The simulated LLM can run various test scenarios:

**1. Exploration Test:**
```python
# Simulated LLM systematically explores all unknown areas
def exploration_test():
    # Continuously navigates to unexplored positions
    # Until map is >95% complete
```

**2. Obstacle Avoidance Test:**
```python
# Simulated LLM navigates around obstacles
def obstacle_test():
    # Finds clear paths around detected obstacles
    # Maintains safety_margin from walls
```

**3. Target Seeking Test:**
```python
# Simulated LLM navigates to marked targets
def target_test():
    # If '*' symbol present, navigate to it
    # Tests direct goal navigation
```

### Test Commands
```bash
# Run unit tests
colcon test --packages-select b4m_llm_nav_api

# Run integration tests with simulation
./b4m_launch.sh --simulation
ros2 run b4m_llm_nav_api test_integration

# Run with simulated LLM
ros2 launch b4m_llm_nav_api llm_nav_api_launch.py use_simulated_llm:=true

# Run full system test with simulated LLM
./tests/test_llm_navigation_system.sh --simulated-llm
```

## Example LLM Interactions

**Note:** The examples below show the JSON communication between the LLM and robot API. The LLM sends/receives JSON via HTTP - it does NOT execute Python code directly.

### Example 1: Basic Exploration

**LLM receives from GET /spatial_context:**
```json
{
  "position": {"x": 1.5, "y": 1.5, "heading": 0.0},
  "grid_analysis": {
    "robot_grid_position": [3, 3],
    "unexplored_positions": [[6,1], [7,1], [8,1]],
    "grid_resolution": 0.5
  }
}
```

**LLM reasoning:** "I'm at grid [3,3]. Unexplored area at [6,1] means I need to move +3 cells east, -2 cells south. That's +1.5m east, -1.0m south from my current position."

**LLM sends to POST /navigate_to:**
```json
{
  "delta": {
    "x": 1.5,     // Move 1.5m east from current position
    "y": -1.0,    // Move 1.0m south from current position  
    "orientation": 0.0
  },
  "label": "explore_unknown"
}
```

**Robot API server calculates absolute goal (1.5+1.5, 1.5-1.0) = (3.0, 0.5) and sends to Nav2**

### Example 2: Target Navigation

**LLM receives from GET /spatial_context:**
```json
{
  "position": {"x": 1.5, "y": 1.5, "heading": 0.0},
  "grid_analysis": {
    "robot_grid_position": [3, 3],
    "target_grid_position": [3, 7],
    "grid_resolution": 0.5
  },
  "text_description": "Target (*) is at (3,7), 2.0m north of your position"
}
```

**LLM reasoning:** "Target is at grid [3,7]. I'm at [3,3]. Need to move 0 cells east, +4 cells north. That's 0m east, +2.0m north from current position."

**LLM sends to POST /navigate_to:**
```json
{
  "delta": {
    "x": 0.0,     // No east/west movement
    "y": 2.0,     // Move 2.0m north
    "orientation": 1.57  // Face north
  },
  "label": "reach_target"
}
```

### Example 3: Doorway Navigation

**LLM receives grid showing doorway pattern:**
```json
{
  "grid_view": [
    ["#", "#", ".", "#", "#"]
  ],
  "text_description": "Doorway detected at position (2,0)"
}
```

**LLM reasoning:** "Single '.' at [2,0] indicates doorway. Navigate through it."

**LLM sends:**
```json
{
  "target": {
    "x": 1.0,
    "y": 0.0,
    "orientation": 1.57
  },
  "label": "navigate_through_doorway"
}
```

### Example 4: Exploration Conversation Flow

**This shows the conversation pattern between LLM and robot API:**

**Round 1 - LLM checks exploration status:**
```json
GET /spatial_context response:
{
  "position": {"x": 2.0, "y": 1.5, "heading": 0.0},
  "grid_analysis": {
    "robot_grid_position": [4, 3],
    "unexplored_percentage": 45,
    "unexplored_positions": [[6,1], [7,2], [8,3]],
    "grid_resolution": 0.5
  }
}
```

**LLM decides:** "45% unexplored, navigate to nearest unknown area [6,1]. That's +2 cells east, -2 cells south from my position."

```json
POST /navigate_to request:
{
  "delta": {"x": 1.0, "y": -1.0, "orientation": 0.0},
  "label": "explore_6_1"
}
```

**Round 2 - After reaching first point:**
```json
GET /spatial_context response:
{
  "position": {"x": 3.0, "y": 0.5, "heading": 0.0},
  "grid_analysis": {
    "robot_grid_position": [6, 1],
    "unexplored_percentage": 35,
    "unexplored_positions": [[7,2], [8,3]]
  }
}
```

**LLM continues exploration using delta movements until unexplored_percentage < 5%**

### Example 5: Semantic Goal Navigation

**LLM receives context with semantic hints:**
```json
GET /spatial_context response:
{
  "grid_analysis": {
    "robot_grid_position": [3, 3],
    "large_open_areas": [
      {"center": [8, 8], "size": 25, "label": "possible_room"}
    ],
    "narrow_passages": [
      {"center": [5, 2], "length": 8, "label": "possible_corridor"}
    ]
  },
  "text_description": "Large open area detected at east (possibly a room). Narrow passage to the north (possibly a corridor)."
}
```

**LLM reasoning:** "User asked to go to 'kitchen'. Large open areas often indicate rooms. Navigate to [8,8]."

**LLM sends:**
```json
{
  "target": {"x": 4.0, "y": 4.0, "orientation": 0.785},
  "label": "navigate_to_kitchen"
}
```

### Implementation Notes

**For Robot API Developers:**
The robot-side API server (b4m_llm_nav_api) handles:
- Converting JSON coordinates to ROS2 PoseStamped messages
- Publishing goals to Navigation2 action servers
- Monitoring navigation feedback from Nav2
- Returning status updates to the LLM

**For LLM Integration:**
The LLM only needs to:
1. Parse JSON from GET /spatial_context
2. Analyze grid patterns and positions
3. Calculate target coordinates (grid_pos × 0.5m)
4. Send JSON to POST /navigate_to
5. Poll GET /status for completion

## Safety Considerations

1. **Delta Movement Validation**: All delta movements are validated before converting to absolute goals
2. **Navigation2 Error Handling**: Invalid goals are handled by Nav2 and reported back to LLM
3. **Emergency Stop Integration**: `POST /emergency_stop` immediately cancels all goals and notifies LLM
4. **Continuous Monitoring**: Real-time obstacle detection and path clearance monitoring
5. **Timeout Notifications**: Navigation timeouts are immediately reported to LLM via `/status`
6. **Automatic Recovery**: Nav2 recovery behaviors are exposed to LLM through detailed status updates

### Navigation in Unexplored Areas (`?` symbols)

**Additional safety measures when navigating to unknown space:**

6. **Unknown Space Validation**: 
   - Verify Navigation2 `allow_unknown: true` before sending goals to `?` positions
   - Check `track_unknown_space: true` is configured in costmaps
   - Fallback to known areas if configuration doesn't support unknown navigation

7. **Exploration Safety Margins**:
   - Increase safety distance when approaching unexplored boundaries
   - Use conservative speeds when entering unknown areas
   - Monitor sensor data for unexpected obstacles

8. **Dynamic Replanning**:
   - Navigation2 will replan if new obstacles discovered in `?` areas
   - Robot automatically stops if unknown area becomes blocked
   - Recovery behaviors activated if exploration goal becomes unreachable

9. **Configuration Validation**:
   ```python
   # Safety check before sending goal to unexplored area
   def validate_unknown_navigation(nav2_config, target_position):
       if target_position in unexplored_areas:
           if not nav2_config.get("allow_unknown", False):
               raise NavigationError("Nav2 not configured for unknown space")
           if not nav2_config.get("track_unknown_space", False):
               raise NavigationError("Costmap not tracking unknown space")
       return True
   ```

10. **Sensor Requirements**:
    - LiDAR must be active and publishing valid data
    - SLAM system must be running and updating map
    - Localization (AMCL) must have good position estimate
    - All transforms (map→odom→base_link) must be valid

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