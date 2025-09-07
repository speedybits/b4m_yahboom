# Ollama Navigation Prompt Specification

## Overview

This document specifies the prompt generation system for the `--ollama-nav-explore` mode. The Spatial Context Builder provides pre-validated safe navigation destinations to the LLM, eliminating goal rejection loops and ensuring all LLM responses are inherently valid.

## Architecture

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│  Spatial Context│───▶│    Prompt        │───▶│   Ollama LLM    │
│     Builder     │    │   Generator      │    │   Selection     │
│                 │    │                  │    │                 │
│ • Map Analysis  │    │ • Up to 10 safe │    │ • Choose 1 of N │
│ • Safety Check  │    │   destinations   │    │   destinations  │
│ • Frontier Info │    │ • Clear options  │    │ • JSON response │
└─────────────────┘    └──────────────────┘    └─────────────────┘
```

## Spatial Context Builder Requirements

### Input Data Sources
- **Map Data**: Current SLAM occupancy grid from `/map` topic
- **Robot Position**: Real-time position from TF transforms or `/odom`  
- **Robot Heading**: Current orientation in radians
- **LIDAR Data**: 360° laser scan from `/scan` topic (optional for additional safety)

### Safe Destination Generation Algorithm

```python
def generate_safe_destinations(robot_pos, robot_heading, map_data, max_destinations=10):
    """
    Generate up to 10 pre-validated safe navigation destinations
    Returns list of SafeDestination objects
    """
    safe_destinations = []
    
    # Search in expanding patterns around robot
    search_patterns = [
        # Pattern 1: 8 cardinal/ordinal directions
        [(distance, angle) for angle in [0, 45, 90, 135, 180, 225, 270, 315] 
         for distance in [1.0, 1.5, 2.0, 2.5, 3.0]],
        
        # Pattern 2: Additional angles for more options  
        [(distance, angle) for angle in [22, 67, 112, 157, 202, 247, 292, 337]
         for distance in [1.5, 2.0, 2.5]]
    ]
    
    for distance, relative_bearing in search_patterns:
        if len(safe_destinations) >= max_destinations:
            break
            
        # Calculate absolute world coordinates
        absolute_bearing = robot_heading + math.radians(relative_bearing)
        target_x = robot_pos[0] + distance * math.cos(absolute_bearing)
        target_y = robot_pos[1] + distance * math.sin(absolute_bearing)
        
        # Validate safety: occupancy <= 24 (even stricter than Nav2 default)
        if is_destination_safe(target_x, target_y, map_data):
            destination = SafeDestination(
                relative_bearing=relative_bearing,
                distance=distance,
                world_coords=(target_x, target_y),
                description=generate_destination_description(relative_bearing, distance, target_x, target_y, map_data)
            )
            safe_destinations.append(destination)
    
    return safe_destinations

def is_destination_safe(x, y, map_data):
    """
    Check if destination coordinates are in confirmed safe space
    Uses occupancy <= 24 (stricter than Nav2's 25 threshold)
    """
    occupancy_value = get_occupancy_at_point(x, y, map_data)
    return occupancy_value <= 24 and occupancy_value >= 0
```

### SafeDestination Data Structure

```python
@dataclass
class SafeDestination:
    relative_bearing: float      # Degrees relative to robot heading (-180 to 180)
    distance: float             # Meters from current position (1.0 to 3.0)
    world_coords: Tuple[float, float]  # (x, y) in map frame
    description: str            # Human-readable description for LLM
    leads_to_frontier: bool     # Whether this direction approaches unexplored areas
    strategic_value: str        # "exploration", "return_to_base", "corridor_following", etc.
```

## Prompt Generation Specification

### Template Structure

```
You are a robot explorer. Choose your next navigation destination from the PRE-VALIDATED safe options below.

CURRENT SITUATION:
• Position: ({robot_x:.2f}, {robot_y:.2f}) facing {robot_heading_deg:.0f}°
• Map Coverage: {exploration_percentage:.0f}% explored  
• Surroundings: {environmental_description}

AVAILABLE SAFE DESTINATIONS (choose one):
{destination_list}

STRATEGIC CONTEXT:
{frontier_information}
{exploration_guidance}

Select destination by number (1-{num_destinations}) in JSON format:
{
  "selected_destination": 3,
  "reasoning": "Brief explanation of choice"
}
```

### Environmental Description Generation

```python
def generate_environmental_description(robot_pos, map_data, lidar_data):
    """
    Create natural language description of robot's surroundings
    Based on LIDAR sectors and map analysis
    """
    descriptions = []
    
    # Analyze 8 main sectors around robot
    sector_descriptions = {
        0: "ahead", 45: "ahead-right", 90: "to the right", 135: "behind-right",
        180: "behind", 225: "behind-left", 270: "to the left", 315: "ahead-left"
    }
    
    for angle, direction in sector_descriptions.items():
        distance = get_lidar_distance_at_angle(angle, lidar_data)
        map_status = analyze_map_in_direction(robot_pos, angle, 2.0, map_data)
        
        if distance < 0.8:
            descriptions.append(f"Wall/obstacle close {direction}")
        elif distance > 3.0 and map_status == "free":
            descriptions.append(f"Open space {direction}")
        elif map_status == "frontier_nearby":
            descriptions.append(f"Unexplored area {direction}")
    
    # Combine into natural description
    if len(descriptions) <= 2:
        return f"Robot is in confined space with {', '.join(descriptions)}"
    else:
        return f"Robot has multiple options: {', '.join(descriptions[:3])}"
```

### Destination List Formatting

```python
def format_destination_list(safe_destinations):
    """
    Format destinations as numbered list for LLM selection
    """
    destination_text = []
    
    for i, dest in enumerate(safe_destinations, 1):
        # Convert bearing to compass direction
        compass_dir = bearing_to_compass(dest.relative_bearing)
        
        # Add strategic indicators
        strategy_indicator = ""
        if dest.leads_to_frontier:
            strategy_indicator = " → EXPLORES NEW AREA"
        elif dest.strategic_value == "return_to_base":
            strategy_indicator = " → RETURNS TO KNOWN AREA"
        
        destination_text.append(
            f"{i}. Move {dest.distance:.1f}m {compass_dir} "
            f"(bearing {dest.relative_bearing:+.0f}°){strategy_indicator}"
        )
    
    return "\n".join(destination_text)

def bearing_to_compass(bearing):
    """Convert relative bearing to compass direction"""
    compass_map = {
        0: "forward", 45: "forward-right", 90: "right", 135: "back-right",
        180: "backward", 225: "back-left", 270: "left", 315: "forward-left"
    }
    
    # Find closest compass direction
    closest_bearing = min(compass_map.keys(), key=lambda x: abs(x - bearing))
    return compass_map[closest_bearing]
```

## LLM Response Processing

### Expected Response Format

```json
{
  "selected_destination": 3,
  "reasoning": "Choosing forward-right option to explore new area while staying in confirmed safe space"
}
```

### Response Validation

```python
def validate_llm_response(response_json, num_available_destinations):
    """
    Validate LLM response format and selection
    Since destinations are pre-validated, only format checking needed
    """
    required_fields = ["selected_destination", "reasoning"]
    
    # Check required fields
    for field in required_fields:
        if field not in response_json:
            return False, f"Missing required field: {field}"
    
    # Validate destination selection
    selection = response_json["selected_destination"]
    if not isinstance(selection, int):
        return False, "selected_destination must be integer"
        
    if not (1 <= selection <= num_available_destinations):
        return False, f"selected_destination must be 1-{num_available_destinations}"
    
    return True, "Valid response"
```

### Goal Conversion

```python
def convert_selection_to_nav_goal(selected_destination, robot_heading):
    """
    Convert LLM selection to Nav2 navigation goal
    No safety validation needed - destinations are pre-validated
    """
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = "map"
    goal_pose.header.stamp = get_current_time()
    
    # Use pre-calculated world coordinates
    goal_pose.pose.position.x = selected_destination.world_coords[0]
    goal_pose.pose.position.y = selected_destination.world_coords[1] 
    goal_pose.pose.position.z = 0.0
    
    # Set orientation (can face goal direction or maintain current heading)
    target_heading = robot_heading + math.radians(selected_destination.relative_bearing)
    quaternion = quaternion_from_euler(0, 0, target_heading)
    
    goal_pose.pose.orientation.x = quaternion[0]
    goal_pose.pose.orientation.y = quaternion[1]
    goal_pose.pose.orientation.z = quaternion[2]
    goal_pose.pose.orientation.w = quaternion[3]
    
    return goal_pose
```

## Error Handling

### No Safe Destinations Available

```python
def handle_no_safe_destinations(robot_pos, robot_heading):
    """
    Fallback when no safe destinations found (rare case)
    """
    # Option 1: Generate rotation-in-place goal
    rotation_angles = [45, 90, 135, 180, -45, -90, -135]
    
    for angle in rotation_angles:
        # Check if rotating would reveal new safe areas
        # Return rotation goal if beneficial
        pass
    
    # Option 2: Request user intervention
    logger.warning("No safe navigation destinations found - manual intervention may be required")
    
    # Option 3: Small backward movement if safe
    return generate_retreat_goal(robot_pos, robot_heading)
```

### LLM Response Errors

Since destinations are pre-validated, LLM response errors should be rare and limited to:
- **Format errors**: Invalid JSON or missing fields
- **Selection errors**: Number out of range (1-N)
- **Connection errors**: Ollama service unavailable

All these trigger the existing retry mechanism with the "try something else" message.

## Integration with Navigation Controller

```python
class PromptGenerator:
    def __init__(self, map_data, robot_pos, robot_heading, lidar_data):
        self.map_data = map_data
        self.robot_pos = robot_pos  
        self.robot_heading = robot_heading
        self.lidar_data = lidar_data
    
    def generate_navigation_prompt(self):
        # 1. Generate safe destinations
        safe_destinations = generate_safe_destinations(
            self.robot_pos, self.robot_heading, self.map_data
        )
        
        if not safe_destinations:
            return self.handle_no_safe_destinations()
        
        # 2. Create environmental description
        env_description = generate_environmental_description(
            self.robot_pos, self.map_data, self.lidar_data
        )
        
        # 3. Format destination list
        destination_list = format_destination_list(safe_destinations)
        
        # 4. Generate complete prompt
        prompt = self.build_complete_prompt(
            env_description, destination_list, safe_destinations
        )
        
        return prompt, safe_destinations
```

## Testing and Validation

### Unit Tests Required
- `test_safe_destination_generation()` - Verify only safe coords generated
- `test_occupancy_validation()` - Confirm occupancy <= 24 threshold
- `test_prompt_formatting()` - Validate prompt structure
- `test_llm_response_parsing()` - Check JSON validation
- `test_goal_conversion()` - Verify Nav2 goal creation

### Integration Tests
- Test with various map scenarios (open space, corridors, dead ends)
- Verify destinations remain valid as robot moves
- Confirm exploration progress with pre-validated goals

---

This specification ensures the LLM only sees safe, actionable options while maintaining clear separation between spatial analysis and prompt generation responsibilities.