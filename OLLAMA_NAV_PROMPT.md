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
    Error handling when no safe destinations found (rare case)
    """
    # Log error and stop exploration
    logger.error("No safe navigation destinations found - stopping exploration")
    logger.error(f"Robot stuck at position: {robot_pos} heading: {robot_heading}")
    
    # Enter ERROR state - requires manual intervention
    raise NoSafeDestinationsError(
        "Cannot find any safe navigation destinations. "
        "Robot may be trapped or map may be fully explored. "
        "Manual intervention required."
    )
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

## Example Prompts

### Example 1: Open Space with Multiple Options

```
You are a robot explorer. Choose your next navigation destination from the PRE-VALIDATED safe options below.

CURRENT SITUATION:
• Position: (2.35, 1.82) facing 45°
• Map Coverage: 67% explored  
• Surroundings: Robot has multiple options: Open space ahead, Open space to the right, Unexplored area ahead-left

AVAILABLE SAFE DESTINATIONS (choose one):
1. Move 2.0m forward (bearing +0°) → EXPLORES NEW AREA
2. Move 2.5m forward-right (bearing +45°) → EXPLORES NEW AREA
3. Move 1.8m right (bearing +90°)
4. Move 2.2m back-right (bearing +135°)
5. Move 1.5m left (bearing -90°) → EXPLORES NEW AREA
6. Move 2.8m forward-left (bearing -45°) → EXPLORES NEW AREA

STRATEGIC CONTEXT:
• Unexplored frontiers detected near: ahead (0°), ahead-left (-45°), left (-90°)
• Most promising exploration direction: ahead-left (-45°) - multiple safe paths available
• Current area appears 67% mapped - good exploration opportunities

Select destination by number (1-6) in JSON format:
{
  "selected_destination": 3,
  "reasoning": "Brief explanation of choice"
}
```

### Example 2: Corridor Navigation

```
You are a robot explorer. Choose your next navigation destination from the PRE-VALIDATED safe options below.

CURRENT SITUATION:
• Position: (5.12, -0.88) facing 90°
• Map Coverage: 43% explored  
• Surroundings: Robot is in corridor with Open space ahead, Wall/obstacle close to the left, Wall/obstacle close behind

AVAILABLE SAFE DESTINATIONS (choose one):
1. Move 3.0m forward (bearing +0°) → EXPLORES NEW AREA
2. Move 2.5m forward (bearing +0°) → EXPLORES NEW AREA
3. Move 1.8m forward (bearing +0°)
4. Move 1.2m back-right (bearing +135°) → RETURNS TO KNOWN AREA

STRATEGIC CONTEXT:
• Unexplored frontiers detected near: ahead (0°)
• Most promising exploration direction: forward (0°) - corridor continues into unexplored territory
• Current area appears to be a corridor - continue forward for exploration

Select destination by number (1-4) in JSON format:
{
  "selected_destination": 3,
  "reasoning": "Brief explanation of choice"
}
```

### Example 3: Limited Options (Tight Space)

```
You are a robot explorer. Choose your next navigation destination from the PRE-VALIDATED safe options below.

CURRENT SITUATION:
• Position: (1.07, 3.24) facing 180°
• Map Coverage: 78% explored  
• Surroundings: Robot is in confined space with Wall/obstacle close ahead, Wall/obstacle close to the right

AVAILABLE SAFE DESTINATIONS (choose one):
1. Move 1.5m left (bearing -90°) → RETURNS TO KNOWN AREA
2. Move 1.2m back-left (bearing -135°) → RETURNS TO KNOWN AREA
3. Move 2.1m backward (bearing +180°) → RETURNS TO KNOWN AREA

STRATEGIC CONTEXT:
• Few unexplored frontiers remaining - most areas mapped
• Current position appears to be near dead end or room corner
• Consider returning to main exploration area to find remaining unmapped regions

Select destination by number (1-3) in JSON format:
{
  "selected_destination": 3,
  "reasoning": "Brief explanation of choice"
}
```

### Example 4: Many Options (Large Open Area)

```
You are a robot explorer. Choose your next navigation destination from the PRE-VALIDATED safe options below.

CURRENT SITUATION:
• Position: (0.23, 0.15) facing 315°
• Map Coverage: 34% explored  
• Surroundings: Robot has multiple options: Open space ahead, Open space to the right, Open space behind

AVAILABLE SAFE DESTINATIONS (choose one):
1. Move 2.8m forward (bearing +0°) → EXPLORES NEW AREA
2. Move 2.3m forward-right (bearing +45°) → EXPLORES NEW AREA
3. Move 3.0m right (bearing +90°) → EXPLORES NEW AREA
4. Move 2.6m back-right (bearing +135°) → EXPLORES NEW AREA
5. Move 2.0m backward (bearing +180°)
6. Move 2.4m back-left (bearing -135°) → EXPLORES NEW AREA
7. Move 2.9m left (bearing -90°) → EXPLORES NEW AREA
8. Move 2.1m forward-left (bearing -45°) → EXPLORES NEW AREA
9. Move 1.8m forward-right (bearing +22°) → EXPLORES NEW AREA
10. Move 2.7m right (bearing +67°) → EXPLORES NEW AREA

STRATEGIC CONTEXT:
• Unexplored frontiers detected near: all directions - robot in center of large unmapped area
• Most promising exploration directions: multiple options available
• Early exploration phase - 34% mapped, many opportunities for discovery

Select destination by number (1-10) in JSON format:
{
  "selected_destination": 3,
  "reasoning": "Brief explanation of choice"
}
```

### Example 5: Retry After Previous Goal Rejection (Rare)

```
You are a robot explorer. Choose your next navigation destination from the PRE-VALIDATED safe options below.

The previous goal was rejected, please try something else.
Use different strategy or destination selection approach.

CURRENT SITUATION:
• Position: (3.67, 2.11) facing 135°
• Map Coverage: 55% explored  
• Surroundings: Robot has multiple options: Open space ahead, Unexplored area to the right, Open space behind

AVAILABLE SAFE DESTINATIONS (choose one):
1. Move 1.8m forward (bearing +0°) → EXPLORES NEW AREA
2. Move 2.2m forward-right (bearing +45°) → EXPLORES NEW AREA
3. Move 1.5m right (bearing +90°) → EXPLORES NEW AREA
4. Move 2.0m back-left (bearing -135°) → RETURNS TO KNOWN AREA
5. Move 2.3m left (bearing -90°) → EXPLORES NEW AREA

STRATEGIC CONTEXT:
• Unexplored frontiers detected near: ahead (0°), right (90°), left (-90°)
• Previous goal encountered unexpected issue - try different approach
• Consider shorter distances or different strategic direction

Select destination by number (1-5) in JSON format:
{
  "selected_destination": 3,
  "reasoning": "Brief explanation of choice"
}
```

## Expected LLM Responses

### Response to Example 1 (Open Space):
```json
{
  "selected_destination": 6,
  "reasoning": "Choosing forward-left option to explore new area in the most promising frontier direction while maintaining safe distance from obstacles"
}
```

### Response to Example 2 (Corridor):
```json
{
  "selected_destination": 1,
  "reasoning": "Moving forward through corridor to maximize exploration of unmapped territory ahead, using full safe distance available"
}
```

### Response to Example 3 (Limited Options):
```json
{
  "selected_destination": 3,
  "reasoning": "Backing out of confined area to return to main exploration region where more mapping opportunities exist"
}
```

### Response to Example 4 (Many Options):
```json
{
  "selected_destination": 7,
  "reasoning": "Selecting left direction for systematic exploration of large open area, good distance for efficient coverage"
}
```

### Response to Example 5 (Retry):
```json
{
  "selected_destination": 3,
  "reasoning": "Trying shorter distance and different direction from previous attempt, focusing on safer right-side exploration"
}
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