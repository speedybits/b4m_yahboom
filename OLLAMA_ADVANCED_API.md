# OLLAMA_ADVANCED_API.md - Advanced 360° Spatial Context for Ollama LLM Navigation

## Overview

This specification describes an advanced integration of Ollama Large Language Models with the B4M Yahboom robot navigation system. Unlike the basic mode that focuses on cardinal directions, the `--ollama-advanced` mode provides comprehensive 360-degree spatial awareness, enabling more sophisticated navigation decisions through detailed environmental descriptions.

### Key Differences from Basic Mode
- **Full 360° Context**: Complete radial distance measurements every 15-30 degrees
- **Periodic Decision Points**: Robot stops at regular intervals for environmental assessment
- **Open-ended Prompting**: Natural language understanding without predefined action constraints
- **Spatial Richness**: Detailed distance measurements in all directions for complex navigation

### Prerequisites
- **Ollama installed and running**: Ollama service must be running on localhost:11434
- **Model downloaded**: At least one compatible model (e.g., `ollama pull llama3.2`)
- **Network access**: Local network connectivity between robot system and Ollama service
- **LIDAR sensor**: 360-degree laser scanner providing continuous distance measurements

## Architecture

### System Components

```
┌─────────────────┐     ┌──────────────────┐     ┌─────────────────┐
│   360° LIDAR    │────▶│  360° Spatial    │────▶│  Ollama API     │
│   Laser Scan    │     │  Context Builder │     │   Client        │
└─────────────────┘     └──────────────────┘     └─────────────────┘
                                                          │
                                                          ▼
┌─────────────────┐     ┌──────────────────┐     ┌─────────────────┐
│  Robot Motion   │◀────│ Command Executor │◀────│ Response Parser │
│   (/cmd_vel)    │     │                  │     │   (NLP → Cmd)   │
└─────────────────┘     └──────────────────┘     └─────────────────┘
```

### Data Flow Pipeline

1. **Continuous Monitoring**: 360° laser scan data from LIDAR (`/scan` topic)
2. **Periodic Stopping**: Robot halts at configurable intervals (time or distance based)
3. **360° Analysis**: Convert full radial scan to comprehensive spatial description
4. **Prompt Generation**: Create natural language description of surroundings
5. **LLM Processing**: Send to Ollama with open-ended question
6. **NLP Parsing**: Extract navigation intent from natural language response
7. **Command Execution**: Convert parsed intent to robot motion commands
8. **Resume Movement**: Continue until next decision point

## 360-Degree Spatial Context Generation

### Radial Sector Analysis

The system divides the 360° LIDAR scan into 24 sectors (15° each) for comprehensive spatial analysis:

- **Sector Division**: 360° divided into 24 sectors, each covering 15 degrees
- **Distance Metrics**: Minimum, average, and maximum distances per sector
- **Obstacle Detection**: Sectors with objects closer than 0.5m are marked as "blocked"
- **Direction Naming**: Human-readable names (front, front-right, right, etc.) for each sector

### Spatial Context Summary

For each 360° scan, the system generates:
- **Closest obstacle**: Distance and direction to nearest object
- **Clearest path**: Direction with maximum average clearance
- **Blocked sectors**: Count of sectors with obstacles < 0.5m
- **Average clearance**: Overall spatial openness metric

### Direction Naming Convention

Angles are converted to intuitive directions:
- 0° = "front"
- 45° = "right-front" 
- 90° = "right-side"
- 135° = "back-right"
- 180° = "behind"
- 225° = "left-back"
- 270° = "left-side"
- 315° = "front-left"

## Natural Language Description Generation

### Spatial Description Format

The system converts 360° scan data into a structured natural language description with four main sections:

1. **Immediate Warnings**: Obstacles closer than 0.3m are highlighted first
2. **Environment Summary**: Grouped sectors described as BLOCKED (<0.5m), PASSABLE (0.5-3m), or CLEAR (>3m)
3. **Navigation Analysis**: Statistical summary of the environment
4. **Feature Detection**: Special patterns like corridors, corners, or dead ends

### Feature Detection Patterns

The system recognizes common spatial configurations:

- **Corridor**: Clear path ahead with blocked sides
- **Tight Corner**: 18+ blocked sectors out of 24
- **Open Area**: Average clearance > 5 meters
- **Dead End**: 20+ blocked sectors out of 24

### Example Spatial Description

```
You are a mobile robot with 360-degree vision. Here's what surrounds you:

⚠️ IMMEDIATE OBSTACLE: 0.25m to your front

SURROUNDING ENVIRONMENT:
• front to front-right: BLOCKED - obstacle at 0.25m
• right-front to right-side: PASSABLE - nearest object at 1.45m
• right-back to back: CLEAR - open space for at least 4.2m
• back-left to left-back: BLOCKED - obstacle at 0.32m
• left-side to left-front: PASSABLE - nearest object at 2.1m

NAVIGATION ANALYSIS:
• Clearest path: right-back
• Blocked directions: 8 of 24 sectors
• Average clearance: 1.87m

DETECTED FEATURES:
• You appear to be in a CORRIDOR extending forward
• This appears to be a DEAD END
```

## Ollama Prompt Engineering

### Structured Response Format

The prompt instructs Ollama to respond with a specific format for reliable parsing:

**Required Format:**
```
"Turn [N] degrees, and then move ahead [M] meters or until we reach an obstacle."
```

**Parameters:**
- **[N]**: Rotation angle in degrees
  - Positive values = right/clockwise rotation
  - Negative values = left/counter-clockwise rotation
  - 0 = no rotation
- **[M]**: Forward movement distance in meters
  - Any positive value = move forward
  - 0 = stay in place

**Valid Response Examples:**
- `"Turn 90 degrees, and then move ahead 3 meters or until we reach an obstacle."`
- `"Turn -45 degrees, and then move ahead 1.5 meters or until we reach an obstacle."`
- `"Turn 0 degrees, and then move ahead 2 meters or until we reach an obstacle."`
- `"Turn 180 degrees, and then move ahead 5 meters or until we reach an obstacle."`
- `"Turn 0 degrees, and then move ahead 0 meters or until we reach an obstacle."` (stop)

### Example Ollama Interaction

**Prompt sent to Ollama:**
```
You are a mobile robot with 360-degree vision. Here's what surrounds you:

⚠️ IMMEDIATE OBSTACLE: 0.25m to your front

SURROUNDING ENVIRONMENT:
• front to front-right: BLOCKED - obstacle at 0.25m
• right-front to right-side: PASSABLE - nearest object at 1.45m
• right-back to back: CLEAR - open space for at least 4.2m
• back-left to left-back: BLOCKED - obstacle at 0.32m
• left-side to left-front: PASSABLE - nearest object at 2.1m

NAVIGATION ANALYSIS:
• Clearest path: right-back
• Blocked directions: 8 of 24 sectors
• Average clearance: 1.87m

DETECTED FEATURES:
• You appear to be in a CORRIDOR extending forward
• This appears to be a DEAD END

Based on this 360-degree view of your surroundings, what is your navigation decision?

IMPORTANT: You must respond in EXACTLY this format:
"Turn [N] degrees, and then move ahead [M] meters or until we reach an obstacle."
```

**Expected Ollama Response:**
```
Turn 135 degrees, and then move ahead 4 meters or until we reach an obstacle.
```

## Response Parsing

### Parser Strategy

The response parser uses a multi-tier approach for reliability:

1. **Primary Pattern**: Exact match for the required format
2. **Fallback Patterns**: Handle slight variations in Ollama's response
3. **Safety Validation**: Enforce maximum turn angles and distances

### Parsing Logic

**Primary Pattern Match:**
- Regex: `Turn ([-]?\d+) degrees, and then move ahead (\d+) meters`
- Extracts turn angle and move distance directly

**Fallback Patterns:**
- Turn and move: `turn X degrees.*move Y meters`
- Just turn: `turn X degrees` (assumes 0 meter movement)
- Just move: `move ahead Y meters` (assumes 0 degree turn)

**Parsed Output Structure:**
```json
{
  "success": true,
  "turn_angle": -135,      // negative = left, positive = right
  "move_distance": 4.0,    // meters
  "parse_method": "structured"
}
```

### Safety Validation

All parsed commands are validated against safety limits:
- **Maximum turn angle**: 360 degrees (configurable)
- **Maximum move distance**: 10 meters (configurable)
- Values exceeding limits are capped with warnings

## Configuration

### Advanced Mode Configuration (ollama_advanced_config.yaml)

```yaml
ollama:
  host: localhost
  port: 11434
  model: llama3.2:latest
  timeout: 20.0  # Longer timeout for processing detailed descriptions
  
generation:
  temperature: 0.7  # Higher temperature for more creative navigation
  top_p: 0.95
  max_tokens: 200  # More tokens for detailed responses
  format: text  # Natural language responses, not JSON
  stream: false
  
navigation:
  decision_interval_distance: 5.0  # Stop every 5 meters for decisions
  decision_interval_time: 30.0  # Or every 30 seconds, whichever comes first
  continuous_movement: true  # Move continuously between decision points
  default_forward_speed: 0.2  # m/s
  default_turn_speed: 0.5  # rad/s
  
spatial_context:
  sector_size_degrees: 15  # Divide 360° into 24 sectors
  obstacle_threshold: 0.5  # Consider <0.5m as obstacle
  clear_threshold: 3.0  # Consider >3.0m as clear
  group_similar_sectors: true  # Group adjacent similar sectors in description
  
safety:
  emergency_stop_distance: 0.15
  max_forward_distance: 10.0  # Maximum distance for single forward command
  max_turn_angle: 360  # Maximum turn angle in degrees
  enable_manual_override: true
  collision_prediction: true  # Predict and prevent collisions
  
response_parsing:
  use_nlp: true  # Use natural language parsing
  fallback_to_keywords: true  # Fallback to keyword matching if NLP fails
  confidence_threshold: 0.5  # Minimum confidence for parsed commands
  max_retries: 3  # Number of retries on parse failure with same prompt
```

## Implementation Integration

### Modified b4m_launch.sh for Advanced Mode

```bash
# Add new option for advanced Ollama mode
elif [ "$1" == "--ollama-advanced" ]; then
    OLLAMA_ADVANCED_MODE=true
    MODE="ollama_advanced"
    shift
    
    # Check if Ollama is running
    if ! curl -s http://localhost:11434/api/tags > /dev/null 2>&1; then
        echo "❌ ERROR: Ollama is not running on localhost:11434"
        echo "   Please start Ollama first: ollama serve"
        exit 1
    fi
```

### Implementation Notes

Key aspects of the advanced mode implementation:

1. **Command-line Activation**: Use `--ollama-advanced` flag to enable this mode
2. **Distance Tracking**: Uses odometry from `/odom` topic (filtered EKF output) to track distance traveled
3. **Decision Points**: Robot stops completely at decision intervals for stable 360° scanning
4. **In-place Rotations**: Turns are executed without forward movement, no obstacle checking needed
5. **Continuous Movement**: Between decision points, robot moves forward continuously at default speed
6. **Safety First**: Any parsing failure or timeout results in immediate stop

### Behavioral Specifications

**Decision Point Triggers:**
- Robot immediately stops when ANY trigger condition is met:
  - Distance traveled ≥ configured interval (e.g., 5m)
  - Time elapsed ≥ configured interval (e.g., 30s)  
  - Obstacle detected within emergency stop distance
- No command completion - immediate stop and new decision

**Obstacle During Movement:**
- If obstacle detected while executing "move 5 meters" command:
  - Robot stops immediately (e.g., after 2 meters)
  - Performs new 360° scan
  - Sends updated spatial context to Ollama
  - Requests new navigation decision
  - Does NOT wait for next scheduled decision point

**Parse Failure Recovery:**
- On parse failure, retry with the SAME prompt up to 3 times
- If all retries fail, stop and require manual intervention
- No fallback to simplified prompts

**Coordinate Reference:**
- All turn angles are relative to robot's CURRENT heading
- "Turn 90 degrees" = 90° clockwise from where robot is facing now
- Each decision starts from current orientation, not from previous decision point

**Movement Between Decisions:**
- Robot continues straight at configured default forward speed (e.g., 0.2 m/s)
- Maintains constant velocity until next decision trigger
- No path correction or obstacle avoidance between decision points

## Console Output Examples

### Example 1: Normal Navigation Decision

```
🦙 ADVANCED OLLAMA MODE ACTIVATED
===============================================================
Robot will use 360° spatial context for navigation
Decision interval: 5.0m
Model: llama3.2:latest
===============================================================

[14:32:15.123] 🤖 Advanced Spatial Interpreter started
[14:32:15.456] ➡️ Moving forward...
[14:32:25.789] 📏 Progress: 2.5m traveled
[14:32:35.012] 📏 Progress: 5.0m traveled - Time for decision

===============================================================
🌐 360° SPATIAL ANALYSIS
===============================================================

You are a mobile robot with 360-degree vision. Here's what surrounds you:

SURROUNDING ENVIRONMENT:
• front to front-right: CLEAR - open space for at least 3.5m
• right-front to right-side: PASSABLE - nearest object at 1.45m
• right-back to back: BLOCKED - obstacle at 0.45m
• back-left to left: CLEAR - open space for at least 5.2m
• left-front: PASSABLE - nearest object at 2.1m

NAVIGATION ANALYSIS:
• Clearest path: left
• Blocked directions: 4 of 24 sectors
• Average clearance: 2.34m

DETECTED FEATURES:
• You appear to be in a CORRIDOR extending forward

🦙 Consulting Ollama for navigation decision...

💭 OLLAMA RESPONSE: Turn 0 degrees, and then move ahead 3 meters or until we reach an obstacle.

🎯 PARSED COMMAND:
   • Turn: 0° (no rotation)
   • Move: 3.0 meters forward
   • Parse method: structured

[14:32:37.234] ➡️ Moving forward 3.0m or until obstacle...
[14:32:42.567] ✅ Movement complete (traveled 3.0m)
```

### Example 2: Obstacle Interrupts Movement

```
[14:32:37.234] ➡️ Moving forward 5.0m or until obstacle...
[14:32:39.567] ⚠️ Obstacle detected after 2.3m - stopping immediately

===============================================================
🌐 360° SPATIAL ANALYSIS (UNSCHEDULED)
===============================================================

You are a mobile robot with 360-degree vision. Here's what surrounds you:

⚠️ IMMEDIATE OBSTACLE: 0.28m to your front

SURROUNDING ENVIRONMENT:
• front: BLOCKED - obstacle at 0.28m
• front-right to right: CLEAR - open space for at least 3.2m
• right-back to back: PASSABLE - nearest object at 1.8m
• left-side to front-left: BLOCKED - obstacle at 0.45m

NAVIGATION ANALYSIS:
• Clearest path: right
• Blocked directions: 10 of 24 sectors
• Average clearance: 1.65m

🦙 Consulting Ollama for navigation decision...

💭 OLLAMA RESPONSE: Turn 90 degrees, and then move ahead 3 meters or until we reach an obstacle.

🎯 PARSED COMMAND:
   • Turn: 90° (right rotation)
   • Move: 3.0 meters forward
   • Parse method: structured

[14:32:41.234] 🔄 Turning right 90°...
[14:32:43.567] ✅ Turn complete
[14:32:43.789] ➡️ Moving forward 3.0m or until obstacle...
```

### Example 3: Scheduled Decision Point

```
===============================================================
🌐 360° SPATIAL ANALYSIS
===============================================================

You are a mobile robot with 360-degree vision. Here's what surrounds you:

⚠️ IMMEDIATE OBSTACLE: 0.22m to your front

SURROUNDING ENVIRONMENT:
• front to right-front: BLOCKED - obstacle at 0.22m
• right-side: PASSABLE - nearest object at 0.85m
• right-back to behind: BLOCKED - obstacle at 0.31m
• back-left to left-back: CLEAR - open space for at least 4.5m
• left-side to left-front: BLOCKED - obstacle at 0.42m

NAVIGATION ANALYSIS:
• Clearest path: back-left
• Blocked directions: 16 of 24 sectors
• Average clearance: 1.12m

DETECTED FEATURES:
• You are in a TIGHT CORNER with limited maneuvering space

🦙 Consulting Ollama for navigation decision...

💭 OLLAMA RESPONSE: Turn -135 degrees, and then move ahead 4 meters or until we reach an obstacle.

🎯 PARSED COMMAND:
   • Turn: -135° (left rotation)
   • Move: 4.0 meters forward
   • Parse method: structured

[14:33:45.123] 🔄 Turning left 135°...
[14:33:48.456] ✅ Turn complete
[14:33:48.789] ➡️ Moving forward 4.0m or until obstacle...
[14:33:53.234] ✅ Movement complete (traveled 4.0m)
```

### Example 4: Parse Failure with Retry

```
🦙 Consulting Ollama for navigation decision...

💭 OLLAMA RESPONSE: I should turn to the right and then proceed forward for about 3 meters.

❌ Failed to parse response: Could not parse response
   Original: I should turn to the right and then proceed forward for about 3 meters.
   
🔄 Retrying (attempt 2/3)...

💭 OLLAMA RESPONSE: Turn 90 degrees, and then move ahead 3 meters or until we reach an obstacle.

🎯 PARSED COMMAND:
   • Turn: 90° (right rotation)
   • Move: 3.0 meters forward
   • Parse method: structured

[14:34:15.123] 🔄 Turning right 90°...
```

## Testing Strategy

### Unit Tests
- Test 360° context generation with various scan patterns
- Test natural language response parsing
- Test command extraction from complex sentences
- Test safety mechanisms and limits

### Integration Tests
- Test with actual Ollama API using various models
- Test decision-making in different spatial scenarios
- Test multi-command execution sequences
- Test error recovery and fallback mechanisms

### Simulation Tests
```bash
# Test in Gazebo with advanced mode
./b4m_launch.sh --ollama-advanced --simulation

# Test with specific decision intervals
./b4m_launch.sh --ollama-advanced --simulation --decision-interval 3.0

# Test with debug output
./b4m_launch.sh --ollama-advanced --simulation --debug-verbose
```

## Performance Considerations

### Optimization Strategies

1. **Context Caching**: Cache similar spatial contexts to avoid redundant Ollama calls
2. **Parallel Processing**: Process sectors in parallel for faster context generation
3. **Response Caching**: Cache common navigation patterns
4. **Adaptive Intervals**: Adjust decision intervals based on environment complexity

### Expected Performance Metrics

- **Context Generation**: < 100ms for 360° analysis
- **Ollama Response Time**: 1-5 seconds depending on model and prompt complexity
- **Command Parsing**: < 50ms
- **Total Decision Time**: 2-6 seconds from stop to resume

## Safety Mechanisms

### Multi-layer Safety System

1. **Continuous Obstacle Monitoring**: Even during movement between decisions
2. **Command Validation**: Verify all commands are safe before execution
3. **Distance Limiting**: Cap maximum movement distances
4. **Collision Prediction**: Anticipate collisions based on current trajectory
5. **Emergency Override**: Manual intervention always available
6. **Timeout Protection**: Stop if Ollama doesn't respond within timeout
7. **Sanity Checks**: Verify parsed commands make spatial sense

## Future Enhancements

1. **Context Memory**: Remember previous positions and decisions
2. **Path Planning**: Build mental map of explored areas
3. **Learning**: Improve decisions based on outcomes
4. **Multi-modal Input**: Incorporate camera and other sensors
5. **Goal-directed Navigation**: Navigate to specific coordinates or landmarks
6. **Semantic Understanding**: Recognize and name areas (kitchen, hallway, etc.)
7. **Collaborative Navigation**: Coordinate with other robots or humans

---

This advanced specification enables sophisticated autonomous navigation using comprehensive 360-degree spatial awareness and natural language processing, allowing the robot to make intelligent navigation decisions in complex environments.