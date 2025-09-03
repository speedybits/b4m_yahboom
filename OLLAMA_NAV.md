# Ollama Navigation Explore Mode - Specification

## Overview

The `--ollama-nav-explore` mode builds upon the proven `--nav` infrastructure to add fully autonomous robot exploration using a Large Language Model (LLM). This mode leverages the existing Nav2 navigation stack that already handles 2D goal pose setting successfully, and adds intelligent goal selection through Ollama LLM queries.

**Foundation:** This mode extends the working `--nav` functionality (Navigation 2 with Cartographer SLAM) by automating goal selection rather than relying on manual RViz goal setting.

## Core Purpose

Create an autonomous exploration system where:
- The robot builds a map from scratch using SLAM
- At startup and after each goal completion, the system analyzes the environment
- Ollama LLM selects the next navigation goal based on current spatial context
- The robot navigates to that goal using Nav2
- Upon reaching or aborting the goal, immediately analyze and select the next goal
- The process repeats continuously for full area exploration

## System Requirements

### Prerequisites
- **Working `--nav` Mode**: The `--nav` flag must function correctly (prerequisite)
- **Ollama Service**: Must be running on `localhost:11434`
- **LLM Model**: Compatible model installed (e.g., `llama3.2:latest`)
- **Existing Infrastructure**: Uses same Nav2 + Cartographer SLAM as `--nav` mode
- **360° LIDAR**: Already working in `--nav` mode for environmental sensing
- **No Pre-built Maps**: System always starts with fresh SLAM (same as `--nav`)

### Launch Commands
```bash
# Real robot exploration
./b4m_launch.sh --ollama-nav-explore

# Simulation mode
./b4m_launch.sh --ollama-nav-explore --simulation

# Debug mode with verbose output
./b4m_launch.sh --ollama-nav-explore --simulation --debug
```

## System Architecture

### Data Flow

```
┌─────────────────┐     ┌──────────────────┐     ┌─────────────────┐
│   360° LIDAR    │────▶│  Spatial Context │────▶│  Ollama LLM     │
│   + Map Data    │     │     Builder      │     │ Goal Selection  │
└─────────────────┘     └──────────────────┘     └─────────────────┘
                                                          │
                                                          ▼
┌─────────────────┐     ┌──────────────────┐     ┌─────────────────┐
│   Robot Motion  │◀────│   Navigation 2   │◀────│  Navigation     │
│                 │     │   (Nav2 Stack)   │     │     Goal        │
└─────────────────┘     └──────────────────┘     └─────────────────┘
         │                                                │
         └────────────────────────────────────────────────┘
                    Goal Reached/Aborted
                    (Triggers new analysis)
```

## Functional Requirements

### 1. Environmental Analysis (Triggered Events)

**When to Analyze:**
- At system startup
- Immediately after goal completion (success)
- Immediately after goal abortion (failure)
- NO periodic timer - analysis is event-driven

**Analysis Requirements:**
- Capture current robot position and orientation
- Analyze 360° LIDAR data in 8 sectors (45° each)
- Identify obstacles, open spaces, and navigable areas
- Detect unexplored map boundaries (frontiers)
- Generate a comprehensive spatial description

### 2. LLM Integration

**Ollama Query Requirements:**
- Send structured prompts with environmental context
- Request navigation decisions in JSON format
- Handle 2-minute timeout gracefully
- Retry on connection failures

**Expected LLM Response Format:**
```json
{
  "relative_distance": 2.5,      // meters from current position
  "relative_bearing": 45,        // degrees relative to current heading
  "final_orientation": 90,       // absolute orientation at goal
  "reasoning": "Moving to explore unmapped corridor",
  "goal_type": "MOVEMENT"        // or "ROTATION"
}
```

### 3. Goal Validation

**Safety Constraints:**
- Movement goals must be ≥1.0m from current position
- Goals must be within explored/safe areas
- Invalid goals automatically convert to rotation-in-place
- 50/50 split between movement and rotation goals

**Multi-waypoint Navigation:**
- Distances <2m: Single goal point
- Distances ≥2m: Generate waypoints at 1.5m intervals
- Maximum 5 waypoints per navigation path

### 4. Navigation Execution

- Send validated goals to Nav2 navigation stack
- Monitor navigation progress
- Handle goal completion/abortion
- Immediately trigger next Ollama query (no delay)

## Logging Requirements

### Terminal Output (Main Launch Terminal)

The main terminal where `b4m_launch.sh` is executed must display:

**Startup:**
```
🧭 OLLAMA NAVIGATION EXPLORE MODE
======================================
Launching Navigation 2 with Cartographer SLAM for LLM-guided exploration

Step 1: Starting Micro-ROS Agent...
✅ Micro-ROS Agent started (PID: 12345)

Step 7: Starting Ollama Exploration Spatial Analysis
✅ Ollama spatial analysis started (PID: 23456)

🤖 AUTONOMOUS NAVIGATION ACTIVE
```

**During Operation:**
The main terminal must display detailed LLM interaction:

```
🔍 ENVIRONMENTAL ANALYSIS
Current Position: (2.35, 1.82) facing 15° northeast
Map Coverage: 67% explored

📤 OLLAMA PROMPT:
========================================
You are a robot explorer. Analyze the environment and select the next navigation goal.

SPATIAL CONTEXT:
• FRONT (0°): CLEAR - Open space extending 3.2m
• FRONT-RIGHT (45°): BLOCKED - Wall at 1.1m  
• RIGHT (90°): CLEAR - Corridor continues 2.8m
• BACK-RIGHT (135°): CLEAR - Open area beyond 4.2m
• BEHIND (180°): BLOCKED - Wall at 0.8m
• BACK-LEFT (225°): CLEAR - Space extends 2.1m
• LEFT (270°): CLEAR - Open area 3.5m
• FRONT-LEFT (315°): PASSABLE - Narrow passage 1.8m

EXPLORATION STATUS:
• Unexplored frontiers detected at bearings: 45°, 135°, 270°
• Most promising frontier: Southeast at 135° (4.2m clear)
• Current room appears 85% mapped

Select navigation goal (1-5m distance, relative bearing ±180°):
========================================

🧠 Waiting for Ollama response... (timeout: 120s)

📥 OLLAMA RESPONSE (1.8s):
{
  "relative_distance": 3.5,
  "relative_bearing": 135,
  "final_orientation": 180,
  "reasoning": "Moving southeast toward unexplored frontier to discover new area",
  "goal_type": "MOVEMENT"
}

✅ Goal validated: Moving 3.5m at 135° (southeast)
🎯 Target coordinates: (4.83, -0.65) facing 180°
🚀 Sending navigation goal to Nav2...

🛤️ NAVIGATION IN PROGRESS
- Multi-waypoint path (3 waypoints)
- Current progress: 45% complete
- ETA: 12 seconds

✅ NAVIGATION COMPLETED
Final position: (4.81, -0.68) facing 182°
Distance traveled: 3.4m in 15.2s
```

**Error States:**
```
❌ OLLAMA SERVICE UNAVAILABLE
==================================================
Ollama LLM service is not responding
Navigation system stopping - no fallback movement
==================================================

❌ INVALID OLLAMA RESPONSE
📥 OLLAMA RESPONSE (2.3s):
{
  "relative_distance": 0.3,
  "relative_bearing": 45,
  "final_orientation": 90,
  "reasoning": "Short movement to nearby area"
}

⚠️ Goal rejected: Movement distance 0.3m < 1.0m minimum
🔄 Generating rotation goal as fallback
🎯 Fallback goal: Rotate to 90° at current position
🚀 Sending rotation goal to Nav2...
```

### Detailed Log Files

**Location:** `logs/ollama_spatial_YYYYMMDD_HHMMSS.log`

**Must Include:**
- Full Ollama prompts sent (same as terminal display)
- Complete LLM responses received (same as terminal display)
- Goal validation decisions with reasoning
- Navigation state transitions
- Detailed error messages and stack traces
- Performance metrics (response times, distances, success rates)

**Log Entry Format:**
```
[TIMESTAMP] [LEVEL] [MODULE]: Message
[2025-09-03 14:23:45.123] [INFO] [ollama_explore]: 🔍 ENVIRONMENTAL ANALYSIS
[2025-09-03 14:23:45.124] [INFO] [ollama_explore]: Current Position: (2.35, 1.82) facing 15° northeast
[2025-09-03 14:23:45.234] [INFO] [ollama_explore]: 📤 OLLAMA PROMPT:
[2025-09-03 14:23:45.235] [DEBUG] [ollama_explore]: You are a robot explorer. Analyze the environment...
[2025-09-03 14:23:46.567] [INFO] [ollama_explore]: 📥 OLLAMA RESPONSE (1.3s):
[2025-09-03 14:23:46.568] [DEBUG] [ollama_explore]: {"relative_distance": 3.5, "relative_bearing": 135...}
[2025-09-03 14:23:46.570] [INFO] [ollama_explore]: ✅ Goal validated: Moving 3.5m at 135° (southeast)
```

**Important:** The main terminal output and log file should contain identical information for transparency and debugging purposes.

### ROS2 Console Output

The ROS2 node must output to console with clear emoji indicators:
- 🔍 Analysis in progress
- 🧠 Querying Ollama
- ✅ Success states
- ⚠️ Warnings
- ❌ Errors
- 🎯 Goal navigation
- 🔄 Rotation goals
- 📍 Position updates

## State Management

### System States

1. **INITIALIZING** - System startup, waiting for sensors
2. **ANALYZING** - Performing spatial analysis
3. **QUERYING_LLM** - Waiting for Ollama response
4. **NAVIGATING** - Executing navigation goal
5. **ERROR** - Handling failures
6. **SHUTDOWN** - Clean termination

### State Transitions

Each state change must be:
- Logged to file with timestamp
- Displayed in terminal (major states only)
- Include relevant context (position, goal, etc.)

## Error Handling

### Ollama Failures
- **Connection Error**: Log, display error, stop navigation (no autonomous retry)
- **Timeout (2 min)**: Log, display error, stop navigation
- **Invalid Response**: Log full response, generate rotation goal as fallback

### Navigation Failures
- **Goal Rejected**: Log reason, immediately analyze and request new goal
- **Path Planning Failed**: Log, abort goal, trigger immediate re-analysis
- **Obstacle Detected**: Abort current goal, immediate environmental re-analysis

### Recovery Behavior
- No autonomous fallback movements when Ollama is unavailable
- System stops and waits for manual intervention or Ollama restoration
- On navigation failures, immediate re-analysis and new goal selection
- Clear error messages in main terminal
- Detailed diagnostics in log file

## Configuration

### File: `config/ollama_nav_config.yaml`

**Key Parameters:**
```yaml
navigation:
  rotation_probability: 0.5      # Movement vs rotation ratio
  min_movement_distance: 1.0     # Minimum goal distance (meters)
  waypoint_spacing: 1.5          # Distance between waypoints (meters)
  max_waypoints: 5               # Maximum waypoints per path
  
ollama:
  host: localhost
  port: 11434
  model: llama3.2:latest
  timeout: 120.0                 # 2-minute timeout
  
safety:
  max_consecutive_failures: 3    # Stop after N failures
  nav2_timeout: 120.0            # Goal execution timeout
  obstacle_clearance: 0.5        # Minimum clearance from obstacles (meters)
```

## Performance Metrics

### Session Statistics (On Shutdown)
```
🧭🦙 OLLAMA NAVIGATION SESSION COMPLETE
===============================================================
Session Statistics:
• Total Goals Selected: 12
• Total Distance Traveled: 24.3m
• Navigation Success Rate: 91.7% (11/12 goals)
• Area Explored: 35% → 78% (+43%)
• Average LLM Response Time: 1.8s
• Session Duration: 12m 45s
===============================================================
```

### Real-time Metrics (In Logs)
- LLM query response time
- Goal validation time
- Navigation execution time
- Distance to goal
- Exploration coverage percentage


## Development Guidelines

### Leveraging Existing Infrastructure

**Reuse `--nav` Components:**
- Launch sequence: Copy from existing `--nav` mode in `b4m_launch.sh`
- Navigation stack: Use identical Nav2 + Cartographer configuration
- Goal sending: Reuse Nav2 action client patterns from existing code
- RViz integration: Same visualization setup as `--nav` mode

**Reference `--ollama` for LLM Integration:**
- HTTP client: Reuse Ollama connection patterns from `--ollama` mode
- JSON handling: Copy request/response parsing from existing `--ollama` code
- Error handling: Leverage timeout and connection error patterns
- Configuration: Extend existing `ollama_config.yaml` structure

### Code Organization
- **Main node**: `scripts/ollama_explore_spatial.py`
- **Configuration**: `config/ollama_nav_config.yaml` (extend existing)
- **Launch integration**: Add to `b4m_launch.sh` (copy `--nav` section)

### Key Interfaces
- **Input**: `/scan` (LIDAR), `/map` (occupancy grid), `/odom` (position)
- **Output**: Navigation goals to Nav2 action server (same as `--nav`)
- **Logging**: Both ROS2 logging and Python logging to file

### Implementation Strategy
1. **Start with `--nav` working mode** - Copy the launch sequence and navigation setup
2. **Add Ollama client** - Copy HTTP client code from `--ollama` mode  
3. **Replace manual goals** - Instead of RViz goal setting, use LLM goal selection
4. **Test incrementally** - Verify each component works with existing infrastructure

### Testing in Simulation
1. Start with `--simulation` flag
2. Verify Ollama connectivity
3. Monitor exploration coverage in RViz
4. Check log files for proper formatting
5. Test error scenarios (disconnect Ollama)

## Future Enhancements (Not Required for Initial Implementation)

- Frontier detection for smarter exploration
- Multi-robot coordination
- Learned exploration strategies
- Dynamic obstacle handling
- Exploration completion detection
- Map quality assessment

---

This specification defines the requirements for the `--ollama-nav-explore` mode. Focus on creating a robust, well-logged system that provides clear feedback to users and maintains safe autonomous operation.
