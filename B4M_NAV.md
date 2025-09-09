# B4M Navigation Explore Mode - Specification

## Overview

The `--b4m-nav-explore` mode builds upon the proven `--nav` infrastructure to add fully autonomous robot exploration using the B4M API. This mode leverages the existing Nav2 navigation stack that already handles 2D goal pose setting successfully, and adds intelligent goal selection through B4M API queries.

**Foundation:** This mode extends the working `--nav` functionality (Navigation 2 with Cartographer SLAM) by automating goal selection rather than relying on manual RViz goal setting.

## Core Purpose

Create an autonomous exploration system that operates in a **closed-loop cycle**:
- The robot builds a map from scratch using SLAM
- At startup and after **every goal completion/abortion**, the system performs fresh environmental analysis
- B4M API selects the next navigation goal using **updated map data** and current spatial context
- The robot navigates to that goal using Nav2
- Upon reaching or aborting the goal, **immediately trigger new analysis** using the expanded/updated map
- This closed-loop process repeats **indefinitely** - the robot continues exploring even when no new frontiers are apparent

**Critical Closed-Loop Behavior:** Each navigation completion immediately triggers a new analysis cycle that incorporates all the latest map data and spatial information gathered during the previous navigation, ensuring continuous improvement in exploration decisions.

## System Requirements

### Prerequisites
- **Working `--nav` Mode**: The `--nav` flag must function correctly (prerequisite)
- **B4M API Service**: Must be accessible (same as `--b4m-ping` mode)
- **Existing Infrastructure**: Uses same Nav2 + Cartographer SLAM as `--nav` mode
- **360° LIDAR**: Already working in `--nav` mode for environmental sensing
- **No Pre-built Maps**: System always starts with fresh SLAM (same as `--nav`)

### Launch Commands
```bash
# Real robot exploration
./b4m_launch.sh --b4m-nav-explore

# Simulation mode
./b4m_launch.sh --b4m-nav-explore --simulation

# Debug mode with verbose output
./b4m_launch.sh --b4m-nav-explore --simulation --debug
```

## System Architecture

### Closed-Loop Architecture

```
    ┌─────────────────────────────────────────────────────────────────┐
    │                        CONTINUOUS CYCLE                         │
    │                                                                 │
    ▼                                                                 │
┌─────────────────┐     ┌──────────────────┐     ┌─────────────────┐ │
│   360° LIDAR    │────▶│  Spatial Context │────▶│    B4M API      │ │
│ + UPDATED Map   │     │ Builder (FRESH)  │     │ Goal Selection  │ │
│   Data (SLAM)   │     │    Analysis      │     │  (NEW Context)  │ │
└─────────────────┘     └──────────────────┘     └─────────────────┘ │
                                                          │           │
                                                          ▼           │
┌─────────────────┐     ┌──────────────────┐     ┌─────────────────┐ │
│   Robot Motion  │◀────│   Navigation 2   │◀────│  Navigation     │ │
│ (Map Updates)   │     │   (Nav2 Stack)   │     │     Goal        │ │
│   + Position    │     │                  │     │                 │ │
└─────────────────┘     └──────────────────┘     └─────────────────┘ │
                                 │                                   │
                                 │                                   │
                    GOAL COMPLETION/ABORTION                        │
                                 ↓                                   │
                 ┌─────────────────────────────┐                   │
                 │   IMMEDIATE RE-ANALYSIS     │                   │
                 │                             │                   │
                 │ • Updated SLAM map data     │                   │
                 │ • New robot position       │                   │
                 │ • Fresh sensor readings    │                   │
                 │ • Updated spatial context  │                   │
                 └─────────────────────────────┘                   │
                                 │                                   │
                                 └───────────────────────────────────┘
                            CLOSES THE LOOP
```

**Key Closed-Loop Elements:**
- **Fresh Analysis**: Every cycle uses the most current map and sensor data
- **Safe-Only Context**: B4M API only sees pre-validated safe navigation options, eliminating rejection loops
- **No Delays**: Goal completion immediately triggers new environmental analysis  
- **Cumulative Learning**: Each navigation adds to map knowledge for better future decisions
- **Continuous Improvement**: Later goals benefit from expanded map coverage and spatial understanding
- **Guaranteed Success**: Pre-filtering ensures B4M selections are always valid, preventing navigation failures

## Functional Requirements

### 1. Environmental Analysis (Closed-Loop Trigger Events)

**Initial Mapping Sequence (Startup Bootstrap):**
- **0.5m Square Mapping**: At startup, robot performs a 0.5-meter square movement pattern to establish initial free space
- **4-Step Sequence**: Forward → Right → Back → Left (0.5m each direction, relative to robot's initial heading) using Nav2 navigation
- **Very Slow Movement**: Initial mapping movements must be executed at very slow speed to ensure laserscan data maintains proper alignment with the SLAM map being built
- **Foundation Building**: Creates confirmed navigable area around starting position before normal exploration
- **Seamless Transition**: After completing initial mapping, system transitions to normal closed-loop exploration
- **Note**: Movements are relative to the robot's starting orientation, not absolute compass directions

**When to Analyze (Event-Driven Closed Loop):**
- After initial mapping sequence completion (using established free space foundation)
- **Immediately after goal completion** (success) - using updated map data from navigation
- **Immediately after goal abortion** (failure) - incorporating any map updates gathered during failed attempt
- NO periodic timer - analysis is purely event-driven to ensure fresh data

**Closed-Loop Analysis Requirements:**
- **Always use latest map state**: Capture the most current SLAM map data (not cached/old data)
- **Fresh position data**: Get real-time robot position and orientation post-navigation
- **Current sensor readings**: Analyze live 360° LIDAR data in 8 sectors (45° each)
- **Safe area pre-filtering**: ONLY include sectors that lead to confirmed safe territory (occupancy ≤ 25) in spatial context
- **Navigation option validation**: For each sector, verify both path AND destination are in free space before including in prompt
- **Frontier detection**: Find unexplored boundaries based on expanded map coverage  
- **Simplified spatial context**: Only describe verified safe navigation options to prevent B4M API from selecting invalid goals

**Critical:** Each analysis cycle MUST use the freshest available data - never reuse previous analysis results or cached spatial descriptions.

### 2. B4M API Integration

**B4M API Query Requirements:**
- Send structured prompts with environmental context (similar to existing `--b4m-ping` implementation)
- Request navigation decisions in JSON format
- Handle timeout gracefully (same timeout as `--b4m-ping`)
- Retry on connection failures
- **Goal Rejection Recovery** (Rare with Safe-Only Approach): If goal is rejected despite safe sector pre-filtering, modify prompt to include "The previous goal was rejected, please try something else" before sending retry request

**B4M API Integration Architecture:**
- **Numbered Selection Approach**: Present up to 10 pre-validated safe destinations to B4M API
- **No Coordinate Calculations**: B4M API selects by number (1-10), eliminating coordinate system confusion
- **Guaranteed Valid Goals**: All destinations are pre-validated for safety and reachability
- **SafeDestination Data Structure**: Each option includes relative_bearing, distance, world_coords, and strategic context
- **API Connection**: Uses same connection patterns as existing `--b4m-ping` mode

**Expected B4M API Response Format:**
```json
{
  "selected_destination": 3,      // Number (1-N) from provided options
  "reasoning": "Brief explanation of choice"
}
```

**SafeDestination Data Structure:**
```python
@dataclass
class SafeDestination:
    relative_bearing: float      # Degrees relative to robot heading (-180 to 180)
    distance: float             # Meters from current position (1.0 to 3.0)
    world_coords: Tuple[float, float]  # (x, y) in map frame
    description: str            # Human-readable description for B4M API
    leads_to_frontier: bool     # Whether unexplored cells exist within 1m of destination
    strategic_value: str        # Optional hint (not prioritized in ordering)
```

**Note**: Destinations are presented in the order they are discovered during the search pattern, not prioritized by strategic value or frontier proximity.

### 3. Goal Validation

**Safety Constraints:**
- Movement goals must be ≥1.0m from current position  
- Goals must be in **free space only** (occupancy value ≤ 25, light gray in RViz per Nav2 standard)
- **NO goals in unexplored territory** - only confirmed safe areas
- Navigation goals should move toward frontiers but **always land in free space**
- Invalid goals automatically convert to rotation-in-place
- 50/50 split between movement and rotation goals

**Goal Validation Rules (CRITICAL - Only Free Space Goals Accepted):**

**Single Rule - Free Space Only:**
- Goals are **ONLY accepted in free space** (occupancy value ≤ 25, light gray in RViz per Nav2 standard)
- This is the **ONLY acceptable case** - all goals must be in confirmed safe areas
- **NO goals in unknown territory** - regardless of nearby free space
- **NO goals in any other occupancy values** - even low probability obstacles

**Rejection Criteria (All Non-Free Space):**
- Goals in **unknown areas** (occupancy value = -1, dark gray in RViz) are **always rejected**
- Goals in **obstacles** (occupancy value ≥ 50, black in RViz) are **always rejected**  
- Goals with **any probability obstacles** (values 1-49) are **always rejected**
- Goals in **any non-zero occupancy value** are **always rejected**

**Validation Implementation:**
```python
# Nav2-aligned validation logic - Use Nav2 free_thresh standard
if occupancy_value <= 25 and occupancy_value >= 0:  # Free space per Nav2 standard
    return True  # Light gray areas in RViz (Nav2 free_thresh: 0.25)
else:  # Everything else (unknown, obstacles, uncertain areas)
    return False  # Reject goals outside Nav2 free space definition
```

**Exploration Strategy:**
- Goals must **always be placed in light gray (free space) areas** visible in RViz
- Robot approaches frontiers by selecting goals in safe territory **near** unexplored areas
- SLAM naturally expands the map as robot moves closer to boundaries
- **Never send goals into dark gray (unknown) or black (obstacle) areas**
- Autonomous exploration occurs by gradually moving through safe areas toward frontiers

### 4. Navigation Execution (Closes the Loop)

- Send validated goals to Nav2 navigation stack
- Monitor navigation progress and map updates during movement
- **Goal completion/abortion handling**: Upon any navigation outcome (success/failure/abortion)
  - **Immediately capture updated map state** from SLAM system
  - **Get current robot position** after navigation attempt
  - **Trigger fresh environmental analysis** using all newly acquired spatial data
  - **NO delays** between navigation completion and new analysis cycle

**Closing the Loop:** Navigation completion is the trigger that restarts the entire cycle with updated knowledge, ensuring each subsequent goal selection benefits from expanded map coverage and improved spatial understanding.

## Logging Requirements

### Terminal Output (Main Launch Terminal)

The main terminal where `b4m_launch.sh` is executed must display:

**Startup:**
```
🧭 B4M NAVIGATION EXPLORE MODE
======================================
Launching Navigation 2 with Cartographer SLAM for B4M API-guided exploration

Step 1: Starting Micro-ROS Agent...
✅ Micro-ROS Agent started (PID: 12345)

Step 7: Starting B4M Exploration Spatial Analysis
✅ B4M spatial analysis started (PID: 23456)

🤖 AUTONOMOUS NAVIGATION ACTIVE
```

**During Operation:**
The main terminal must display detailed B4M API interaction:

```
🔍 ENVIRONMENTAL ANALYSIS
Current Position: (2.35, 1.82) facing 15° northeast

📤 B4M API PROMPT:
========================================
You are a robot explorer. Analyze the environment and select the next navigation goal.

AVAILABLE SAFE NAVIGATION OPTIONS:
• FRONT (0°): Safe navigation to explored area 3.2m ahead - leads toward frontier
• RIGHT (90°): Safe navigation to explored corridor 2.8m away
• BACK-RIGHT (135°): Safe navigation to explored area 4.2m away - leads toward frontier  
• BACK-LEFT (225°): Safe navigation to explored space 2.1m away
• LEFT (270°): Safe navigation to explored area 3.5m away - leads toward frontier

EXPLORATION STATUS:
• Unexplored frontiers detected near: 45°, 135°, 270°
• Most promising exploration direction: Southeast (135°) - safe path available
• Current room appears 85% mapped

NOTE: All directions above are PRE-VALIDATED as safe. Choose any option.

Select navigation goal (1-5m distance, relative bearing ±180°):
========================================

🧠 Waiting for B4M API response... (timeout: 120s)

📥 B4M API RESPONSE (1.8s):
{
  "selected_destination": 3,
  "reasoning": "Moving southeast toward frontier to expand map coverage"
}

✅ Goal validated: Moving 3.5m at 135° (southeast)
🎯 Target coordinates: (4.83, -0.65) facing 180°
🚀 Sending navigation goal to Nav2...

🛤️ NAVIGATION IN PROGRESS
- Current progress: 45% complete
- ETA: 12 seconds

✅ NAVIGATION COMPLETED
Final position: (4.81, -0.68) facing 182°
Distance traveled: 3.4m in 15.2s
```

**Error States:**
```
❌ B4M API SERVICE UNAVAILABLE
==================================================
B4M API service is not responding
Navigation system stopping - no fallback movement
==================================================

❌ INVALID B4M API RESPONSE
📥 B4M API RESPONSE (2.3s):
{
  "selected_destination": 15,
  "reasoning": "Invalid selection outside available options"
}

⚠️ Goal rejected: Selection 15 > maximum options (10)
🔄 Generating rotation goal as fallback
🎯 Fallback goal: Rotate to 90° at current position
🚀 Sending rotation goal to Nav2...
```

### Detailed Log Files

**Location:** `logs/b4m_spatial_YYYYMMDD_HHMMSS.log`

**Must Include:**
- **Full B4M API prompts sent (CLEARLY DISPLAYED)** - Complete prompt text must be logged with clear delimiters
- Complete B4M API responses received (same as terminal display)
- Goal validation decisions with reasoning
- Navigation state transitions
- Detailed error messages and stack traces
- Performance metrics (response times, distances, success rates)

**Log Entry Format:**
```
[TIMESTAMP] [LEVEL] [MODULE]: Message
[2025-09-03 14:23:45.123] [INFO] [b4m_explore]: 🔍 ENVIRONMENTAL ANALYSIS
[2025-09-03 14:23:45.124] [INFO] [b4m_explore]: Current Position: (2.35, 1.82) facing 15° northeast
[2025-09-03 14:23:45.234] [INFO] [b4m_explore]: 📤 B4M API PROMPT:
[2025-09-03 14:23:45.235] [DEBUG] [b4m_explore]: You are a robot explorer. Analyze the environment...
[2025-09-03 14:23:46.567] [INFO] [b4m_explore]: 📥 B4M API RESPONSE (1.3s):
[2025-09-03 14:23:46.568] [DEBUG] [b4m_explore]: {"selected_destination": 3, "reasoning": "Moving southeast..."}
[2025-09-03 14:23:46.570] [INFO] [b4m_explore]: ✅ Goal validated: Moving 3.5m at 135° (southeast)
```

**Important:** The main terminal output and log file should contain identical information for transparency and debugging purposes.

### ROS2 Console Output

The ROS2 node must output to console with clear emoji indicators:
- 🔍 Analysis in progress
- 🧠 Querying B4M API
- ✅ Success states
- ⚠️ Warnings
- ❌ Errors
- 🎯 Goal navigation
- 🔄 Rotation goals
- 📍 Position updates

## State Management

### System States

1. **INITIALIZING** - System startup, waiting for sensors
2. **INITIAL_MAPPING** - Executing 0.5m square mapping sequence at startup
3. **ANALYZING** - Performing spatial analysis
4. **QUERYING_B4M** - Waiting for B4M API response
5. **NAVIGATING** - Executing navigation goal
6. **ERROR** - Handling failures
7. **SHUTDOWN** - Clean termination

### State Transitions

Each state change must be:
- Logged to file with timestamp
- Displayed in terminal (major states only)
- Include relevant context (position, goal, etc.)

## Error Handling

### B4M API Failures
- **Connection Error**: Log, display error, stop navigation (no autonomous retry)
- **Timeout**: Log, display error, stop navigation
- **Invalid Response**: Log full response, generate rotation goal as fallback

### Navigation Failures
- **Goal Rejected**: Log reason, immediately analyze and request new goal
- **Path Planning Failed**: Log, abort goal, trigger immediate re-analysis
- **Obstacle Detected**: Abort current goal, immediate environmental re-analysis

### Recovery Behavior
- No autonomous fallback movements when B4M API is unavailable
- System stops and waits for manual intervention or API restoration
- On navigation failures, immediate re-analysis and new goal selection
- Clear error messages in main terminal
- Detailed diagnostics in log file

### Critical Navigation Timing Issue (MUST AVOID)

**Problem:** Navigation goals completing instantaneously without robot movement

**Symptoms:**
- Goals transition from NAVIGATING → ANALYZING in milliseconds
- Robot position remains unchanged despite goal acceptance
- Endless loop of B4M API queries with identical environmental data
- Navigation "completes" but robot never physically moves

**Root Cause:** Implementation incorrectly treats Nav2 goal **acceptance** as goal **completion**

**MANDATORY Fix Requirements:**
```
WRONG: Goal accepted by Nav2 → immediately trigger re-analysis
CORRECT: Goal accepted by Nav2 → wait for actual completion/abortion → then trigger re-analysis
```

**Implementation Requirements:**
- MUST distinguish between goal acceptance and goal completion
- MUST wait for Nav2 action server to report actual goal status
- MUST NOT transition to ANALYZING state until robot has physically moved or goal genuinely fails
- MUST monitor actual robot position changes during navigation
- MUST implement proper Nav2 action client with result callbacks

**State Transition Fix:**
```
❌ WRONG PATTERN:
QUERYING_B4M → NAVIGATING (goal sent)
NAVIGATING → ANALYZING (goal accepted - WRONG!)

✅ CORRECT PATTERN:  
QUERYING_B4M → NAVIGATING (goal sent)
NAVIGATING → NAVIGATING (goal accepted - wait for completion)
NAVIGATING → ANALYZING (goal completed/aborted - robot moved or failed)
```

**Debugging Requirements:**
- Log actual robot position before and after each navigation attempt
- Compare position changes to validate actual movement occurred
- Log Nav2 action server status transitions (accepted → active → succeeded/aborted)
- Detect and alert when robot position is unchanged despite "successful" navigation

### Critical Position Tracking Requirements (MANDATORY)

**Problem:** Using hardcoded/placeholder position data instead of real robot position

**Symptoms:**
- Robot position never changes despite navigation commands
- All goals appear at same coordinates regardless of actual robot location
- Rotation goals fail because Nav2 receives "zero distance" commands
- System cannot track actual robot movement or validate goal success

**Root Cause:** Implementation uses placeholder functions returning static values instead of real TF/odometry data

**MANDATORY Implementation Requirements:**

**Real Position Tracking:**
```python
# WRONG: Hardcoded placeholders
def get_current_position(self):
    return (2.35, 1.82)  # Static value - WRONG!
    
# CORRECT: Real TF/odometry lookup
def get_current_position(self):
    try:
        # Try TF transform (map → base_link) 
        transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        return (transform.transform.translation.x, transform.transform.translation.y)
    except tf2_ros.LookupException:
        # Fallback to odometry
        if self.latest_odom_data:
            return (self.latest_odom_data.pose.pose.position.x, 
                   self.latest_odom_data.pose.pose.position.y)
```

**Movement Validation Requirements:**
- MUST track position at goal start vs goal completion
- MUST calculate actual distance moved and rotation achieved
- MUST detect when robot position is unchanged (movement < 0.1m, rotation < 5°)
- MUST log position changes for all navigation attempts
- MUST provide clear warnings when "successful" goals result in no movement

**TF Integration Requirements:**
- MUST subscribe to `/odom` topic as fallback
- MUST implement TF2 buffer and listener for `map → base_link` transforms
- MUST handle TF lookup exceptions gracefully with odometry fallback
- MUST cache position updates during environmental analysis
- MUST use quaternion-to-euler conversion for heading calculations

**Rotation Goal Handling:**
- MUST ensure rotation goals use exact current position (not target coordinates)
- MUST clearly distinguish rotation vs movement goals in execution
- MUST provide specialized logging for pure rotation commands
- MUST validate rotation goals achieve actual heading changes

### Enhanced Error Handling Requirements

**Configuration Errors (CRITICAL):**
- MUST catch KeyError exceptions when accessing config
- MUST provide clear error messages showing expected vs actual keys
- MUST validate all required configuration keys on startup
- MUST fail fast with helpful diagnostics if config is invalid

**Goal Validation Error Recovery:**
- When goal validation fails, system MUST continue operation (not enter ERROR state)
- Fallback rotation goals MUST use same data structure as movement goals
- State transitions after goal validation failures MUST return to ANALYZING state
- System MUST NOT remain in ERROR state for recoverable failures

**State Transition Rules (MANDATORY):**
```
ANALYZING → QUERYING_B4M (always on analysis trigger)
QUERYING_B4M → NAVIGATING (on valid goal received)
QUERYING_B4M → ANALYZING (on fallback goal generated)  
QUERYING_B4M → ERROR (only on B4M API unavailable/timeout)
NAVIGATING → ANALYZING (on goal completion/abortion - immediate trigger)
ERROR → ANALYZING (only on manual intervention or service restoration)
```

**Forbidden State Transitions:**
- NEVER: QUERYING_B4M → ERROR for goal validation failures
- NEVER: Goal validation failure should stop the exploration loop
- NEVER: Remain in ERROR state for recoverable conditions

## Configuration

### File: `config/b4m_nav_config.yaml`

**Key Parameters:**
```yaml
navigation:
  rotation_probability: 0.5      # Movement vs rotation ratio
  min_goal_distance: 1.0         # Minimum goal distance (meters)
  max_goal_distance: 5.0         # Maximum goal distance (meters)
  
b4m_nav:                         # B4M API configuration section
  api_endpoint: ""               # B4M API endpoint (same as --b4m-ping)
  timeout: 120.0                 # 2-minute timeout
  
generation:
  temperature: 0.2               # API temperature for consistent responses
  top_p: 0.9                     # Top-p sampling
  max_tokens: 200                # Maximum tokens per response
  
safety:
  max_consecutive_failures: 3    # Stop after N failures
  nav2_timeout: 120.0            # Goal execution timeout
  obstacle_clearance: 0.5        # Minimum clearance from obstacles (meters)
```

### Configuration Validation Requirements

**Critical Configuration Keys - MUST Use Exact Names:**

The implementation MUST use these exact key names from the existing `config/b4m_nav_config.yaml`:

**Navigation Section:**
- `min_goal_distance`
- `max_goal_distance`
- `rotation_probability`

**B4M API Section:**
- `b4m_nav.api_endpoint`
- `b4m_nav.timeout`

**Generation Section:**
- `generation.temperature`
- `generation.top_p`
- `generation.max_tokens`

### Configuration Testing Requirements
Before implementation, verify config access:
```python
# Test all required config keys exist - MANDATORY
required_keys = [
    ('b4m_nav', 'api_endpoint'),
    ('b4m_nav', 'timeout'),
    ('navigation', 'min_goal_distance'),
    ('navigation', 'max_goal_distance'),
    ('navigation', 'rotation_probability'),
    ('generation', 'temperature'),
    ('generation', 'max_tokens')
]

for section, key in required_keys:
    try:
        value = config[section][key]
    except KeyError:
        raise ConfigError(f"Missing required config: {section}.{key}")
```

## Performance Metrics

### Session Statistics (On Shutdown)
```
🧭🤖 B4M NAVIGATION SESSION COMPLETE
===============================================================
Session Statistics:
• Total Goals Selected: 12
• Total Distance Traveled: 24.3m
• Navigation Success Rate: 91.7% (11/12 goals)
• Total navigation goals completed
• Average B4M API Response Time: 1.8s
• Session Duration: 12m 45s
===============================================================
```

### Real-time Metrics (In Logs)
- B4M API query response time
- Goal validation time
- Navigation execution time
- Distance to goal

## B4M API Prompt Generation Details

### Prompt Structure

The B4M API will receive prompts in this format:

```
You are a robot explorer. Choose your next navigation destination from the PRE-VALIDATED safe options below.

CURRENT SITUATION:
• Position: ({robot_x:.2f}, {robot_y:.2f}) facing {robot_heading_deg:.0f}°
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

**Destination List Formatting:**
```
1. Move 2.0m forward (bearing +0°) → EXPLORES NEW AREA
2. Move 2.5m forward-right (bearing +45°) → EXPLORES NEW AREA
3. Move 1.8m right (bearing +90°)
4. Move 2.2m back-right (bearing +135°)
5. Move 1.5m left (bearing -90°) → EXPLORES NEW AREA
```

**Strategic Indicators:**
- "→ EXPLORES NEW AREA" for destinations leading toward frontiers
- "→ RETURNS TO KNOWN AREA" for safer fallback options
- "→ CORRIDOR FOLLOWING" for structured navigation patterns

**Safe Destination Pre-Generation Algorithm (MANDATORY):**
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

## Development Guidelines

### CRITICAL: Start with Working `--nav` Code as Foundation

**⚠️ MANDATORY APPROACH: DO NOT implement from scratch!**

The `--b4m-nav-explore` mode MUST be built by modifying existing working code, NOT by writing new code from scratch. This prevents dependency conflicts, ensures compatibility, and leverages proven functionality.

### Implementation Checklist (MANDATORY ORDER)

**Step 1: Copy Working `--nav` Foundation**
- [ ] Start with existing `ollama_nav_controller.py` as the base file (rename to `b4m_explore_spatial.py`)
- [ ] Ensure all imports, dependencies, and patterns are already working
- [ ] Verify quaternion handling, TF transforms, and Nav2 integration work correctly
- [ ] Confirm position tracking and goal sending patterns are functional
- [ ] Test that the base code runs without tf_transformations or other external dependencies

**Step 2: Modify for Autonomous Exploration**
- [ ] Replace manual goal input with B4M API-driven goal selection
- [ ] Add environmental analysis and frontier detection
- [ ] Implement closed-loop behavior (goal completion → immediate re-analysis)
- [ ] Add proper state management for autonomous operation

**Step 3: Add B4M API Integration**
- [ ] Copy API client patterns from existing `--b4m-ping` mode
- [ ] Integrate B4M API query system with proven connection handling
- [ ] Add JSON parsing and validation using existing patterns

**❌ NEVER DO THIS:**
- Start with empty file and import tf_transformations (ROS1 dependency)
- Write custom quaternion conversion when working version exists
- Implement Nav2 action client from scratch when pattern already works
- Create new TF/odometry handling when existing code already handles it

### Leveraging Existing Infrastructure

**Reuse `--nav` Components (EXACT COPY PATTERNS):**
- **Launch sequence**: Copy from existing `--nav` mode in `b4m_launch.sh`
- **Navigation stack**: Use identical Nav2 + Cartographer configuration  
- **Goal sending**: Reuse Nav2 action client patterns from existing code (EXACT SAME APPROACH)
- **Position tracking**: Copy TF/odometry patterns from `ollama_nav_controller.py`
- **Quaternion handling**: Use existing `euler_from_quaternion()` function
- **RViz integration**: Same visualization setup as `--nav` mode

**Reference `--b4m-ping` for API Integration (EXACT COPY PATTERNS):**
- **API client**: Reuse B4M API connection patterns from `--b4m-ping` mode
- **JSON handling**: Copy request/response parsing from existing `--b4m-ping` code
- **Error handling**: Leverage timeout and connection error patterns  
- **Configuration**: Extend existing B4M configuration structure

### Code Organization
- **Main node**: `scripts/b4m_explore_spatial.py`
- **Configuration**: `config/b4m_nav_config.yaml` (create new)
- **Launch integration**: Add to `b4m_launch.sh` (copy `--nav` section, modify for B4M API)

### Key Interfaces
- **Input**: `/scan` (LIDAR), `/map` (occupancy grid - OccupancyGrid message), `/odom` (position)
- **Output**: Navigation goals to Nav2 action server (same as `--nav`)
- **Logging**: Both ROS2 logging and Python logging to file

### Research-Based Implementation Details

**Map Data Source (Confirmed):**
- Subscribe to `/map` topic (OccupancyGrid message type)
- This is the standard SLAM map output used by existing `--nav` mode
- Map data contains: width, height, resolution, origin, and occupancy data array

**Frontier Detection Method (Simple Implementation):**
- Scan occupancy grid for boundaries between free (0) and unknown (-1) space
- Use simple grid traversal to find frontier cells
- Convert grid coordinates to world coordinates for goal selection
- No complex frontier clustering required - basic boundary detection sufficient

**Simplified Goal Validation (Post-B4M API):**
- Since B4M API can only choose from pre-validated safe sectors, goal validation becomes trivial
- Simply convert bearing + distance to coordinates and send to Nav2
- **No rejection loops** - all goals are guaranteed safe by design

**Navigation Progress Monitoring (Simplified):**
- Use Nav2 action client status only: ACCEPTED → ACTIVE → SUCCEEDED/ABORTED
- Remove custom progress calculation - Nav2 provides sufficient status information
- Focus on goal completion detection rather than progress percentages

### Implementation Strategy

**🚨 CRITICAL REQUIREMENT: Use `ollama_nav_controller.py` as Starting Point**

The implementation MUST begin by copying the existing, working `ollama_nav_controller.py` file to `b4m_explore_spatial.py` and modifying it, rather than starting from scratch. This ensures:
- All ROS2 dependencies are already correct
- Quaternion handling works without external libraries
- Nav2 integration patterns are proven to work
- Position tracking and TF transforms are functional
- No tf_transformations or other compatibility issues

**Implementation Steps:**
1. **Copy `ollama_nav_controller.py` → `b4m_explore_spatial.py`** - Start with working foundation
2. **Replace Ollama client with B4M API client** - Copy patterns from `--b4m-ping` mode
3. **Add autonomous exploration logic** - Replace manual input with environmental analysis
4. **Implement closed-loop behavior** - Add goal completion → immediate re-analysis
5. **Test incrementally** - Verify each component works with existing infrastructure

### Testing in Simulation
1. Start with `--simulation` flag
2. Verify B4M API connectivity
3. Monitor exploration coverage in RViz
4. Check log files for proper formatting
5. Test error scenarios (disconnect B4M API)

## Implementation Checklist (Based on Existing Code Research)

### Pre-Implementation Verification
- [ ] Confirm `/map` topic publishes OccupancyGrid messages in current `--nav` mode
- [ ] Verify B4M API service responds (same endpoint as `--b4m-ping`)
- [ ] Test Nav2 action client functionality in existing `--nav` mode
- [ ] Create configuration file structure matching other modes

### Core Implementation Tasks
- [ ] Replace placeholder `detect_frontiers()` with occupancy grid boundary detection
- [ ] Update `build_exploration_prompt()` to match specification format exactly
- [ ] Implement `is_goal_in_safe_territory()` using `occupancy_value <= 25` validation (Nav2 standard)
- [ ] Create configuration key access for B4M API settings
- [ ] Remove navigation progress monitoring - use Nav2 status only
- [ ] Ensure closed-loop behavior with fresh map data on each cycle

### Testing Sequence
1. **Simulation Testing**: `./b4m_launch.sh --b4m-nav-explore --simulation`
2. **B4M API Connectivity**: Verify API responses and JSON parsing
3. **Frontier Detection**: Check boundary detection in RViz map visualization
4. **Goal Validation**: Test various map scenarios (free, unknown, obstacle areas)
5. **Closed-Loop Verification**: Confirm fresh map data usage between cycles

## Future Enhancements (Not Required for Initial Implementation)

- Advanced frontier clustering and prioritization
- Multi-robot coordination
- Learned exploration strategies
- Dynamic obstacle handling
- Exploration completion detection
- Map quality assessment

---

This specification defines the requirements for the `--b4m-nav-explore` mode. Focus on creating a robust, well-logged system that provides clear feedback to users and maintains safe autonomous operation using the B4M API instead of Ollama.