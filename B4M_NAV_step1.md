# B4M Navigation Step 1: Simplified Spatial Interpreter

## Overview

This document describes the implementation of a simplified `b4m_spatial_interpreter` that provides text-based spatial descriptions when the robot encounters obstacles. Instead of making random turning decisions, the robot will stop and present a console-based interface for manual navigation decisions.

## System Architecture

### Components

```
┌─────────────────────┐
│   b4m_launch.sh     │
│   (--b4m-api mode)  │
└──────────┬──────────┘
           │ Launches
           ▼
┌─────────────────────┐
│ b4m_spatial_        │
│ interpreter.py      │
└──────────┬──────────┘
           │ Subscribes to
           ▼
┌─────────────────────┐
│  /scan (LaserScan)  │
│  /cmd_vel (Twist)   │
└─────────────────────┘
```

### Data Flow

1. **Normal Operation**: Robot moves forward at slow speed (0.08 m/s) - console stays quiet
2. **Obstacle Detection**: When ANY obstacle detected within 30cm (1 foot) - stops at every obstacle
3. **Stop & Describe**: Robot stops completely, generates spatial description (blocking mode)
4. **User Decision**: Console displays options, waits for user input (blocking input)
5. **Execute Turn**: Robot performs selected turn based on user choice
6. **Validate with LaserScan**: Continue turning until front path is clear (>40cm)
7. **Resume**: Robot continues forward movement when LaserScan confirms clear path

## Key Behavior Specifications

### 1. Blocking Console Mode
- When obstacle detected, robot completely stops and waits for user input
- Uses Python's `input()` function for simple blocking interaction
- No background movement while waiting for decision
- System is fully blocked until user provides input

### 2. Stop at Every Obstacle
- Stops whenever ANY obstacle comes within 30cm (front, left, or right)
- Always presents the decision menu, even if there's an obvious clear path
- Ensures consistent behavior and full user control
- No automatic decision making

### 3. Quiet Normal Operation
- During forward movement, console produces NO output
- Only displays spatial description when obstacle triggers a stop
- Keeps console clean and highlights important decision points
- No periodic status updates during normal movement

### 4. Pure Text Interface
- No references to RViz or other visualization tools
- All spatial information conveyed through text descriptions
- Self-contained console experience
- Works identically in simulation and real robot modes

### 5. User Choice Execution
- Always executes the user's selected turn direction
- Respects user decision even if that direction appears blocked
- Uses LaserScan during turn to find clear path
- May turn more than requested angle if needed for clear path

### 6. LaserScan-Based Turn Completion
- After starting turn in user-selected direction, monitors LaserScan
- Turn considered complete when front sector shows >40cm clearance
- If user selects "Turn Right 90°" but obstacle still present after 90°:
  - Continue turning right until clear path detected
  - This ensures robot always resumes with clear path ahead
- No odometry or IMU required - purely LaserScan-based

## Spatial Description Format

When an obstacle is detected, the system generates a text description based on laser scan analysis:

### Example Output

```
═══════════════════════════════════════════════════════════════
🤖 B4M SPATIAL INTERPRETER - OBSTACLE DETECTED
═══════════════════════════════════════════════════════════════

📍 Current Situation:
───────────────────────────────────────────────────────────────
FRONT:  ⚠️  BLOCKED - Wall at 0.25m (10 inches)
LEFT:   ✅ CLEAR   - Open space, nearest obstacle at 1.2m
RIGHT:  ⚠️  NARROW  - Wall at 0.4m (16 inches)
BEHIND: ✅ CLEAR   - Open space for at least 2.0m

📊 Detailed Scan Analysis:
───────────────────────────────────────────────────────────────
• Front sector (±15°):  Min: 0.25m, Avg: 0.28m
• Left sector (±45°):   Min: 1.20m, Avg: 1.85m  
• Right sector (±45°):  Min: 0.40m, Avg: 0.65m
• Laser points: 360 readings covering 360°

🎯 Navigation Options:
───────────────────────────────────────────────────────────────
1) Turn LEFT 90°  - Clear path detected
2) Turn RIGHT 90° - Narrow but passable
3) Turn AROUND 180° - Return the way you came

Please select action (1-3): _
```

## Implementation Details

### Core Functions

```python
class B4MSpatialInterpreter(Node):
    def __init__(self):
        # Initialize ROS2 node
        super().__init__('b4m_spatial_interpreter')
        
        # Robot parameters
        self.linear_speed = 0.08  # Slow speed for safety
        self.angular_speed = 0.5  # Turning speed
        self.stop_distance = 0.3  # Stop at 30cm (1 foot)
        
        # State management
        self.state = "moving_forward"  # States: moving_forward, stopped, turning
        self.turn_target = 0  # Target angle for turning
        self.turn_start_yaw = 0  # Starting yaw when turn begins
        
    def analyze_spatial_context(self, laser_data):
        """
        Analyze laser scan and generate spatial description
        Returns: Dictionary with spatial analysis
        """
        return {
            'front': self.analyze_sector(laser_data, -15, 15),
            'left': self.analyze_sector(laser_data, 45, 135),
            'right': self.analyze_sector(laser_data, -135, -45),
            'behind': self.analyze_sector(laser_data, 150, -150),
            'closest_obstacle': min_distance,
            'recommended_action': self.get_recommendation()
        }
    
    def display_spatial_description(self, spatial_context):
        """
        Display formatted spatial description in console
        """
        # Clear screen for better visibility
        print("\033[2J\033[H")  # ANSI escape codes to clear screen
        
        # Display formatted description
        print("═" * 63)
        print("🤖 B4M SPATIAL INTERPRETER - OBSTACLE DETECTED")
        print("═" * 63)
        # ... (formatted output as shown above)
        
    def get_user_decision(self):
        """
        Get navigation decision from user via console input
        Returns: Selected action (1, 2, or 3)
        """
        while True:
            try:
                choice = input("\nPlease select action (1-3): ")
                if choice in ['1', '2', '3']:
                    return int(choice)
                print("Invalid input. Please enter 1, 2, or 3.")
            except KeyboardInterrupt:
                return None  # Allow graceful exit
```

### Sector Analysis

The system divides the laser scan into sectors for analysis:

```python
def analyze_sector(self, laser_data, start_angle, end_angle):
    """
    Analyze a sector of laser scan data
    
    Args:
        laser_data: LaserScan message
        start_angle: Start angle in degrees (0° = front)
        end_angle: End angle in degrees
        
    Returns:
        Dictionary with sector analysis
    """
    # Convert angles to laser scan indices
    indices = self.angles_to_indices(start_angle, end_angle, laser_data)
    
    # Extract ranges for sector
    sector_ranges = [laser_data.ranges[i] for i in indices]
    
    # Filter invalid readings
    valid_ranges = [r for r in sector_ranges 
                   if not math.isinf(r) and not math.isnan(r) and r > 0]
    
    if not valid_ranges:
        return {'status': 'NO_DATA', 'min': None, 'avg': None}
    
    min_dist = min(valid_ranges)
    avg_dist = sum(valid_ranges) / len(valid_ranges)
    
    # Determine status
    if min_dist < 0.3:
        status = 'BLOCKED'
    elif min_dist < 0.6:
        status = 'NARROW'
    else:
        status = 'CLEAR'
    
    return {
        'status': status,
        'min': min_dist,
        'avg': avg_dist,
        'description': self.generate_description(status, min_dist)
    }
```

### Turn Execution with LaserScan Validation

```python
def execute_turn(self, turn_choice):
    """
    Execute the selected turn maneuver and validate with LaserScan
    Turn continues until front path is clear, even if exceeding initial angle
    
    Args:
        turn_choice: 1 (left 90°), 2 (right 90°), or 3 (180°)
    """
    if turn_choice == 1:  # Left 90°
        self.turn_direction = 1  # Positive angular velocity
        turn_description = "LEFT"
    elif turn_choice == 2:  # Right 90°
        self.turn_direction = -1  # Negative angular velocity
        turn_description = "RIGHT"
    else:  # Turn around 180°
        self.turn_direction = 1  # Default to left for 180°
        turn_description = "AROUND"
    
    print(f"\n🔄 Executing turn {turn_description}...")
    print("   (Will continue turning until path ahead is clear)")
    self.state = "turning"
    
def validate_turn_completion(self, laser_data):
    """
    Check if turn should complete based on LaserScan data
    Turn is complete when front sector shows clear path (>40cm)
    
    Returns:
        bool: True if front path is clear, False otherwise
    """
    front_distance = self.get_front_distance(laser_data)
    
    if front_distance > self.safe_distance:  # 0.4m clear ahead
        print(f"✅ Clear path detected ahead ({front_distance:.2f}m)")
        return True
    else:
        # Continue turning in same direction
        return False
```

## Integration with b4m_launch.sh

### Launching the Spatial Interpreter

When using `--b4m-api` mode, the script launches the spatial interpreter instead of autonomous exploration:

```bash
# In b4m_launch.sh, Step 6 for B4M API mode:
echo "🚀 Step 6: Starting B4M Spatial Interpreter with manual decisions"
cd "$WORKSPACE_ROOT" && . install/setup.bash && \
    python3 "$WORKSPACE_ROOT/scripts/b4m_spatial_interpreter.py" 2>&1 | \
    tee "$LOGS_DIR/b4m_api_spatial_$TIMESTAMP.log"
# Note: Not running in background (&) to keep console interactive
```

### Console Interaction

The spatial interpreter runs in the foreground to enable console interaction:

1. **Output Display**: Spatial descriptions appear directly in the launch terminal
2. **User Input**: The terminal waits for user input when decisions are needed
3. **Logging**: All interactions are logged to file while displaying on console

## Usage Example

### Starting the System

```bash
# For simulation
./b4m_launch.sh --b4m-api --simulation

# For real robot
./b4m_launch.sh --b4m-api
```

### Example Session

```
[Launch sequence completes...]

✅ B4M API MODE ACTIVE
══════════════════════════════════════════════════════════════
🔌 Robot starting with B4M Spatial Interpreter

[Console stays quiet while robot moves forward...]
[No output during normal movement...]
[Suddenly, when obstacle detected:]

═══════════════════════════════════════════════════════════════
🤖 B4M SPATIAL INTERPRETER - OBSTACLE DETECTED
═══════════════════════════════════════════════════════════════

📍 Current Situation:
───────────────────────────────────────────────────────────────
FRONT:  ⚠️  BLOCKED - Wall at 0.28m (11 inches)
LEFT:   ✅ CLEAR   - Doorway detected, open at 1.5m
RIGHT:  ⚠️  BLOCKED - Wall at 0.22m (9 inches)
BEHIND: ✅ CLEAR   - Corridor extends at least 3.0m

📊 Detailed Scan Analysis:
───────────────────────────────────────────────────────────────
• Front sector (±15°):  Min: 0.28m, Avg: 0.31m
• Left sector (±45°):   Min: 1.50m, Avg: 2.20m  
• Right sector (±45°):  Min: 0.22m, Avg: 0.35m
• Laser points: 360 readings covering 360°

🎯 Navigation Options:
───────────────────────────────────────────────────────────────
1) Turn LEFT 90°  
2) Turn RIGHT 90° 
3) Turn AROUND 180°

Please select action (1-3): 2

🔄 Executing turn RIGHT...
   (Will continue turning until path ahead is clear)
   
[After initial 90° turn, if still blocked:]
   Continuing turn... front still blocked at 0.35m
   Continuing turn... front still blocked at 0.42m
   
✅ Clear path detected ahead (0.85m)

[Console returns to quiet mode - no output during forward movement]
[No "Resuming forward movement" message - just continues silently]
```

## State Machine

```
┌──────────────┐
│   START      │
└──────┬───────┘
       ▼
┌──────────────┐
│   FORWARD    │◄─────────────┐
│   MOVEMENT   │              │
└──────┬───────┘              │
       │                      │
       ▼ Obstacle detected    │
┌──────────────┐              │
│    STOP &    │              │
│   DESCRIBE   │              │
└──────┬───────┘              │
       │                      │
       ▼                      │
┌──────────────┐              │
│  WAIT FOR    │              │
│   USER INPUT │              │
└──────┬───────┘              │
       │                      │
       ▼ User selects         │
┌──────────────┐              │
│   EXECUTE    │              │
│     TURN     │              │
└──────┬───────┘              │
       │                      │
       ▼ Turn complete        │
       └──────────────────────┘
```

## Configuration Parameters

```python
# Robot movement parameters
LINEAR_SPEED = 0.08      # m/s - slow for safety
ANGULAR_SPEED = 0.5      # rad/s - moderate turning
STOP_DISTANCE = 0.30     # meters - 1 foot safety margin
SAFE_DISTANCE = 0.40     # meters - resume when clear

# Sector analysis parameters  
FRONT_SECTOR_ANGLE = 15  # degrees - narrow front view
SIDE_SECTOR_ANGLE = 45   # degrees - wide side view
BLOCKED_THRESHOLD = 0.30 # meters - consider blocked
NARROW_THRESHOLD = 0.60  # meters - consider narrow

# Turn parameters
TURN_PRECISION = 0.05    # radians - turn completion tolerance
MIN_TURN_TIME = 1.0      # seconds - minimum turn duration
```

## Benefits of This Approach

1. **Human-Readable Output**: Clear text descriptions of spatial context
2. **Manual Control**: User maintains decision authority
3. **Learning Tool**: Helps understand robot perception
4. **Safe Operation**: Stops and waits for decisions
5. **Simple Interface**: Console-based, no additional UI needed
6. **Stepping Stone**: Foundation for future LLM integration

## Future Enhancement Path

This Step 1 implementation provides the foundation for:

1. **Step 2**: Add grid visualization alongside text
2. **Step 3**: Integrate with LLM for automated decisions
3. **Step 4**: Add HTTP API for remote LLM integration
4. **Step 5**: Full B4M Navigation API implementation

## Testing

### Simulation Testing

```bash
# Launch in simulation with spatial interpreter
./b4m_launch.sh --b4m-api --simulation

# Robot will move in Gazebo, console shows decisions
```

### Real Robot Testing

```bash
# Ensure safe testing area
./b4m_launch.sh --b4m-api

# Monitor console for spatial descriptions
# Make navigation decisions when prompted
```

## Troubleshooting

### Common Issues

1. **No spatial descriptions appearing**
   - Check that laser scan topic `/scan` is publishing
   - Verify the script is running (check process list)

2. **Console input not working**
   - Ensure script is running in foreground (not background)
   - Check terminal settings allow input

3. **Robot not turning after selection**
   - Verify `/cmd_vel` topic is connected
   - Check that LaserScan data is being received during turn
   - Ensure turn direction is being maintained

4. **Descriptions not accurate**
   - Calibrate laser scan parameters
   - Adjust sector angle definitions
   - Check for laser scan noise/interference

## Summary

This simplified spatial interpreter provides a manual decision interface that:
- Converts laser scan data to human-readable descriptions
- Stops when obstacles are detected
- Presents clear navigation options
- Executes user-selected turns
- Resumes autonomous forward movement

It serves as an educational tool and stepping stone toward the full B4M LLM Navigation system while providing immediate value for understanding the robot's spatial perception.