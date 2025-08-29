# OLLAMA_API.md - Ollama LLM Integration for Robot Navigation

## Overview

This specification describes the integration of Ollama Large Language Models (LLMs) with the B4M Yahboom robot navigation system. The `--ollama` mode enables autonomous navigation decisions based on spatial understanding, where the robot uses natural language descriptions of its surroundings to make intelligent movement choices through an LLM.

### Prerequisites

- **Ollama installed and running**: Ollama service must be running on localhost:11434
- **Model downloaded**: At least one compatible model (e.g., `ollama pull llama3.2`)
- **Network access**: Local network connectivity between robot system and Ollama service

### Purpose
- Replace manual navigation decisions with LLM-based autonomous decision-making
- Provide natural language understanding of spatial contexts
- Enable adaptive navigation behavior based on environmental descriptions
- Maintain safety through structured responses and fallback mechanisms

### Key Features
- Real-time spatial description generation from laser scan data
- Structured LLM prompts for consistent navigation decisions
- JSON-formatted responses for reliable command parsing
- Safety mechanisms including timeouts and manual override
- Support for multiple Ollama models (llama3.2, mistral, etc.)

## Architecture

### System Components

```
┌─────────────────┐     ┌──────────────────┐     ┌─────────────────┐
│   Laser Scan    │────▶│ Spatial Context  │────▶│  Ollama API     │
│   (360° LIDAR)  │     │   Generator      │     │   Client        │
└─────────────────┘     └──────────────────┘     └─────────────────┘
                                                           │
                                                           ▼
┌─────────────────┐     ┌──────────────────┐     ┌─────────────────┐
│  Robot Motion   │◀────│ Command Executor │◀────│ Response Parser │
│   (/cmd_vel)    │     │                  │     │                 │
└─────────────────┘     └──────────────────┘     └─────────────────┘
```

### Data Flow Pipeline

1. **Sensor Input**: 360° laser scan data from LIDAR (`/scan` topic)
2. **Spatial Analysis**: Convert raw distances to semantic descriptions
3. **Prompt Generation**: Create structured prompt with spatial context
4. **LLM Processing**: Send to Ollama API for decision-making
5. **Response Parsing**: Extract navigation command from JSON response
6. **Command Execution**: Convert to robot motion commands (`/cmd_vel`)
7. **Safety Monitoring**: Continuous obstacle detection and emergency stop

## API Integration

### Ollama API Endpoint

```python
OLLAMA_API_URL = "http://localhost:11434/api/generate"
```

### Request Format

```json
{
  "model": "llama3.2",
  "prompt": "<spatial_description_and_navigation_request>",
  "format": "json",
  "stream": false,
  "options": {
    "temperature": 0.3,
    "top_p": 0.9,
    "max_tokens": 100
  }
}
```

### Python Implementation Example

```python
import requests
import json

class OllamaNavigator:
    def __init__(self, model="llama3.2", api_url="http://localhost:11434"):
        self.model = model
        self.api_url = f"{api_url}/api/generate"
        
    def get_navigation_decision(self, spatial_context):
        prompt = self.generate_prompt(spatial_context)
        
        payload = {
            "model": self.model,
            "prompt": prompt,
            "format": "json",
            "stream": False,
            "options": {
                "temperature": 0.3,  # Lower temperature for consistent decisions
                "top_p": 0.9,
                "max_tokens": 100
            }
        }
        
        try:
            response = requests.post(
                self.api_url, 
                json=payload, 
                timeout=5.0  # 5 second timeout for safety
            )
            
            if response.status_code == 200:
                result = response.json()
                return json.loads(result['response'])
            else:
                return self.get_fallback_decision(spatial_context)
                
        except (requests.Timeout, requests.RequestException, json.JSONDecodeError):
            return self.get_fallback_decision(spatial_context)
```

## Prompt Engineering

### Prompt Template

```python
NAVIGATION_PROMPT_TEMPLATE = """
You are a navigation AI for a robot. Based on the following spatial description, 
decide the best action for the robot to take.

CURRENT SITUATION:
{spatial_description}

AVAILABLE ACTIONS:
- "turn_left": Rotate 90 degrees to the left
- "turn_right": Rotate 90 degrees to the right
- "go_straight": Continue moving forward
- "turn_around": Rotate 180 degrees

SAFETY RULES:
1. Never move forward if FRONT is BLOCKED (obstacle < 30cm)
2. Prefer turning toward the direction with more open space
3. Turn around only if all other directions are blocked
4. When path is clear, prefer going straight

Respond with a JSON object containing:
- "action": one of the available actions
- "reason": brief explanation for the decision
- "confidence": confidence level (0.0 to 1.0)

Example response:
{{"action": "turn_left", "reason": "Front blocked, left side clear", "confidence": 0.95}}
"""
```

### Spatial Description Format

```python
def format_spatial_description(spatial_context):
    """Convert spatial context to natural language description"""
    description = []
    
    # Front status
    front = spatial_context['front']
    description.append(f"FRONT: {front['description']}")
    
    # Side status
    left = spatial_context['left']
    right = spatial_context['right']
    description.append(f"LEFT: {left['description']}")
    description.append(f"RIGHT: {right['description']}")
    
    # Behind status (for context)
    behind = spatial_context['behind']
    description.append(f"BEHIND: {behind['description']}")
    
    # Additional context
    if spatial_context.get('distance_traveled', 0) > 0:
        description.append(f"DISTANCE TRAVELED: {spatial_context['distance_traveled']:.2f}m since last stop")
    
    return "\n".join(description)
```

## Response Format

### Expected JSON Response Structure

```json
{
  "action": "turn_left|turn_right|go_straight|turn_around",
  "reason": "Brief explanation of the decision",
  "confidence": 0.95
}
```

### Response Validation

```python
def validate_response(response):
    """Validate and sanitize LLM response"""
    valid_actions = ["turn_left", "turn_right", "go_straight", "turn_around"]
    
    # Check required fields
    if not isinstance(response, dict):
        return None
        
    if "action" not in response:
        return None
        
    # Validate action
    if response["action"] not in valid_actions:
        return None
        
    # Validate confidence (optional but recommended)
    if "confidence" in response:
        try:
            confidence = float(response["confidence"])
            if not 0.0 <= confidence <= 1.0:
                response["confidence"] = 0.5
        except (ValueError, TypeError):
            response["confidence"] = 0.5
    else:
        response["confidence"] = 0.5
        
    return response
```

## Configuration

### Mode Activation

The Ollama mode is activated by launching with the `--ollama` flag:
```bash
./b4m_launch.sh --ollama              # Real robot with Ollama
./b4m_launch.sh --ollama --simulation  # Simulation with Ollama

# For debugging with verbose output:
./b4m_launch.sh --ollama --simulation --debug-verbose
```

This flag causes `b4m_launch.sh` to pass `--ollama-mode` argument to the spatial interpreter.

### Configuration File Location

The configuration file should be placed at:
```
/home/mike/projects/b4m_yahboom/config/ollama_config.yaml
```

### Configuration File (ollama_config.yaml)

```yaml
ollama:
  host: localhost
  port: 11434
  model: llama3.2:latest
  timeout: 15.0  # Increased to 15s for llama3.2 model response time
  
generation:
  temperature: 0.1  # Very low for deterministic obstacle avoidance
  top_p: 0.9
  max_tokens: 100
  format: json
  stream: false
  
navigation:
  confidence_threshold: 0.7
  fallback_mode: stop  # stop, manual, or random
  retry_attempts: 0  # No retries - stop immediately on failure
  
safety:
  max_response_time: 3.0  # Maximum time to wait for Ollama (safety cutoff)
  emergency_stop_distance: 0.10
  enable_manual_override: true
  stop_while_thinking: true  # Robot stops while waiting for Ollama response
  rate_limiting:
    min_ollama_interval: 2.0  # Minimum seconds between Ollama calls
    obstacle_detection_debounce: 3  # Require 3 consistent readings
```

## Implementation Integration Points

### Modified b4m_launch.sh

The launch script needs to pass the `--ollama-mode` flag to the spatial interpreter when running in Ollama mode:

```bash
# In the Ollama mode section of b4m_launch.sh
# Step 6: Start Ollama Spatial Interpreter in foreground
echo "🦙 Step 6: Starting Ollama Spatial Interpreter"
echo "   All output and interaction will happen in this terminal"

# Run the spatial interpreter with --ollama-mode flag and unbuffered output
python3 -u "$WORKSPACE_ROOT/scripts/b4m_spatial_interpreter.py" --ollama-mode 2>&1 | tee "$LOGS_DIR/ollama_spatial_$TIMESTAMP.log"
```

### Modified b4m_spatial_interpreter.py

Key modifications needed:

1. **Add Ollama client initialization**:
```python
def __init__(self):
    super().__init__('b4m_spatial_interpreter')
    
    # Check if running in Ollama mode via command-line argument
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--ollama-mode', action='store_true', 
                       help='Enable Ollama LLM navigation mode')
    args, unknown = parser.parse_known_args()
    
    self.ollama_mode = args.ollama_mode
    
    if self.ollama_mode:
        # Load configuration
        self.load_ollama_config()
        self.ollama_navigator = OllamaNavigator(self.config)
        self.get_logger().info("🦙 Ollama mode activated")
        print("\n🦙 OLLAMA MODE ACTIVATED")
        print("=" * 63)
        print("Robot will use Ollama LLM for navigation decisions")
        print(f"Model: {self.config['ollama']['model']}")
        print(f"API: {self.config['ollama']['host']}:{self.config['ollama']['port']}")
        print("=" * 63)
```

2. **Replace manual decision with Ollama decision**:
```python
def get_navigation_decision(self, spatial_context):
    """Get navigation decision from Ollama or manual input"""
    if self.ollama_mode:
        # Display current situation
        self.display_spatial_description(spatial_context)
        
        # Show prompt being sent to Ollama
        print("\n📝 OLLAMA PROMPT:")
        print("-" * 63)
        prompt = self.ollama_navigator.generate_prompt(spatial_context)
        print(prompt)
        print("-" * 63)
        
        # Get Ollama decision
        print("\n🦙 Consulting Ollama for navigation decision...")
        print("   (Robot stopped while waiting for response)")
        
        decision = self.ollama_navigator.get_navigation_decision(spatial_context)
        
        if decision:
            print("\n✅ OLLAMA RESPONSE:")
            print("-" * 63)
            print(f"   Action: {decision['action'].upper()}")
            print(f"   Reason: {decision.get('reason', 'No reason provided')}")
            print(f"   Confidence: {decision.get('confidence', 0.5):.2f}")
            print("-" * 63)
            return self.map_ollama_to_choice(decision['action'])
        else:
            print("\n🛑 OLLAMA UNAVAILABLE - STOPPING")
            print("   Ollama did not respond within timeout period")
            print("   Robot stopping for safety")
            return None  # Stop the robot
    else:
        return self.get_user_decision()
```

3. **Map Ollama actions to existing choice system**:
```python
def map_ollama_to_choice(self, action):
    """Map Ollama action strings to choice numbers"""
    mapping = {
        "turn_left": 1,
        "turn_right": 2,
        "turn_around": 3,
        "go_straight": 4
    }
    return mapping.get(action, 4)  # Default to forward if unknown
```

## Error Handling

### Timeout Handling
```python
try:
    response = requests.post(url, json=payload, timeout=5.0)
except requests.Timeout:
    self.get_logger().warn("Ollama request timed out, using fallback")
    return self.get_fallback_decision(spatial_context)
```

### Invalid Response Handling
```python
if not validate_response(parsed_response):
    self.get_logger().warn("Invalid Ollama response, using fallback")
    return self.get_fallback_decision(spatial_context)
```

### Fallback Strategy

When Ollama fails or times out, the system will:
1. **Stop immediately** - Robot halts all movement
2. **Alert the operator** - Display clear error message in console
3. **Log the failure** - Record the error for debugging
4. **Require manual intervention** - Operator must restart or switch to manual mode

### Safety Mechanisms

1. **Response Timeout**: Maximum 5 seconds for LLM response (3 seconds safety cutoff)
2. **Rate Limiting**: Minimum 2 seconds between Ollama API calls to prevent system overload
3. **Obstacle Detection Debouncing**: Requires 3 consistent readings to confirm obstacles
4. **Confidence Threshold**: Ignore low-confidence decisions
5. **Manual Override**: Allow user to interrupt at any time
6. **Emergency Stop**: Immediate stop if obstacle detected during execution
7. **Validation**: Strict response format validation
8. **Debug Verbosity Control**: Minimal output by default, verbose mode available with `--debug-verbose`

## Console Output and User Experience

### User Experience Scenarios

The following sections show what users will see in their terminal when running `./b4m_launch.sh --ollama --simulation`.

#### Scenario 1: Robot Moving Freely (No Obstacles)

When the robot is moving forward without encountering obstacles, the console output is minimal to prevent information overload:

```
🦙 OLLAMA MODE ACTIVATED
===============================================================
Robot will use Ollama LLM for navigation decisions
Model: llama3.2:latest
API: localhost:11434
===============================================================

[14:32:15.123] 🤖 B4M Spatial Interpreter started
[14:32:15.456] 📡 Laser scan data received - robot moving forward
[14:32:25.789] 📏 Progress: 2.5m traveled, continuing forward
[14:32:35.012] 📏 Progress: 5.0m traveled, continuing forward
[14:32:45.345] 📏 Progress: 7.5m traveled, continuing forward
```

**Expected delays:**
- Initial startup: 2-3 seconds
- Progress updates: Every 10 seconds while moving freely
- No Ollama calls during free movement (saves API calls and system resources)

#### Scenario 2: Robot Encounters Wall (Obstacle Detection)

When the robot detects a wall or obstacle in front, it triggers the Ollama decision-making process:

```
[14:33:12.678] 🚨 Obstacle detected - stopping for analysis

===============================================================
🤖 B4M SPATIAL INTERPRETER - OBSTACLE DETECTED
===============================================================

📏 Movement Since Last Stop:
---------------------------------------------------------------
• Distance traveled: 3.42m forward
• Time elapsed: 22.3 seconds
• Average speed: 0.153 m/s

📍 Current Situation:
---------------------------------------------------------------
FRONT:  ⚠️ BLOCKED - Wall at 0.25m (10 inches)
LEFT:   ✅ CLEAR   - Open space, nearest obstacle at 1.23m
RIGHT:  ⚠️ NARROW  - Wall at 0.45m (18 inches)
BEHIND: ✅ CLEAR   - Open space for at least 2.1m

📊 Detailed Scan Analysis:
---------------------------------------------------------------
• Front sector (±15°):  Min: 0.25m, Avg: 0.28m
• Left sector (67.5°-112.5°):   Min: 1.23m, Avg: 1.85m
• Right sector (-112.5°--67.5°):  Min: 0.45m, Avg: 0.67m
• Laser points: 360 readings covering 360°

🎯 Navigation Options:
---------------------------------------------------------------
1) Turn LEFT 90°
2) Turn RIGHT 90°
3) Turn AROUND 180°
4) Move FORWARD - Continue straight for 5 feet or until obstacle

[14:33:13.789] 🦙 Consulting Ollama for navigation decision...
[14:33:13.790]    (Robot stopped while waiting for response)

✅ OLLAMA RESPONSE: (received in 1.2s)
---------------------------------------------------------------
   Action: TURN_LEFT
   Reason: Front blocked, left side has most open space
   Confidence: 0.92
---------------------------------------------------------------

[14:33:15.012] 🔄 Executing turn LEFT...
[14:33:15.013]    (Will continue turning until path ahead is clear)

[14:33:18.456] ✅ Turn complete - path ahead is clear
[14:33:18.457] ➡️ Resuming forward movement

[14:33:28.789] 📏 Progress: 2.1m traveled, continuing forward
```

**Expected delays:**
- Obstacle detection to analysis display: < 0.5 seconds
- Analysis display to Ollama query: < 0.2 seconds  
- Ollama response time: 1-3 seconds (typically 1.2s)
- Response parsing to action execution: < 0.1 seconds
- Turn execution time: 3-4 seconds for 90° turn
- Resume forward movement: Immediate after turn completion

#### Error Scenario: Ollama Unavailable

If Ollama fails to respond or times out:

```
[14:35:45.123] 🚨 Obstacle detected - stopping for analysis

[Analysis display as above...]

[14:35:46.234] 🦙 Consulting Ollama for navigation decision...
[14:35:46.235]    (Robot stopped while waiting for response)

[14:35:51.240] ⏰ Rate limiting: Waiting 0.8s before next Ollama call
[14:35:52.045] 🦙 Consulting Ollama for navigation decision...

[14:35:57.050] 🛑 OLLAMA UNAVAILABLE - STOPPING
   Ollama did not respond within timeout period (5.0s)
   Robot stopping for safety
   
⚠️ MANUAL INTERVENTION REQUIRED
   Please check Ollama service status or restart in manual mode
   Use Ctrl+C to exit, then restart without --ollama flag
```

**Expected delays:**
- Initial Ollama timeout: 5 seconds
- Rate limiting wait: 0.8 seconds (if needed)
- Second attempt timeout: 5 seconds
- Total delay before stopping: ~11 seconds maximum

### Output Flow

When running in Ollama mode, the console displays:

1. **Startup Message**:
```
🦙 OLLAMA MODE ACTIVATED
===============================================================
Robot will use Ollama LLM for navigation decisions
Model: llama3.2
API: localhost:11434
===============================================================
```

2. **Per-Obstacle Output**:
```
===============================================================
🤖 B4M SPATIAL INTERPRETER - OBSTACLE DETECTED
===============================================================

📏 Movement Since Last Stop:
---------------------------------------------------------------
• Distance traveled: 2.34m forward
• Time elapsed: 15.2 seconds
• Average speed: 0.154 m/s

📍 Current Situation:
---------------------------------------------------------------
FRONT:  ⚠️ BLOCKED - Wall at 0.25m (10 inches)
LEFT:   ✅ CLEAR   - Open space, nearest obstacle at 1.23m
RIGHT:  ⚠️ NARROW  - Wall at 0.45m (18 inches)
BEHIND: ✅ CLEAR   - Open space for at least 2.1m

📝 OLLAMA PROMPT:
---------------------------------------------------------------
[Full prompt text shown here]
---------------------------------------------------------------

🦙 Consulting Ollama for navigation decision...
   (Robot stopped while waiting for response)

✅ OLLAMA RESPONSE:
---------------------------------------------------------------
   Action: TURN_LEFT
   Reason: Front blocked, left side has most open space
   Confidence: 0.92
---------------------------------------------------------------

🔄 Executing turn LEFT...
```

3. **Error Output**:
```
🛑 OLLAMA UNAVAILABLE - STOPPING
   Ollama did not respond within timeout period
   Robot stopping for safety
```

### Terminal Requirements

- The output is designed for standard terminal width (80 characters)
- Uses UTF-8 encoding for emoji display
- Requires terminal with color support for optimal readability

## Testing Strategy

### Unit Tests
- Test prompt generation with various spatial contexts
- Test response parsing with valid/invalid JSON
- Test fallback mechanisms
- Test timeout handling

### Integration Tests
- Test with actual Ollama API
- Test with different models
- Test failure scenarios
- Test safety mechanisms

### System Tests
- Full navigation loop in simulation
- Response time measurements
- Decision quality assessment
- Safety validation

## Model Recommendations

### Recommended Models
1. **llama3.2** - Fast, reliable, good spatial understanding
2. **mistral** - Excellent reasoning, slightly slower
3. **phi3** - Very fast, suitable for real-time navigation

### Model Selection Criteria
- Response time < 2 seconds
- Consistent JSON formatting
- Good spatial reasoning
- Low resource usage

## Known Issues and Solutions

### System Stability Issues (Fixed)
- **Problem**: Running `--ollama` mode caused system disruption and unresponsiveness
- **Root Causes**:
  - Python output buffering caused delayed console output
  - Sensor noise flicker between 0.28-0.31m around 0.30m threshold caused rapid obstacle detection
  - Excessive API calls during sensor noise flicker (800% CPU usage)
- **Solutions Applied**:
  - Added Python -u flag for unbuffered output in launch script
  - Implemented hysteresis: detect obstacles at 0.28m, clear at 0.32m
  - Added print_flush() function for real-time console output
  - Increased Ollama timeout to 15s for llama3.2 model
  - Optimized prompt from 1189 to ~350 characters for faster responses

### Laser Data Filtering Issues (Fixed)
- **Problem**: Real robot detected false obstacles at 5 inches (0.12m) causing immediate stops
- **Root Cause**: 
  - LIDAR sensor `range_min` is ~0.12m, but readings at/below this threshold are sensor noise or ground reflections
  - Original code incorrectly treated readings at `range_min` as legitimate obstacles
  - `--explore` mode worked because it properly filtered these readings as invalid
- **Inconsistent Behavior**:
  ```python
  # WRONG (original --ollama mode):
  ranges[ranges == 0.0] = msg.range_min  # Treated noise as obstacle!
  
  # CORRECT (--explore mode and fixed --ollama):
  ranges[ranges == 0.0] = msg.range_max  # Treat noise as clear space
  ranges[ranges <= msg.range_min] = msg.range_max  # Filter sensor noise
  ```
- **Solution Applied**:
  - Modified laser data preprocessing in `b4m_spatial_interpreter.py`
  - Now filters out readings at or below `range_min` (0.12m) as invalid sensor noise
  - Aligns `--ollama` behavior with working `--explore` mode behavior
  - Real robot now operates correctly without false positive obstacle detection

### Simulation vs Real Robot Differences
- **Why `--simulation` worked but real robot failed**:
  - **Gazebo Classic**: Provides mathematically perfect laser data without sensor noise
  - **Real Robot**: Has physical sensor limitations including ground reflections, electrical noise, and minimum range artifacts
  - **Key Insight**: Simulation testing may not reveal real-world sensor noise issues
- **Recommendation**: Always test robot navigation modes on actual hardware to validate sensor data handling

### Console Output Management
- **Default Mode**: Minimal output showing only navigation decisions and actions
- **Debug Mode**: Use `--debug-verbose` flag for detailed diagnostic output
- **Rate Limiting**: Prevents console flooding from rapid obstacle detection changes

## Future Enhancements

1. **Vision Integration**: Add camera input to prompts
2. **Learning**: Store and learn from navigation decisions
3. **Multi-modal**: Combine LIDAR with other sensors
4. **Contextual Memory**: Remember previous locations
5. **Goal-oriented**: Navigate to specific destinations
6. **Natural Language Commands**: Accept voice/text navigation goals

## Appendix: Complete Example

### Full Navigation Loop

```python
# 1. Get laser scan data
laser_data = self.laser_data

# 2. Generate spatial context
spatial_context = {
    'front': self.analyze_front_sector(laser_data),
    'left': self.analyze_left_sector(laser_data),
    'right': self.analyze_right_sector(laser_data),
    'behind': self.analyze_behind_sector(laser_data),
    'distance_traveled': self.distance_traveled
}

# 3. Create prompt
prompt = NAVIGATION_PROMPT_TEMPLATE.format(
    spatial_description=format_spatial_description(spatial_context)
)

# 4. Get Ollama decision
response = requests.post(
    "http://localhost:11434/api/generate",
    json={
        "model": "llama3.2",
        "prompt": prompt,
        "format": "json",
        "stream": False
    },
    timeout=5.0
)

# 5. Parse response
decision = json.loads(response.json()['response'])

# 6. Execute action
if decision['action'] == 'turn_left':
    self.execute_turn(1)
elif decision['action'] == 'turn_right':
    self.execute_turn(2)
elif decision['action'] == 'turn_around':
    self.execute_turn(3)
else:  # go_straight
    self.execute_forward()
```

---

This specification provides a complete blueprint for integrating Ollama LLM capabilities into the B4M Yahboom robot navigation system, enabling intelligent autonomous navigation based on natural language understanding of spatial contexts.