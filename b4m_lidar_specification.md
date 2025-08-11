# B4M LiDAR-Based Intelligent Navigation Specification

## Overview

This specification describes the integration of the B4M API with the Yahboom robot's LiDAR sensor system to enable intelligent, obstacle-aware autonomous navigation. The system analyzes LiDAR scan data to detect obstacles, describes the situation to the B4M AI API, and uses the AI's response to make turn direction decisions, replacing random exploration with intelligent navigation.

## System Architecture

### Components

1. **LiDAR Input**
   - Topic: `/scan` (LaserScan)
   - 360-degree laser scanner providing distance measurements
   - Range: 0.12m to 12m typical
   - Angular resolution: ~1 degree
   - Update rate: 10 Hz

2. **B4M API Integration**
   - Endpoint: `https://app.bike4mind.com/api/chat`
   - API Key: `b4m_live_c491719bd23cc716e2db2c5182f4f900`
   - Model: `gpt-4o-mini`
   - Purpose: Analyze obstacle patterns and provide turn direction guidance

3. **Movement Control**
   - Output topic: `/cmd_vel` (Twist)
   - Linear velocity: 0.08 m/s (slow, safe exploration)
   - Angular velocity: 0.3 rad/s (moderate turning)
   - Stop distance: 0.3048m (1 foot)
   - Resume distance: 0.4m

4. **Safety Layer**
   - Primary sensor: LiDAR scan
   - Hard stop at 30.48cm (1 foot)
   - Immediate stop override for any safety violation
   - No movement without clear LiDAR data

## LiDAR-Based Navigation Pipeline

### 1. Obstacle Detection and Analysis
```python
# Process laser scan data
ranges = np.array(msg.ranges)
# Segment scan into regions (front, left, right, back)
front_obstacle = min(front_ranges) < stop_distance
left_clear = min(left_ranges) > safe_distance
right_clear = min(right_ranges) > safe_distance
```

### 2. Situation Description
The system will generate detailed obstacle descriptions with distances:
- "Obstacle 0.25m ahead, left clear for 3.2m, right blocked at 0.4m"
- "Walls at 0.5m left and 0.6m right, narrow corridor, front blocked at 0.3m"
- "Dead end: front 0.3m, left 0.4m, right 0.5m"
- "Open left >5m, obstacle right at 0.5m, front clear for 2.1m"
- "Clear path: front >5m, left >5m, right 2.3m"

Scan regions analyzed:
- Front: -15° to +15° (critical for forward movement)
- Left: -90° to -30° (left turn availability)
- Right: +30° to +90° (right turn availability)
- Wide front: -30° to +30° (corridor detection)

### 3. API Request Format
```json
{
  "message": "I am a mobile robot navigating with LiDAR. Current situation: [obstacle description]. Should I turn left or right? Respond with only 'left' or 'right'.",
  "model": "gpt-4o-mini",
  "temperature": 0.1,
  "max_tokens": 20
}
```

Example messages:
- "I am a mobile robot navigating with LiDAR. Current situation: Obstacle 0.2m directly ahead, left path clear for 3m, right path blocked at 0.4m. Should I turn left or right? Respond with only 'left' or 'right'."
- "I am a mobile robot navigating with LiDAR. Current situation: Narrow corridor with walls 0.5m on both sides, dead end 0.3m ahead. Should I turn left or right? Respond with only 'left' or 'right'."

### 4. API Implementation Details

**Complete Python Implementation**:
```python
import requests
import json
from typing import Optional

class B4MLidarAPI:
    def __init__(self):
        self.api_url = "https://app.bike4mind.com/api/chat"
        self.api_key = "b4m_live_c491719bd23cc716e2db2c5182f4f900"
        self.timeout = 2.0  # 2 second timeout
        
    def get_turn_direction(self, obstacle_description: str) -> Optional[int]:
        """
        Query B4M API for turn direction based on obstacle description.
        
        Returns:
            1 for left turn (positive angular velocity)
            -1 for right turn (negative angular velocity)
            0 for stop (unclear response)
            None for API failure
        """
        headers = {
            'X-API-Key': self.api_key,
            'Content-Type': 'application/json'
        }
        
        data = {
            'message': f'I am a mobile robot navigating with LiDAR. Current situation: {obstacle_description}. Should I turn left or right? Respond with only "left" or "right".',
            'model': 'gpt-4o-mini',
            'temperature': 0.1,
            'max_tokens': 20
        }
        
        try:
            response = requests.post(
                self.api_url,
                headers=headers,
                json=data,
                timeout=self.timeout
            )
            
            # Check for successful response
            if response.status_code == 200:
                result = response.json()
                # The API returns a JSON object with a 'response' field
                response_text = result.get('response', '').lower()
                
                if 'left' in response_text:
                    return 1  # Turn left
                elif 'right' in response_text:
                    return -1  # Turn right
                else:
                    # Unclear response - stop
                    self.logger.warning(f"Unclear API response: {response_text}")
                    return 0
                    
            elif response.status_code == 401:
                self.logger.error("API authentication failed - invalid API key")
                return None
            elif response.status_code == 429:
                self.logger.warning("API rate limit exceeded")
                return None
            else:
                self.logger.error(f"API error: {response.status_code}")
                return None
                
        except requests.exceptions.Timeout:
            self.logger.warning("API request timed out")
            return None
        except requests.exceptions.RequestException as e:
            self.logger.error(f"API request failed: {e}")
            return None
        except json.JSONDecodeError:
            self.logger.error("Failed to parse API response")
            return None
```

**Expected API Response Format**:
```json
{
  "response": "left",
  "model": "gpt-4o-mini",
  "usage": {
    "prompt_tokens": 45,
    "completion_tokens": 1,
    "total_tokens": 46
  }
}
```

**Error Response Examples**:
```json
// 401 Unauthorized
{
  "error": "Invalid API key"
}

// 429 Too Many Requests
{
  "error": "Rate limit exceeded. Please wait before making another request."
}

// 500 Internal Server Error
{
  "error": "Internal server error"
}
```

### 5. API Response Processing

**Response Parsing Logic**:
```python
def parse_api_response(self, api_result: dict) -> int:
    """Parse API response for turn direction."""
    response_text = api_result.get('response', '').lower()
    
    # Look for direction keywords
    if 'left' in response_text:
        return 1  # Positive angular velocity
    elif 'right' in response_text:
        return -1  # Negative angular velocity
    else:
        # Stop robot if response unclear
        return 0  # 0 indicates stop
```

### 6. Movement Execution

| Situation | Action | Linear (m/s) | Angular (rad/s) | Duration |
|-----------|--------|--------------|-----------------|----------|
| Clear path ahead | Move forward | 0.08 | 0.0 | Continuous |
| Obstacle detected | Stop | 0.0 | 0.0 | Immediate |
| API says "left" | Turn left in place | 0.0 | 0.3 | Until clear |
| API says "right" | Turn right in place | 0.0 | -0.3 | Until clear |
| API timeout/error | Stop and wait | 0.0 | 0.0 | Until manual intervention |

Turn completion criteria:
- Minimum turn time: 1.0 seconds
- Front path clear (> 0.4m)
- 3 consecutive clear readings

## ROS2 Node Structure

### Node: `b4m_lidar_navigator`

#### Node Integration with API
```python
import time
from typing import Optional

class B4MLidarNavigator(Node):
    def __init__(self):
        super().__init__('b4m_lidar_navigator')
        
        # Initialize API client
        self.api_client = B4MLidarAPI()
        self.api_client.logger = self.get_logger()
        
        # API cooldown management
        self.last_api_call_time = 0
        self.api_cooldown_seconds = 20.0
        self.waiting_for_cooldown = False
        
        # Other initialization...
    
    def request_turn_direction(self, obstacle_desc: str) -> Optional[int]:
        """Request turn direction from API with cooldown management."""
        current_time = time.time()
        time_since_last_call = current_time - self.last_api_call_time
        
        if time_since_last_call < self.api_cooldown_seconds:
            remaining_time = self.api_cooldown_seconds - time_since_last_call
            self.get_logger().info(f"API cooldown: {remaining_time:.1f}s remaining")
            self.waiting_for_cooldown = True
            return None  # Must wait
        
        # Make API call
        self.last_api_call_time = current_time
        self.waiting_for_cooldown = False
        direction = self.api_client.get_turn_direction(obstacle_desc)
        
        if direction is None:
            self.get_logger().error("API call failed - stopping robot")
        elif direction == 0:
            self.get_logger().warning("Unclear API response - stopping robot")
        else:
            direction_str = "left" if direction == 1 else "right"
            self.get_logger().info(f"API decision: turn {direction_str}")
        
        return direction
```

#### Publishers
- `/cmd_vel` (geometry_msgs/Twist) - Movement commands
- `/b4m_lidar/status` (std_msgs/String) - System status
- `/b4m_lidar/obstacle_info` (std_msgs/String) - Current obstacle description
- `/b4m_lidar/api_cooldown` (std_msgs/Float32) - Seconds until next API call allowed

#### Subscribers
- `/scan` (sensor_msgs/LaserScan) - LiDAR input
- `/b4m_lidar/command` (std_msgs/String) - Manual override commands

#### Services
- `/b4m_lidar/enable` - Enable/disable intelligent navigation
- `/b4m_lidar/set_api_mode` - Switch between API and random mode

### Configuration Parameters

```yaml
b4m_lidar_navigator:
  ros__parameters:
    # API Configuration
    api_endpoint: "https://app.bike4mind.com/api/chat"
    api_key: "b4m_live_c491719bd23cc716e2db2c5182f4f900"
    model: "gpt-4o-mini"
    temperature: 0.1  # Low temperature for consistent decisions
    max_tokens: 20    # Only need "left" or "right"
    api_timeout: 2.0  # Seconds to wait for API response
    
    # LiDAR Processing
    scan_rate: 10.0  # Hz - laser scan processing rate
    stop_distance: 0.3048  # meters (1 foot)
    safe_distance: 0.4     # meters - clear path threshold
    
    # Movement Parameters
    linear_speed: 0.08   # m/s - forward speed
    angular_speed: 0.3   # rad/s - turning speed
    min_turn_time: 1.0   # seconds - minimum turn duration
    required_clear_readings: 3  # consecutive clear readings before moving
    
    # Behavior
    enable_api: true     # Use API for decisions
    api_cooldown: 20.0   # Seconds between API calls (rate limit)
    debug_mode: false    # Enable detailed logging
```

## Implementation Phases

### Phase 1: Basic Integration (Week 1)
- Adapt autonomous_exploration.py to use API decisions
- Simple obstacle detection (front blocked/clear)
- Basic API integration (left/right decision)
- Testing on real robot with safety constraints
- Stop on API failure (no fallback)

### Phase 2: Enhanced Obstacle Description (Week 2)
- Multi-region obstacle analysis (front, left, right, sides)
- Distance and angle information in descriptions
- Corridor and dead-end detection
- Improved API prompts with richer context

### Phase 3: Contextual Awareness (Week 3)
- Recent turn history (avoid oscillation)
- Stuck detection and recovery
- Open space preference
- Wall-following capability when appropriate

### Phase 4: Advanced Features (Week 4+)
- Path optimization based on historical data
- Multi-obstacle prioritization
- Integration with mapping data
- Performance metrics and learning

## Launch Integration

### Launch Command
```bash
# Launch with B4M LiDAR navigation on real robot
./b4m_HA_launch.sh --b4m-lidar --skip-agent

# Launch with debugging enabled
./b4m_HA_launch.sh --b4m-lidar --skip-agent --debug
```

The `--b4m-lidar` flag will:
1. Start all standard robot systems
2. Launch the B4M LiDAR navigator node
3. Enable API-based turn decisions
4. Skip GUI and waypoint navigation
5. Begin autonomous exploration with intelligent turning
6. Run as exclusive mode (incompatible with --explore, --regression, etc.)

### Integration with Existing Launch Script

Modify `b4m_HA_launch.sh` to add:
```bash
B4M_LIDAR=false  # New flag

# In argument parsing
--b4m-lidar)
    B4M_LIDAR=true
    shift
    ;;

# In compatibility checking
if [ "$B4M_LIDAR" = true ]; then
    if [ "$EXPLORE" = true ] || [ "$REGRESSION" = true ]; then
        echo "ERROR: --b4m-lidar is incompatible with other modes"
        exit 1
    fi
fi

# In launch sequence (after navigation)
if [ "$B4M_LIDAR" = true ]; then
    echo "🤖 Starting B4M LiDAR Navigator..."
    echo "   API cooldown: 20 seconds between calls"
    echo "   Robot will stop during cooldown if obstacles detected"
    ros2 run b4m_lidar b4m_lidar_navigator &
    B4M_LIDAR_PID=$!
    sleep 2
fi
```

## Safety Considerations

1. **Hard Safety Limits**
   - Never move if obstacle < 30.48cm (1 foot)
   - Immediate stop on sensor failure
   - No movement without valid LiDAR data
   - Angular velocity limited to prevent tipping

2. **API Failure Handling**
   - 2-second timeout on API calls
   - Stop robot completely on API failure
   - Require manual intervention to resume
   - Log all API failures for analysis
   
3. **API Cooldown Behavior**
   - 20-second minimum between API calls
   - If obstacle detected during cooldown:
     - Robot stops immediately
     - Waits for cooldown period to expire
     - Makes new API call when allowed
   - Clear status indication of cooldown state

4. **Movement Validation**
   - Verify turn direction is safe before executing
   - Check both sides have minimum clearance
   - Smooth acceleration/deceleration
   - Emergency stop always available

## Testing Procedures

### Unit Tests
1. LiDAR data processing and segmentation
2. Obstacle description generation
3. API request/response handling
4. Turn direction decision logic
5. Safety override mechanisms

### Integration Tests
1. End-to-end obstacle to turn pipeline
2. API timeout and error handling
3. Movement execution and monitoring
4. Safety system validation

### System Tests
1. Navigation in various environments:
   - Open spaces
   - Narrow corridors
   - Dead ends
   - Cluttered areas
2. Long-duration operation (1+ hours)
3. API failure recovery
4. Comparison with random exploration

## Performance Metrics

- API response time: < 2 seconds
- Decision latency: < 100ms after API response
- Turn success rate: > 90% (clear path after turn)
- Exploration efficiency: Measured vs baseline
- API call rate: ≤ 0.05 Hz (max 1 call per 20 seconds)
- Safety violations: 0 (never hit obstacles)
- Cooldown wait compliance: 100%

## Differences from Camera-Based System

| Aspect | Camera-Based (b4m_direct) | LiDAR-Based (b4m_lidar) |
|--------|---------------------------|-------------------------|
| Primary Sensor | ESP32 Camera | LiDAR Scanner |
| Data Type | Visual scenes | Distance measurements |
| API Purpose | Scene understanding | Turn direction decision |
| Complexity | High (image processing) | Low (distance analysis) |
| Initial Development | Simulation first | Real robot first |
| Information Richness | High (colors, objects) | Low (distances only) |
| Processing Load | Heavy | Light |
| Weather Sensitivity | High | Low |
| Lighting Dependency | Yes | No |

## Example Usage

### Starting the System
```bash
# On real robot
./b4m_HA_launch.sh --b4m-lidar --skip-agent

# Monitor status
ros2 topic echo /b4m_lidar/status

# View obstacle descriptions
ros2 topic echo /b4m_lidar/obstacle_info

# Check movement commands
ros2 topic echo /cmd_vel
```

### Manual Testing
```bash
# Enable/disable API mode
ros2 service call /b4m_lidar/enable std_srvs/srv/SetBool "{data: true}"

# Switch to random mode for comparison
ros2 service call /b4m_lidar/set_api_mode std_srvs/srv/SetBool "{data: false}"
```

### Monitoring
```bash
# View logs with API interactions
ros2 run b4m_lidar b4m_lidar_navigator --ros-args -p debug_mode:=true

# Record bag for analysis
ros2 bag record /scan /cmd_vel /b4m_lidar/status /b4m_lidar/obstacle_info
```

## Future Enhancements

1. **Hybrid Sensor Fusion**
   - Combine LiDAR with camera when available
   - Use ultrasonic sensors for close-range
   - IMU integration for better turning

2. **Advanced API Integration**
   - Send LiDAR visualization to API
   - Multi-step planning queries
   - Learning from navigation success

3. **Collaborative Navigation**
   - Share obstacle maps between robots
   - Coordinated exploration
   - Swarm intelligence

4. **Performance Optimization**
   - Local ML model for fast decisions
   - Predictive obstacle avoidance
   - Path planning integration

## Dependencies

- ROS2 Humble
- Python packages:
  - requests
  - numpy
  - rclpy
- Hardware:
  - LiDAR scanner (YDLidar or equivalent)
  - Computing platform with network access
  - Differential drive robot base

## Notes

- Builds on existing `autonomous_exploration.py` codebase
- Replaces random turn selection with API-driven decisions
- Maintains all safety features from original system
- API provides intelligence layer without compromising safety
- Initial implementation focuses on simplicity and reliability
- Stops safely on API failure (no degraded mode)
- All API interactions logged for analysis and improvement
- System designed for real robot deployment from day one
- No simulation phase required (but could be added later)
- Compatible with existing SLAM and mapping systems