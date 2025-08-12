# B4M LiDAR Intelligent Navigation

LiDAR-based intelligent navigation system using B4M API for turn direction decisions.

## Features

- Obstacle detection using LiDAR sensor
- Intelligent turn decisions via B4M API
- 20-second cooldown between API calls
- Safety-first design with immediate stop on obstacles
- Real-time status monitoring via ROS2 topics

## Usage

### Launch with b4m_HA_launch.sh (Recommended)

```bash
# For real robot
./b4m_HA_launch.sh --b4m-lidar --skip-agent

# For simulation (Gazebo Classic)
./b4m_HA_launch.sh --b4m-lidar --simulation

# With debug output
./b4m_HA_launch.sh --b4m-lidar --skip-agent --debug
./b4m_HA_launch.sh --b4m-lidar --simulation --debug
```

### Manual Launch

```bash
# Source workspace
source install/setup.bash

# Run the navigator (real robot)
ros2 run b4m_lidar b4m_lidar_navigator

# Run the navigator (simulation)
ros2 run b4m_lidar b4m_lidar_navigator --ros-args -p use_sim_time:=true

# Or use launch file
ros2 launch b4m_lidar b4m_lidar_launch.py

# Launch file with simulation
ros2 launch b4m_lidar b4m_lidar_launch.py use_sim_time:=true
```

### Monitoring

```bash
# View status
ros2 topic echo /b4m_lidar/status

# View obstacle descriptions
ros2 topic echo /b4m_lidar/obstacle_info

# View API cooldown timer
ros2 topic echo /b4m_lidar/api_cooldown

# View movement commands
ros2 topic echo /cmd_vel
```

### Control Commands

```bash
# Stop navigation
ros2 topic pub -1 /b4m_lidar/command std_msgs/String '{data: stop}'

# Start navigation
ros2 topic pub -1 /b4m_lidar/command std_msgs/String '{data: start}'

# Reset state
ros2 topic pub -1 /b4m_lidar/command std_msgs/String '{data: reset}'
```

### Services

```bash
# Enable/disable navigation
ros2 service call /b4m_lidar/enable std_srvs/srv/SetBool "{data: true}"

# Enable/disable API mode (false = random turns for testing)
ros2 service call /b4m_lidar/set_api_mode std_srvs/srv/SetBool "{data: false}"
```

## Configuration

Edit `config/b4m_lidar_params.yaml` to adjust:
- API cooldown time (default: 20 seconds)
- Movement speeds
- Stop/safe distances
- Debug mode

## Safety Features

- Immediate stop when obstacle detected within 30.48cm (1 foot)
- Robot stops completely during API cooldown if obstacle detected
- Manual override always available
- Automatic stop on API failure

## API Integration

The system communicates with:
- Endpoint: `https://app.bike4mind.com/api/chat`
- Model: `gpt-4o-mini`
- Cooldown: 20 seconds between requests

## Troubleshooting

### Robot not moving
- Check if obstacle is detected: `ros2 topic echo /b4m_lidar/obstacle_info`
- Check navigation state: `ros2 topic echo /b4m_lidar/status`
- Verify API is enabled: `ros2 service call /b4m_lidar/set_api_mode std_srvs/srv/SetBool "{data: true}"`

### API errors
- Check logs: `ros2 run b4m_lidar b4m_lidar_navigator --ros-args -p debug_mode:=true`
- Verify API key is correct in config
- Check network connectivity

### Testing without API
- Disable API mode to use random turns: `ros2 service call /b4m_lidar/set_api_mode std_srvs/srv/SetBool "{data: false}"`

## Debugging and Logging

### B4M API Request Logs
All API requests and responses are automatically logged to dedicated files:
- **Location**: `/tmp/b4m_lidar_logs/b4m_api_requests_YYYYMMDD_HHMMSS.log`
- **Contains**: Complete HTTP request/response details, timing, and decisions
- **Format**: Timestamped entries with call IDs for easy tracking

### Debug Mode
Enable verbose debug logging:
```bash
# Via parameter
ros2 run b4m_lidar b4m_lidar_navigator --ros-args -p debug_mode:=true

# Via launch script
./b4m_HA_launch.sh --b4m-lidar --simulation --debug
```

Debug mode logs:
- Navigation state transitions
- LiDAR obstacle detection details
- Control loop decisions
- Movement command publishing
- API cooldown status

### Log Files
- **API logs**: `/tmp/b4m_lidar_logs/b4m_api_requests_*.log`
- **Main logs**: `/home/mike/projects/b4m_yahboom/logs/b4m_lidar_navigator_*.log`

### Example Log Analysis
```bash
# View API requests in real-time
tail -f /tmp/b4m_lidar_logs/b4m_api_requests_*.log

# Search for API decisions
grep "DECISION:" /tmp/b4m_lidar_logs/b4m_api_requests_*.log

# View navigation state changes
grep "State transition:" /home/mike/projects/b4m_yahboom/logs/b4m_lidar_navigator_*.log
```