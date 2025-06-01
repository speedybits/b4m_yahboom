#!/bin/bash

# This script tracks exit codes of Gazebo and related processes
# to debug the exit code 143 issue

# Create log directory and file
LOG_DIR="/home/yahboom/b4m_yahboom/logs"
LOG_FILE="${LOG_DIR}/exit_code_track_$(date +%Y%m%d_%H%M%S).log"
mkdir -p "${LOG_DIR}" 2>/dev/null
touch "${LOG_FILE}"

# Function to log messages
log_message() {
  local message="$1"
  local timestamp=$(date +"%Y-%m-%d %H:%M:%S")
  echo "[${timestamp}] ${message}" | tee -a "${LOG_FILE}"
}

# Clean up existing processes
log_message "Cleaning up existing processes..."
pkill gazebo || true
pkill gzserver || true
pkill gzclient || true
pkill rviz2 || true
sleep 2

# Source ROS workspace
log_message "Sourcing ROS workspace..."
source /opt/ros/humble/setup.bash
source /home/yahboom/b4m_yahboom/install/setup.bash 2>/dev/null || true

# Export Gazebo model path
log_message "Exporting Gazebo model path..."
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/yahboom/b4m_yahboom/yahboomcar_description/models

# Check system resources before starting
log_message "Memory usage before starting:"
free -h >> "${LOG_FILE}"
log_message "CPU load before starting:"
uptime >> "${LOG_FILE}"

# Start Gazebo with trapping exit code
log_message "Starting Gazebo with exit code trapping..."

# Create a wrapper script to capture the exit code
cat > /tmp/gazebo_wrapper.sh << 'EOF'
#!/bin/bash
LOG_FILE="/home/yahboom/b4m_yahboom/logs/exit_code_track_gazebo.log"
echo "[$(date +"%Y-%m-%d %H:%M:%S")] Starting Gazebo with PID $$" > "$LOG_FILE"
echo "[$(date +"%Y-%m-%d %H:%M:%S")] Command: gazebo --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so" >> "$LOG_FILE"

# Function to handle signals
handle_signal() {
  echo "[$(date +"%Y-%m-%d %H:%M:%S")] Received signal $1 (PID: $$)" >> "$LOG_FILE"
}

# Trap signals
trap 'handle_signal SIGHUP' HUP
trap 'handle_signal SIGINT' INT
trap 'handle_signal SIGQUIT' QUIT
trap 'handle_signal SIGTERM' TERM
trap 'handle_signal SIGKILL' KILL
trap 'handle_signal EXIT' EXIT

# Start Gazebo
gazebo --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so

# Capture exit code
EXIT_CODE=$?
echo "[$(date +"%Y-%m-%d %H:%M:%S")] Gazebo exited with code $EXIT_CODE (PID: $$)" >> "$LOG_FILE"

# Analyze exit code
case $EXIT_CODE in
  0)
    echo "[$(date +"%Y-%m-%d %H:%M:%S")] Gazebo exited normally" >> "$LOG_FILE"
    ;;
  130)
    echo "[$(date +"%Y-%m-%d %H:%M:%S")] Gazebo was terminated by SIGINT (Ctrl+C)" >> "$LOG_FILE"
    ;;
  137)
    echo "[$(date +"%Y-%m-%d %H:%M:%S")] Gazebo was killed by SIGKILL (kill -9) - likely due to out of memory" >> "$LOG_FILE"
    echo "[$(date +"%Y-%m-%d %H:%M:%S")] Memory at exit time:" >> "$LOG_FILE"
    free -h >> "$LOG_FILE"
    ;;
  143)
    echo "[$(date +"%Y-%m-%d %H:%M:%S")] Gazebo was terminated by SIGTERM (kill) - likely by another process" >> "$LOG_FILE"
    echo "[$(date +"%Y-%m-%d %H:%M:%S")] Processes that might have sent SIGTERM:" >> "$LOG_FILE"
    ps aux | grep -E 'systemd|upstart|init|oom|cgroup' >> "$LOG_FILE"
    ;;
  *)
    echo "[$(date +"%Y-%m-%d %H:%M:%S")] Gazebo exited with unexpected code $EXIT_CODE" >> "$LOG_FILE"
    ;;
esac
EOF

chmod +x /tmp/gazebo_wrapper.sh

# Run the wrapper in a new terminal
gnome-terminal --title="Gazebo with Exit Code Tracking" -- bash -c "/tmp/gazebo_wrapper.sh; exec bash"
log_message "Started Gazebo in a new terminal with exit code tracking"

# Wait a bit for Gazebo to start
sleep 5

# Check if Gazebo is running
if pgrep -f gazebo >/dev/null; then
  log_message "Gazebo is running"
else
  log_message "Gazebo failed to start or exited immediately"
  exit 1
fi

# Start RViz with exit code tracking
log_message "Starting RViz with exit code tracking..."

# Create a wrapper script for RViz
cat > /tmp/rviz_wrapper.sh << 'EOF'
#!/bin/bash
LOG_FILE="/home/yahboom/b4m_yahboom/logs/exit_code_track_rviz.log"
echo "[$(date +"%Y-%m-%d %H:%M:%S")] Starting RViz with PID $$" > "$LOG_FILE"

# Function to handle signals
handle_signal() {
  echo "[$(date +"%Y-%m-%d %H:%M:%S")] Received signal $1 (PID: $$)" >> "$LOG_FILE"
}

# Trap signals
trap 'handle_signal SIGHUP' HUP
trap 'handle_signal SIGINT' INT
trap 'handle_signal SIGQUIT' QUIT
trap 'handle_signal SIGTERM' TERM
trap 'handle_signal SIGKILL' KILL
trap 'handle_signal EXIT' EXIT

# Start RViz
RVIZ_CONFIG="/opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz"
if [ -f "$RVIZ_CONFIG" ]; then
  echo "[$(date +"%Y-%m-%d %H:%M:%S")] Starting RViz with config: $RVIZ_CONFIG" >> "$LOG_FILE"
  rviz2 -d "$RVIZ_CONFIG"
else
  echo "[$(date +"%Y-%m-%d %H:%M:%S")] Config not found, starting RViz without config" >> "$LOG_FILE"
  rviz2
fi

# Capture exit code
EXIT_CODE=$?
echo "[$(date +"%Y-%m-%d %H:%M:%S")] RViz exited with code $EXIT_CODE (PID: $$)" >> "$LOG_FILE"

# Analyze exit code
case $EXIT_CODE in
  0)
    echo "[$(date +"%Y-%m-%d %H:%M:%S")] RViz exited normally" >> "$LOG_FILE"
    ;;
  130)
    echo "[$(date +"%Y-%m-%d %H:%M:%S")] RViz was terminated by SIGINT (Ctrl+C)" >> "$LOG_FILE"
    ;;
  137)
    echo "[$(date +"%Y-%m-%d %H:%M:%S")] RViz was killed by SIGKILL (kill -9) - likely due to out of memory" >> "$LOG_FILE"
    echo "[$(date +"%Y-%m-%d %H:%M:%S")] Memory at exit time:" >> "$LOG_FILE"
    free -h >> "$LOG_FILE"
    ;;
  143)
    echo "[$(date +"%Y-%m-%d %H:%M:%S")] RViz was terminated by SIGTERM (kill) - likely by another process" >> "$LOG_FILE"
    echo "[$(date +"%Y-%m-%d %H:%M:%S")] Processes that might have sent SIGTERM:" >> "$LOG_FILE"
    ps aux | grep -E 'systemd|upstart|init|oom|cgroup' >> "$LOG_FILE"
    ;;
  *)
    echo "[$(date +"%Y-%m-%d %H:%M:%S")] RViz exited with unexpected code $EXIT_CODE" >> "$LOG_FILE"
    ;;
esac
EOF

chmod +x /tmp/rviz_wrapper.sh

# Run the RViz wrapper in a new terminal
gnome-terminal --title="RViz with Exit Code Tracking" -- bash -c "source /opt/ros/humble/setup.bash && source /home/yahboom/b4m_yahboom/install/setup.bash 2>/dev/null && /tmp/rviz_wrapper.sh; exec bash"
log_message "Started RViz in a new terminal with exit code tracking"

# Wait a bit for RViz to start
sleep 5

# Check if RViz is running
if pgrep -f rviz2 >/dev/null; then
  log_message "RViz is running"
else
  log_message "RViz failed to start or exited immediately"
fi

# Check system resources after starting components
log_message "Memory usage after starting components:"
free -h >> "${LOG_FILE}"
log_message "CPU load after starting components:"
uptime >> "${LOG_FILE}"

# Display instructions
echo "======================================================================"
echo "Exit code tracking is now running for Gazebo and RViz"
echo "======================================================================"
echo "Log files:"
echo "- Main log: ${LOG_FILE}"
echo "- Gazebo exit code log: /home/yahboom/b4m_yahboom/logs/exit_code_track_gazebo.log"
echo "- RViz exit code log: /home/yahboom/b4m_yahboom/logs/exit_code_track_rviz.log"
echo ""
echo "Let these processes run until they exit or you can manually terminate them."
echo "Then check the log files to see the exit codes and signals received."
echo "======================================================================"

# Keep the script running to monitor
log_message "Monitoring for process exits..."
while true; do
  # Check if Gazebo is still running
  if ! pgrep -f gazebo >/dev/null; then
    log_message "Gazebo has exited. Check the log file for details."
    echo "Gazebo has exited. Check /home/yahboom/b4m_yahboom/logs/exit_code_track_gazebo.log for details."
  fi
  
  # Check if RViz is still running
  if ! pgrep -f rviz2 >/dev/null; then
    log_message "RViz has exited. Check the log file for details."
    echo "RViz has exited. Check /home/yahboom/b4m_yahboom/logs/exit_code_track_rviz.log for details."
  fi
  
  # If both have exited, we can stop monitoring
  if ! pgrep -f gazebo >/dev/null && ! pgrep -f rviz2 >/dev/null; then
    log_message "Both Gazebo and RViz have exited. Ending monitoring."
    echo "Both Gazebo and RViz have exited. Check the log files for details."
    break
  fi
  
  sleep 5
done

log_message "Exit code tracking completed"
echo "Exit code tracking completed. Check the log files for details."
