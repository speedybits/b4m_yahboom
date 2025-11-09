# B4M Yahboom Robot Information

## Robot Specifications

The B4M Yahboom robot is a differential drive robot equipped with:
- LIDAR sensor for mapping and navigation
- ESP32 microcontroller for hardware control
- Micro-ROS agent for ROS2 communication
- IMU (Inertial Measurement Unit) for pose estimation
- Camera for computer vision tasks

## Navigation Capabilities

The robot uses the Nav2 navigation stack with:
- AMCL (Adaptive Monte Carlo Localization) for position tracking
- Gmapping or Cartographer for SLAM mapping
- DWB local planner for obstacle avoidance
- Waypoint navigation via MQTT integration with Home Assistant

## Launch Sequences

To start the robot system:
1. Start Micro-ROS agent
2. Power on physical robot
3. Launch robot bringup (sensor integration)
4. Start RViz visualization
5. Launch navigation system
6. Initialize robot pose
7. Start waypoint navigation with MQTT

## ROSIE AI Assistant

ROSIE is the voice-controlled conversational AI that runs on the robot. Key features:
- Local Ollama LLM for fast responses (under 1 second)
- Whisper for speech-to-text recognition
- Piper for text-to-speech output
- Wake word activation using "Rosie"
- Dual-mode temperature for factual vs conversational responses

## Development

The robot code is managed in a ROS2 workspace with multiple packages:
- yahboomcar_bringup: Core robot launch files
- yahboomcar_nav: Navigation stack
- b4m_waypoint_nav: MQTT waypoint navigation
- yahboom_esp32_camera: Camera integration
