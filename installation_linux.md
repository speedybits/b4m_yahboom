# ROS2 Humble Installation Checklist - Native Linux

This checklist provides step-by-step instructions to install ROS2 Humble on native Linux systems to support the B4M Yahboom robot project. Check off each step as you complete it.

## Prerequisites

- [ ] Ubuntu 22.04 LTS (Jammy Jellyfish) recommended
- [ ] Internet connection for downloading packages
- [ ] Terminal access with sudo privileges

## 1. Set Language Environment

- [ ] Check current locale settings:
  ```bash
  locale
  ```

- [ ] Install locale packages:
  ```bash
  sudo apt update && sudo apt install locales
  ```

- [ ] Generate UTF-8 locales:
  ```bash
  sudo locale-gen en_US en_US.UTF-8
  ```

- [ ] Update system locale:
  ```bash
  sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
  ```

- [ ] Export language variable for current session:
  ```bash
  export LANG=en_US.UTF-8
  ```

- [ ] Verify settings:
  ```bash
  locale
  ```

**Note:** The locale can be different, but must support UTF-8 encoding.

## 2. Configure Software Sources

### Enable Ubuntu Universe Repository

- [ ] Install software-properties-common:
  ```bash
  sudo apt install software-properties-common
  ```

- [ ] Add universe repository:
  ```bash
  sudo add-apt-repository universe
  ```

### Add ROS2 APT Repository

- [ ] Update package list and install curl:
  ```bash
  sudo apt update && sudo apt install curl -y
  ```

- [ ] Download and add ROS GPG key:
  ```bash
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
  ```

- [ ] Add ROS2 repository to sources list:
  ```bash
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
  ```

## 3. Install ROS2 Humble

### Update Package Cache

- [ ] Update package cache:
  ```bash
  sudo apt update
  ```

### Upgrade System Packages

- [ ] Upgrade system packages:
  ```bash
  sudo apt upgrade
  ```

**Important:** ROS2 packages are built on frequently updated Ubuntu systems. Ensure your system is up to date before installing new packages.

### Install ROS2 Desktop

- [ ] Install the desktop version (recommended) which includes ROS, RViz, examples, and tutorials:
  ```bash
  sudo apt install ros-humble-desktop python3-argcomplete
  ```

### Install Build Tools

- [ ] Install colcon build tools required for building ROS2 packages:
  ```bash
  sudo apt install python3-colcon-common-extensions
  ```

### Install Robot Packages

- [ ] Install additional ROS2 packages required for the robot's functionality:
  ```bash
  sudo apt install ros-humble-imu-complementary-filter ros-humble-imu-filter-madgwick ros-humble-imu-tools ros-humble-robot-localization ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui ros-humble-xacro ros-humble-nav2-bringup ros-humble-cartographer-ros
  ```

### Install Gazebo Classic and Simulation Dependencies

- [ ] Install Gazebo Classic for robot simulation:
  ```bash
  sudo apt install gazebo-11 ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control
  ```

- [ ] Verify Gazebo Classic installation:
  ```bash
  gazebo --version
  # Should show Gazebo multi-robot simulator, version 11.10.2 or later
  ```

**Note:** Gazebo Classic 11.10.2 is the primary simulation platform used for development, testing, and regression validation. The simulation environment provides reliable sensor data for SLAM and integrated robot spawning capabilities.

## 4. Configure Environment

### Manual Sourcing

- [ ] Source the ROS2 environment for current terminal session:
  ```bash
  source /opt/ros/humble/setup.bash
  ```

### Automatic Sourcing (Recommended)

- [ ] Add automatic sourcing to your `.bashrc`:
  ```bash
  echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
  echo "export ROS_DOMAIN_ID=20" >> ~/.bashrc
  ```

This eliminates the need to manually configure the environment each time you open a new terminal and sets the correct domain ID for the robot.

## 5. Verification

- [ ] Source the environment (if not added to .bashrc):
  ```bash
  source /opt/ros/humble/setup.bash
  ```

- [ ] Run a simple test in first terminal:
  ```bash
  ros2 run demo_nodes_cpp talker
  ```

- [ ] In another terminal, run the listener:
  ```bash
  source /opt/ros/humble/setup.bash
  ros2 run demo_nodes_py listener
  ```

- [ ] Verify that you see the talker publishing messages and the listener receiving them

## 6. Additional Requirements for B4M Yahboom Project

After installing ROS2 Humble, you'll need additional packages for the Yahboom robot:

### Docker (for Micro-ROS Agent)

- [ ] Install Docker:
  ```bash
  sudo apt install docker.io
  ```

- [ ] Add user to docker group:
  ```bash
  sudo usermod -aG docker $USER
  ```

- [ ] Log out and log back in, then test:
  ```bash
  docker --version
  ```

**Note**: If docker commands still show "permission denied" after logout/login, you may need to reboot your system for group membership to take effect properly.

### Python Dependencies

- [ ] Install PyQt5 for GUI applications:
  ```bash
  sudo apt install python3-pyqt5 python3-pyqt5.qtsvg
  ```

- [ ] Install Python packages:
  ```bash
  sudo apt install python3-pip python3-serial
  pip3 install pillow numpy pyserial paho-mqtt
  ```

### Image Analysis Dependencies (for Regression Testing)

- [ ] Install OpenCV and scikit-image for screenshot comparison:
  ```bash
  sudo apt install python3-opencv python3-skimage
  ```

**Note:** These packages are required for the automated regression testing system that compares RViz screenshots to ensure consistent laser scan visualization and SLAM mapping functionality.

### Network Analysis Tools (for troubleshooting)

- [ ] Install network diagnostic tools:
  ```bash
  sudo apt install nmap netstat-nat tcpdump wireshark
  ```

- [ ] Install system monitoring tools:
  ```bash
  sudo apt install htop
  ```

## 7. Serial Device Access

### USB Device Permissions

- [ ] Add user to dialout group for serial device access:
  ```bash
  sudo usermod -a -G dialout $USER
  ```

- [ ] Log out and log back in for group changes to take effect

### Verify Serial Devices

- [ ] List available serial devices:
  ```bash
  ls -la /dev/ttyUSB* /dev/ttyACM*
  ```

- [ ] Check USB devices:
  ```bash
  lsusb
  ```

## 8. Network Configuration

### Network Setup for Robot Communication

The WiFi setup wizard (recommended) handles network configuration automatically using mDNS hostname resolution, eliminating the need for manual IP configuration.

#### Use WiFi Setup Wizard (Recommended)

- [ ] The WiFi setup wizard automatically handles network configuration:
  ```bash
  # Navigate to project directory
  cd ~/projects/b4m_yahboom
  
  # Run the interactive WiFi setup wizard
  ./b4m_launch.sh --setup-wifi
  ```

The wizard will:
- Auto-detect your hostname and IP address
- Configure the robot to use mDNS hostname resolution (e.g., `hostname.local`)
- Eliminate the need for fixed IP addresses
- Handle network changes automatically

#### Verify Network Configuration

- [ ] Start the Micro-ROS agent:
  ```bash
  docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:humble udp4 --port 8090
  ```

- [ ] In another terminal, verify agent is listening:
  ```bash
  ss -tulpn | grep 8090
  ```

- [ ] Test network connectivity:
  ```bash
  ping -c 3 google.com
  ```

**✅ Network Configuration Complete!**

Your setup now has:
- Automatic network configuration via mDNS hostname resolution
- Robot configured to connect using hostname instead of fixed IP
- Automatic handling of network changes without manual configuration

## 9. Repository Setup and Workspace Build

Before configuring the robot, you need to clone the B4M Yahboom repository and build the workspace.

### Clone the Repository

- [ ] Create projects directory and clone the repository:
  ```bash
  # Create projects directory
  mkdir -p ~/projects
  
  # Clone the B4M Yahboom repository
  cd ~/projects
  git clone https://github.com/mikebvansickle/b4m_yahboom.git
  
  # Navigate to the project directory
  cd b4m_yahboom
  ```

### Build the Workspace

**IMPORTANT:** The workspace must be built before using any of the launch scripts or WiFi setup wizard.

- [ ] Build the workspace (required before first use):
  ```bash
  colcon build --symlink-install
  ```

### Configure Environment Sourcing

- [ ] Add workspace sourcing to .bashrc for automatic setup:
  ```bash
  echo "source ~/projects/b4m_yahboom/source_workspaces.sh" >> ~/.bashrc
  ```

- [ ] Apply to current terminal (or open a new terminal):
  ```bash
  source ~/.bashrc
  ```

**Automatic Sourcing:** This eliminates the need to manually source workspaces in every new terminal session.

### Verify Setup

- [ ] Verify the workspace was built successfully:
  ```bash
  ls install/
  ```

- [ ] Test that ROS2 packages are available:
  ```bash
  source install/setup.bash
  ros2 pkg list | grep yahboomcar_bringup
  ```

- [ ] If the last command returns `yahboomcar_bringup`, your workspace is properly set up.

## 10. microROS Control Board Configuration

The config_robot.py script supports cross-platform serial port detection:

- [ ] List available ports:
  ```bash
  python3 config_robot.py --list-ports
  ```

- [ ] Specify port manually (if needed):
  ```bash
  python3 config_robot.py --port /dev/ttyUSB0
  ```

- [ ] Set via environment variable (alternative method):
  ```bash
  export ROBOT_SERIAL_PORT=/dev/ttyUSB0
  python3 config_robot.py
  ```

### Configuration Steps

**Option 1: Interactive WiFi Setup Wizard (Recommended)**

- [ ] Use the new interactive WiFi setup wizard for guided configuration:
  ```bash
  # Navigate to project directory
  cd ~/projects/b4m_yahboom
  
  # Run the interactive WiFi setup wizard
  ./b4m_launch.sh --setup-wifi
  ```

The wizard will guide you through:
1. Prerequisites checking (Python, serial permissions, files)
2. USB connection detection and selection
3. WiFi network configuration (SSID and password)
4. Agent connection method selection (mDNS hostname vs fixed IP)
5. Configuration summary and confirmation
6. Automatic robot configuration

**Option 2: Manual Configuration**

- [ ] Connect the robot via USB
- [ ] Briefly press the reset button on the microROS control board
- [ ] Wait for configuration mode (robot enters configuration mode within 5 seconds of booting - MCU indicator flashes every 300ms)
- [ ] Run the configuration script:
  ```bash
  python3 config_robot.py
  ```
- [ ] Verify the returned data matches your settings

**⚠️ IMPORTANT:** The robot configuration script automatically applies changes AND shows current settings. The output will display what's stored in the robot's memory including WiFi SSID, IP address, and other parameters.

### Verify Robot Configuration (Recommended)

If you experience connection issues, verify the robot's stored configuration:

- [ ] Connect robot via USB and check stored settings:
  ```bash
  python3 config_robot.py --port /dev/ttyUSB0
  ```

- [ ] Verify the output shows:
  - **WiFi SSID**: Your network name
  - **IP Address**: Your Linux machine's static IP
  - **Port**: 8090
  - **Car Type**: CAR_TYPE_COMPUTER

- [ ] If settings are incorrect, the script will have updated them - power cycle the robot to apply changes

## 11. Troubleshooting

### Common Issues

**Issue: Permission denied for serial device**
- [ ] Solution: Add user to dialout group and log out/in
  ```bash
  sudo usermod -a -G dialout $USER
  ```

**Issue: Docker permission denied**
- [ ] Solution: Add user to docker group and log out/in
  ```bash
  sudo usermod -aG docker $USER
  ```

**Issue: "No such file or directory" - /dev/ttyUSB0 not found**
- **Cause:** USB device disconnected or not recognized by system
- **Solution:** Check USB connection and device permissions
- [ ] Run diagnostic commands:
  ```bash
  # List USB devices
  lsusb
  
  # Check for serial devices
  ls -la /dev/ttyUSB* /dev/ttyACM*
  
  # Check system messages for USB events
  dmesg | grep -i usb
  ```

**Issue: Robot not connecting to Micro-ROS agent**
- **Cause:** Network configuration issues or robot WiFi settings
- **Solution:** Use the WiFi setup wizard to configure robot network settings:
  - [ ] Run `./b4m_launch.sh --setup-wifi` to configure robot with mDNS hostname resolution
  - [ ] Verify robot and computer are on the same network
- [ ] **Check:** Verify WiFi setup wizard completed successfully
- [ ] **Verify:** Confirm robot is connecting to correct WiFi network

### Advanced Troubleshooting: Network Traffic Analysis

If the robot still doesn't connect, check if it's actually sending connection attempts:

- [ ] Monitor network traffic for robot connection attempts:
  ```bash
  sudo tcpdump -i any -n port 8090
  ```

- [ ] Power cycle the robot while monitoring. You should see:
  - **Connection attempts**: `IP <robot_ip>.8090 > <your_hostname_ip>.8090: UDP, length 24`
  - **If no packets appear**: Robot isn't connecting to WiFi or network configuration issue
  - **If packets appear**: Robot is connecting but agent might not be running

- [ ] If you see connection packets, ensure micro-ros agent is running:
  ```bash
  docker run --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:humble udp4 --port 8090 -v6
  ```

### Network Diagnostics

- [ ] Test Micro-ROS agent connectivity:
  ```bash
  docker run --rm --net=host microros/micro-ros-agent:humble udp4 --port 8090 -v6
  ```

- [ ] Check port availability:
  ```bash
  ss -tulpn | grep 8090
  ```

- [ ] Check network interface configuration:
  ```bash
  ip addr show $(ip route get 1.1.1.1 | grep -oP 'dev \K\S+')
  ```

- [ ] Test connectivity to gateway:
  ```bash
  ping -c 3 $(ip route | grep default | awk '{print $3}')
  ```

- [ ] Monitor network traffic (if needed):
  ```bash
  sudo tcpdump -i any -n port 8090
  ```

## 12. Troubleshooting Launch Issues

### Debug Tools

The project includes enhanced debugging tools to help identify launch issues:

- [ ] **Debug Script:** Run the bringup debug script to identify common issues:
  ```bash
  cd ~/projects/b4m_yahboom
  ./debug_bringup.sh
  ```

- [ ] **Enhanced Launch Script:** The main launch script now includes comprehensive logging:
  ```bash
  ./b4m_HA_launch.sh
  ```

### Log Files

All launch activities are now logged for troubleshooting:

- [ ] **Main log location:** `~/projects/b4m_yahboom/logs/`
- [ ] **Step-specific logs:** Each launch step creates its own detailed log file
- [ ] **Debug logs:** The debug script creates timestamped logs

**Example log files:**
```
logs/
├── b4m_launch_YYYYMMDD_HHMMSS.log           # Main launch log
├── step1_starting_the_micro-ros_agent_*.log  # MicroROS agent log
├── step3_starting_the_car's_underlying_*.log # Bringup log
├── bringup_debug_YYYYMMDD_HHMMSS.log        # Debug script output
└── ...other step logs
```

### Common Issues and Solutions

- [ ] **"Package not found" errors:** Usually indicates workspace not built or sourced
  ```bash
  cd ~/projects/b4m_yahboom
  colcon build --symlink-install
  source install/setup.bash
  ```

- [ ] **Missing install directory:** Run colcon build first:
  ```bash
  colcon build --symlink-install
  ```

- [ ] **Launch failures:** Check step-specific log files in `logs/` directory for detailed error messages

- [ ] **Node restart required:** After code changes, restart ROS2 nodes to load new code:
  ```bash
  # Stop running nodes, then rebuild and restart
  colcon build --symlink-install
  ```

### Verification Commands

- [ ] **Test package availability:**
  ```bash
  source install/setup.bash
  ros2 pkg list | grep yahboomcar_bringup
  ```

- [ ] **Test launch file:**
  ```bash
  source install/setup.bash
  ros2 launch yahboomcar_bringup yahboomcar_bringup_launch.py --show-args
  ```

## 13. Uninstallation (Optional)

If you need to uninstall ROS2 Humble:

- [ ] Remove ROS2 packages:
  ```bash
  sudo apt remove ~nros-humble-* && sudo apt autoremove
  ```

- [ ] Remove ROS2 repository:
  ```bash
  sudo rm /etc/apt/sources.list.d/ros2.list
  ```

- [ ] Update package cache and clean up:
  ```bash
  sudo apt update
  sudo apt autoremove
  ```

- [ ] Consider upgrading packages previously shadowed:
  ```bash
  sudo apt upgrade
  ```

## Next Steps

- [ ] ✅ **Repository Setup Complete** - You've already cloned and built the B4M Yahboom repository in section 9
- [ ] Review the `WORKSPACE_README.md` for advanced workspace management
- [ ] Review the `CLAUDE.md` for development guidelines
- [ ] **Configure robot WiFi using `./b4m_launch.sh --setup-wifi`** (recommended for first-time setup)
- [ ] Execute the launch sequence as described in the project documentation

For project-specific setup and usage instructions, refer to the other documentation files in this repository.

**Quick Start Tip:** Now that everything is installed and built, the easiest way to configure your robot for the first time is to run `./b4m_launch.sh --setup-wifi` which provides an interactive wizard to guide you through WiFi and network configuration.
