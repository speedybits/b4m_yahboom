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

### Fixed IP Address Configuration (REQUIRED)

**CRITICAL:** DHCP networks assign different IP addresses after restart/hibernation, breaking robot connectivity. A fixed IP address is REQUIRED for reliable operation.

**Solution:** Configure your router to assign a fixed IP address to your Linux machine using DHCP reservation (recommended) or use NetworkManager to request a specific IP.

#### Step 1: Get your network information and choose a fixed IP

- [ ] Get your current network information:
  ```bash
  # Find current IP and network info
  ip route get 1.1.1.1 | grep -oP 'src \K\S+'  # Current IP
  ip route | grep default | awk '{print $3}'    # Gateway
  
  # Get your MAC address for DHCP reservation
  ip link show $(ip route get 1.1.1.1 | grep -oP 'dev \K\S+') | grep ether
  ```

- [ ] Note down these values:
  - Current IP address (e.g., `192.168.68.105`)
  - Gateway/Router (e.g., `192.168.68.1`)
  - MAC address (e.g., `28:d0:43:f7:74:ac`)
  - Network interface name (e.g., `wlo1` for WiFi)

- [ ] Choose your fixed IP address (this is the IP you'll use everywhere):
  - Pick an address in the same network range as your current IP
  - Choose something easy to remember and unlikely to be used by other devices
  - **Example:** If your current IP is `192.168.68.105`, choose `192.168.68.100`
  - **Your chosen fixed IP will be used for:**
    - Router DHCP reservation
    - Robot configuration in `config_robot.py`
    - All future connections

- [ ] **📝 Write down your chosen fixed IP:** `192.168.68.100` (example - use your chosen IP in YOUR network range)

#### Step 2: Configure fixed IP address

**Method A: DHCP Reservation (Recommended - Router Configuration):**

- [ ] Access your router's admin interface:
  - Open web browser and go to your gateway IP (e.g., `http://192.168.68.1`)
  - Login with admin credentials (often on router label)

- [ ] Find DHCP Reservation settings:
  - Look for "DHCP Reservations", "Static DHCP", "Address Reservation", or "Reserved IPs"
  - Usually under "Network", "LAN", or "DHCP" sections

- [ ] Add DHCP reservation:
  - **MAC Address:** Your MAC address from Step 1 (e.g., `28:d0:43:f7:74:ac`)
  - **IP Address:** Your chosen fixed IP (e.g., `192.168.68.100`)
  - **Device Name:** "Linux-Robot-PC" (optional description)
  - Save/Apply settings

- [ ] Restart router's DHCP service (or reboot router if needed)

- [ ] Restart your network connection:
  ```bash
  sudo nmcli con down $(nmcli -t -f NAME,DEVICE con show --active | grep $(ip route get 1.1.1.1 | grep -oP 'dev \K\S+') | cut -d: -f1)
  sudo nmcli con up $(nmcli -t -f NAME,DEVICE con show --active | grep $(ip route get 1.1.1.1 | grep -oP 'dev \K\S+') | cut -d: -f1)
  ```

**Method B: NetworkManager DHCP with Preferred IP (Fallback):**

If router configuration isn't possible, request specific IP through DHCP:

- [ ] Configure NetworkManager to request your chosen IP:
  ```bash
  # Replace 'connection_name' with your actual connection name
  CONNECTION_NAME=$(nmcli -t -f NAME,DEVICE con show --active | grep $(ip route get 1.1.1.1 | grep -oP 'dev \K\S+') | cut -d: -f1)
  
  # Request specific IP via DHCP
  sudo nmcli con modify "$CONNECTION_NAME" ipv4.dhcp-client-id "192.168.68.100"
  sudo nmcli con down "$CONNECTION_NAME" && sudo nmcli con up "$CONNECTION_NAME"
  ```

**Note:** Method B requests the IP but doesn't guarantee it. Method A (router configuration) is more reliable.

#### Step 3: Verify your fixed IP is working

- [ ] Verify your new fixed IP is active:
  ```bash
  ip addr show $(ip route get 1.1.1.1 | grep -oP 'dev \K\S+') | grep 'inet '
  # Should show your chosen fixed IP (e.g., 192.168.68.100)
  ```

- [ ] Test internet connectivity:
  ```bash
  ping -c 3 google.com
  ```

- [ ] Verify gateway connectivity:
  ```bash
  ping -c 3 $(ip route | grep default | awk '{print $3}')  # Your gateway IP
  ```

- [ ] Check DNS is working automatically:
  ```bash
  nslookup google.com
  ```

**✅ Fixed IP Configuration Complete!**

Your Linux machine now has a fixed IP address with automatic DNS and network settings.

#### Step 4: Configure Robot with Your Fixed IP

- [ ] Edit the `config_robot.py` file with your network settings:
  ```python
  # Use YOUR chosen fixed IP from Step 1
  robot.set_wifi_config("your_wifi_name", "your_wifi_password")  
  robot.set_udp_config([192, 168, 68, 100], 8090)  # YOUR fixed IP here (match your network)
  robot.set_car_type(robot.CAR_TYPE_COMPUTER)
  ```

**⚠️ IMPORTANT:** Use the exact same fixed IP you configured via DHCP reservation (from Step 1).

#### Step 5: Verify Everything Works

- [ ] Start the Micro-ROS agent (it will be accessible on your fixed IP):
  ```bash
  docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:humble udp4 --port 8090
  ```

- [ ] In another terminal, verify agent is listening:
  ```bash
  ss -tulpn | grep 8090
  ```

- [ ] Check network connectivity:
  ```bash
  ping -c 3 192.168.68.100  # Your fixed IP (use YOUR chosen IP)
  ```

**✅ Network Configuration Complete!**

Your setup now has:
- Linux with fixed IP address via DHCP reservation
- Automatic DNS and network configuration
- Robot configured to connect to your fixed IP
- Micro-ROS agent accessible on your fixed IP

## 9. microROS Control Board Configuration

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

## 10. Troubleshooting

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
- **Cause:** Either fixed IP not configured or robot configured with wrong IP
- **Solution:** Follow section 8 Network Configuration steps in order:
  - [ ] Configure DHCP reservation on router OR use NetworkManager to request specific IP
  - [ ] Configure robot with the same fixed IP
- [ ] **Check:** Verify robot IP matches Linux fixed IP exactly
- [ ] **Verify:** Confirm fixed IP is active with `ip addr show`

**Issue: DHCP reservation not working**
- **Cause:** Router not assigning reserved IP or DHCP cache issues
- **Solution:** Troubleshoot DHCP reservation:
  - [ ] Verify reservation is saved in router: Check router admin interface
  - [ ] Clear DHCP lease: `sudo nmcli con down <connection> && sudo nmcli con up <connection>`
  - [ ] Restart router if needed to clear DHCP lease table
  - [ ] Verify MAC address matches exactly in router settings
- [ ] **Fallback:** Use NetworkManager method from section 8 if router method fails

**Issue: Intermittent internet connectivity (less common with DHCP reservation)**
- **Cause:** DHCP providing different DNS servers or network configuration issues  
- **Solution:** DHCP reservation should automatically provide correct DNS:
  - [ ] Check current DNS: `nslookup google.com`
  - [ ] If DNS fails, verify router's DNS settings
  - [ ] Router should automatically provide correct gateway IP as DNS
- [ ] **Test:** Verify connectivity: `ping -c 3 google.com`

### Advanced Troubleshooting: Network Traffic Analysis

If the robot still doesn't connect, check if it's actually sending connection attempts:

- [ ] Monitor network traffic for robot connection attempts:
  ```bash
  sudo tcpdump -i any -n port 8090
  ```

- [ ] Power cycle the robot while monitoring. You should see:
  - **Connection attempts**: `IP <robot_ip>.8090 > <your_ip>.8090: UDP, length 24`
  - **If no packets appear**: Robot isn't connecting to WiFi or has wrong IP
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

## 11. Workspace Setup

After completing the installation, you'll need to build the workspaces and configure environment sourcing.

### Critical Workspace Requirements

**IMPORTANT:** After cloning the B4M Yahboom repository, you MUST build the workspace and configure automatic sourcing:

- [ ] Navigate to the project directory:
  ```bash
  cd ~/projects/b4m_yahboom
  ```

- [ ] Build the workspace (required before first use):
  ```bash
  colcon build --symlink-install
  ```

- [ ] Add workspace sourcing to .bashrc for automatic setup:
  ```bash
  echo "source ~/projects/b4m_yahboom/source_workspaces.sh" >> ~/.bashrc
  ```

- [ ] Apply to current terminal (or open a new terminal):
  ```bash
  source ~/.bashrc
  ```

**Automatic Sourcing:** This eliminates the need to manually source workspaces in every new terminal session.

**Common Error:** If you see `Package 'yahboomcar_bringup' not found`, it means the workspace isn't sourced. Check that the sourcing command is in your .bashrc:

- [ ] Verify workspace sourcing is in .bashrc:
  ```bash
  grep "source_workspaces.sh" ~/.bashrc
  ```

- [ ] If missing, add it manually:
  ```bash
  echo "source ~/projects/b4m_yahboom/source_workspaces.sh" >> ~/.bashrc
  ```

**After Code Changes:** When you modify ROS2 package code, rebuild and restart nodes:

- [ ] Rebuild after code changes:
  ```bash
  colcon build --symlink-install
  ```

- [ ] Restart any running ROS2 nodes/processes to load new code

See the `WORKSPACE_README.md` for detailed instructions on workspace management.

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

- [ ] Clone the B4M Yahboom repository
- [ ] Follow the `WORKSPACE_README.md` for workspace setup
- [ ] Review the `CLAUDE.md` for development guidelines
- [ ] Execute the launch sequence as described in the project documentation

For project-specific setup and usage instructions, refer to the other documentation files in this repository.
