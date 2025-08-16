# ROS2 Humble Installation Guide - macOS

This guide provides step-by-step instructions to install ROS2 Humble on macOS systems using UTM virtual machine to support the B4M Yahboom robot project.

## Prerequisites

- macOS 10.15 (Catalina) or later
- UTM virtual machine software
- Ubuntu 22.04 LTS (Jammy Jellyfish) virtual machine
- Internet connection for downloading packages
- At least 8GB RAM and 50GB storage allocated to VM

## 1. UTM Virtual Machine Setup

### Install UTM

Download and install UTM from:
- Mac App Store (paid version with support)
- GitHub releases (free version): https://github.com/utmapp/UTM/releases

### Create Ubuntu Virtual Machine

1. **Download Ubuntu 22.04 LTS ISO**
   - Download from: https://releases.ubuntu.com/22.04.5/ubuntu-22.04.5-desktop-amd64.iso.torrent
   - Choose Ubuntu 22.04.3 LTS

2. **Create new VM in UTM**
   - Click "Create a New Virtual Machine"
   - Choose "Virtualize" (not Emulate)
   - Select "Linux"
   - Load Ubuntu ISO file
   - Allocate at least 8GB RAM and 50GB storage

3. **Configure VM settings**
   - Enable hardware acceleration
   - Set network to "Shared Network" for internet access
   - Enable USB passthrough for robot connection

4. **Install Ubuntu**
   - Boot from ISO and follow Ubuntu installation
   - Choose "Normal installation" with updates
   - Create user account with sudo privileges

## 2. Set Language Environment

In your Ubuntu VM terminal:

```bash
# Check current locale settings
locale

# Install locale packages
sudo apt update && sudo apt install locales

# Generate UTF-8 locales
sudo locale-gen en_US en_US.UTF-8

# Update system locale
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Export language variable for current session
export LANG=en_US.UTF-8

# Verify settings
locale
```

## 3. Configure Software Sources

### Enable Ubuntu Universe Repository

```bash
# Install software-properties-common
sudo apt install software-properties-common

# Add universe repository
sudo add-apt-repository universe
```

### Add ROS2 APT Repository

```bash
# Update package list and install curl
sudo apt update && sudo apt install curl -y

# Download and add ROS GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS2 repository to sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

## 4. Install ROS2 Humble

### Update Package Cache

```bash
sudo apt update
```

### Upgrade System Packages

```bash
sudo apt upgrade
```

### Install ROS2 Desktop

Install the desktop version (recommended) which includes ROS, RViz, examples, and tutorials:

```bash
sudo apt install ros-humble-desktop python3-argcomplete
```

### Install Build Tools

Install colcon build tools required for building ROS2 packages:

```bash
sudo apt install python3-colcon-common-extensions
```

### Install Robot Packages

Install additional ROS2 packages required for the robot's functionality:

```bash
sudo apt install ros-humble-imu-complementary-filter ros-humble-imu-filter-madgwick ros-humble-imu-tools ros-humble-robot-localization ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui ros-humble-xacro ros-humble-nav2-bringup ros-humble-cartographer-ros
```

### Install Gazebo Classic and Simulation Dependencies

Install Gazebo Classic for robot simulation:

```bash
# Install Gazebo Classic for robot simulation
sudo apt install gazebo-11 ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control

# Verify Gazebo Classic installation
gazebo --version
# Should show Gazebo multi-robot simulator, version 11.10.2 or later
```

**Note:** Gazebo Classic 11.10.2 is the primary simulation platform used for development, testing, and regression validation. The simulation environment provides reliable sensor data for SLAM and integrated robot spawning capabilities.

## 5. Configure Environment

### Manual Sourcing

For each terminal session, you need to source the ROS2 environment:

```bash
source /opt/ros/humble/setup.bash
```

### Automatic Sourcing (Recommended)

To automatically source ROS2 in every new terminal, add it to your `.bashrc`:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=20" >> ~/.bashrc
```

This eliminates the need to manually configure the environment each time you open a new terminal and sets the correct domain ID for the robot.

## 6. Verification

Test your ROS2 installation by running:

```bash
# Source the environment (if not added to .bashrc)
source /opt/ros/humble/setup.bash

# Run a simple test
ros2 run demo_nodes_cpp talker
```

In another terminal:

```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener
```

You should see the talker publishing messages and the listener receiving them.

## 7. Additional Requirements for B4M Yahboom Project

### Docker (for Micro-ROS Agent)

```bash
# Install Docker
sudo apt install docker.io

# Add user to docker group
sudo usermod -aG docker $USER

# Log out and log back in, then test
docker --version
```

### Python Dependencies

```bash
# Install PyQt5 for GUI applications
sudo apt install python3-pyqt5 python3-pyqt5.qtsvg

# Install Python packages
sudo apt install python3-pip python3-serial
pip3 install pillow numpy pyserial paho-mqtt
```

### Image Analysis Dependencies (for Regression Testing)

```bash
# Install OpenCV and scikit-image for screenshot comparison
sudo apt install python3-opencv python3-skimage
```

**Note:** These packages are required for the automated regression testing system that compares RViz screenshots to ensure consistent laser scan visualization and SLAM mapping functionality.

### Network Analysis Tools (for troubleshooting)

```bash
# Install network diagnostic tools
sudo apt install nmap netstat-nat tcpdump wireshark

# Install system monitoring tools
sudo apt install htop
```

## 8. Serial Device Access (UTM-Specific)

### Enable USB Passthrough in UTM

1. **Power off the VM**
2. **Edit VM settings in UTM**
   - Go to "Devices" tab
   - Add "USB" device
   - Choose "USB 3.0"
3. **Connect robot via USB to Mac**
4. **Start VM and connect USB device**
   - In UTM, go to "Devices" menu
   - Select your robot device to connect it to VM

### Verify USB Access

USB passthrough should work automatically. Devices typically appear as:
- `/dev/cu.usbserial-*`
- `/dev/cu.wch*` (CH340/CH341 chips)
- `/dev/tty.usb*`

```bash
# List available serial devices
ls -la /dev/ttyUSB* /dev/ttyACM*

# Check USB devices
lsusb

# Add user to dialout group
sudo usermod -a -G dialout $USER
```

**Note:** Log out and log back in for group changes to take effect.

## 9. Network Configuration (UTM-Specific)

### Static IP Configuration (REQUIRED)

**CRITICAL:** DHCP networks assign different IP addresses after sleep/restart, breaking robot connectivity. Static IP configuration is REQUIRED for reliable operation.

**Solution:** Configure macOS to use a static IP address within your network range.

**Step 1: Determine your network range and choose a static IP**

1. **Get your current network information:**
   ```bash
   # Get current IP, gateway, and subnet
   route -n get default
   
   # Get DNS servers
   scutil --dns | grep nameserver
   
   # Get current WiFi IP
   ifconfig en0 | grep 'inet ' | awk '{print $2}'
   ```

2. **Note down these values:**
   - Current IP address (e.g., `192.168.1.105`)
   - Router/Gateway (e.g., `192.168.1.1`)
   - Subnet mask (usually `255.255.255.0`)
   - DNS servers (e.g., `192.168.1.1` or `8.8.8.8`)

3. **Choose your static IP address** (this is the IP you'll use everywhere):
   - Pick an address in the same network range as your current IP
   - Choose something easy to remember and unlikely to be used by other devices
   - **Example:** If your current IP is `192.168.1.105`, choose `192.168.1.100`
   - **Your chosen static IP will be used for:**
     - macOS network configuration
     - UTM VM (via bridged networking)
     - Robot configuration in `config_robot.py`
     - All future connections

**📝 Write down your chosen static IP:** `192.168.1.100` (example - use your chosen IP)

**Step 2: Configure macOS with your chosen static IP**

1. **Open Network Preferences:**
   - Apple Menu → System Preferences → Network
   - Or System Settings → Network (macOS Ventura+)

2. **Select WiFi and configure:**
   - Select "Wi-Fi" from the left sidebar
   - Click "Advanced..." button
   - Go to "TCP/IP" tab

3. **Set your chosen static IP:**
   - Change "Configure IPv4" from "Using DHCP" to "Manually"
   - **IPv4 Address:** `192.168.1.100` (use YOUR chosen static IP)
   - **Subnet Mask:** `255.255.255.0` (from step 1)
   - **Router:** `192.168.1.1` (from step 1)

4. **Configure DNS:**
   - Go to "DNS" tab
   - Add DNS servers: `192.168.1.1`, `8.8.8.8`
   - Click "OK" and "Apply"

**Step 3: Verify your static IP is working**

```bash
# Verify your new static IP is active
ifconfig en0 | grep 'inet ' | awk '{print $2}'
# Should show your chosen static IP (e.g., 192.168.1.100)

# Test internet connectivity
ping -c 3 google.com
```

**✅ Static IP Configuration Complete!**

Your Mac now has a fixed IP address that will never change after sleep/restart.

**Step 4: Configure UTM for Bridged Networking**

Configure UTM so the VM shares the same network as your Mac, allowing the robot to connect directly.

1. **Configure UTM for bridged networking:**
   - Power off your Ubuntu VM
   - In UTM, edit VM settings → Network
   - Change from "Shared Network" to "Bridged Network"
   - Select your Mac's active network interface (usually en0 for WiFi)
   - Start the VM

2. **Verify VM gets IP on same network as Mac:**
   ```bash
   # In VM terminal, check VM gets an IP in same range as Mac
   ip addr show | grep 'inet ' | grep -v 127.0.0.1
   # Should show an IP like 192.168.1.x (same range as your Mac)
   ```

**Step 5: Configure Robot with Your Static IP**

Now configure the robot to connect to your chosen static IP address.

Edit the `config_robot.py` file with your network settings:

```python
# Use YOUR chosen static IP from Step 1
robot.set_wifi_config("your_wifi_name", "your_wifi_password")  
robot.set_udp_config([192, 168, 1, 100], 8090)  # YOUR static IP here
robot.set_car_type(robot.CAR_TYPE_COMPUTER)
```

**⚠️ IMPORTANT:** Use the exact same static IP you configured in macOS (from Step 1).

**Step 6: Verify Everything Works**

After configuring UTM bridged networking:

```bash
# In UTM VM, verify network configuration
ip addr show | grep 'inet ' | grep -v 127.0.0.1

# Start the Micro-ROS agent (it will be accessible on your static IP via bridged network)
docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:humble udp4 --port 8090

# In another VM terminal, verify agent is listening
netstat -tulpn | grep 8090

# Test connectivity between VM and Mac
ping 192.168.1.100  # Your Mac's static IP
```

**✅ Network Configuration Complete!**

Your setup now has:
- Mac with fixed static IP address
- UTM VM using bridged networking (same network as Mac)
- Robot configured to connect to your static IP
- Micro-ROS agent accessible on your static IP via bridged network

## 10. microROS Control Board Configuration

The config_robot.py script supports cross-platform serial port detection:

**List available ports:**
```bash
python3 config_robot.py --list-ports
```

**Specify port manually:**
```bash
python3 config_robot.py --port /dev/ttyUSB0
```

**Set via environment variable:**
```bash
export ROBOT_SERIAL_PORT=/dev/ttyUSB0
python3 config_robot.py
```

### Configuration Steps

1. Connect the robot via USB to your Mac
2. Connect the USB device to the VM through UTM
3. Briefly press the reset button on the microROS control board
4. The robot enters configuration mode within 5 seconds of booting (MCU indicator flashes every 300ms)
5. Run the configuration script:
   ```bash
   python3 config_robot.py
   ```
6. Verify the returned data matches your settings

## 11. Troubleshooting

### Common UTM Issues

**Issue: VM is slow or laggy**
- Solution: Increase allocated RAM and enable hardware acceleration
- Check: UTM VM settings → System → Hardware acceleration enabled

**Issue: USB device not appearing in VM**
- Solution: Connect USB device through UTM Devices menu
- Check: Device is powered on and recognized by macOS first

**Issue: Robot can't connect to Micro-ROS agent**
- **Cause:** Either static IP not configured, UTM bridged networking not set up, or robot configured with wrong IP
- **Solution:** Follow section 9 Network Configuration steps in order:
  1. Configure Mac with static IP
  2. Set up UTM bridged networking
  3. Configure robot with the same static IP
- **Check:** Verify robot IP matches Mac static IP exactly
- **Verify:** Confirm UTM is using bridged networking and VM gets IP in same range as Mac

**Issue: GUI applications don't work properly**
- Solution: Install additional graphics drivers in Ubuntu VM
- Command: `sudo apt install ubuntu-desktop-minimal`

### Advanced Troubleshooting: Network Traffic Analysis

If the robot still doesn't connect, check if it's actually sending connection attempts:

```bash
# Monitor network traffic for robot connection attempts
sudo tcpdump -i any -n port 8090

# Power cycle the robot while monitoring. You should see:
# - Connection attempts: IP <robot_ip>.8090 > <your_ip>.8090: UDP, length 24
# - If no packets appear: Robot isn't connecting to WiFi or has wrong IP
# - If packets appear: Robot is connecting but agent might not be running

# If you see connection packets, ensure micro-ros agent is running:
docker run --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:humble udp4 --port 8090 -v6
```

### Network Diagnostics

```bash
# Test Micro-ROS agent connectivity
docker run --rm --net=host microros/micro-ros-agent:humble udp4 --port 8090 -v6

# Check port availability
ss -tulpn | grep 8090

# Check network interface configuration
ip addr show $(ip route get 1.1.1.1 | grep -oP 'dev \K\S+')

# Test connectivity to gateway
ping -c 3 $(ip route | grep default | awk '{print $3}')

# Check network configuration
ip addr show

# Test Mac connectivity
ping <mac_ip_address>

# Monitor network traffic (if needed)
sudo tcpdump -i any -n port 8090
```

### UTM Performance Optimization

1. **Allocate more resources**
   - Increase RAM to 8GB or more
   - Enable multiple CPU cores
   - Allocate sufficient disk space

2. **Enable hardware acceleration**
   - In UTM VM settings → System
   - Enable "Hardware acceleration"
   - Use "Virtualize" instead of "Emulate"

3. **Optimize Ubuntu for VM**
   ```bash
   # Disable unnecessary services
   sudo systemctl disable bluetooth
   sudo systemctl disable cups
   
   # Install lightweight desktop (optional)
   sudo apt install xubuntu-desktop
   ```

## 12. Workspace Setup

After completing the installation, you'll need to build the workspaces and configure environment sourcing.

### Critical Workspace Requirements

**IMPORTANT:** After cloning the B4M Yahboom repository, you MUST build the workspace and configure automatic sourcing:

```bash
# Navigate to the project directory
cd ~/projects/b4m_yahboom

# Build the workspace (required before first use)
colcon build --symlink-install

# Add workspace sourcing to .bashrc for automatic setup
echo "source ~/projects/b4m_yahboom/source_workspaces.sh" >> ~/.bashrc

# Apply to current terminal (or open a new terminal)
source ~/.bashrc
```

**Automatic Sourcing:** This eliminates the need to manually source workspaces in every new terminal session.

**Common Error:** If you see `Package 'yahboomcar_bringup' not found`, it means the workspace isn't sourced. Check that the sourcing command is in your .bashrc:

```bash
# Verify workspace sourcing is in .bashrc
grep "source_workspaces.sh" ~/.bashrc

# If missing, add it manually
echo "source ~/projects/b4m_yahboom/source_workspaces.sh" >> ~/.bashrc
```

**After Code Changes:** When you modify ROS2 package code, rebuild and restart nodes:

```bash
# Rebuild after code changes
colcon build --symlink-install

# Restart any running ROS2 nodes/processes to load new code
```

See the `WORKSPACE_README.md` for detailed instructions on workspace management.

## 13. Troubleshooting Launch Issues

### Debug Tools

The project includes enhanced debugging tools to help identify launch issues:

```bash
# Debug Script: Run the bringup debug script to identify common issues
cd ~/projects/b4m_yahboom
./debug_bringup.sh

# Enhanced Launch Script: The main launch script now includes comprehensive logging
./b4m_HA_launch.sh
```

### Log Files

All launch activities are now logged for troubleshooting:

- **Main log location:** `~/projects/b4m_yahboom/logs/`
- **Step-specific logs:** Each launch step creates its own detailed log file
- **Debug logs:** The debug script creates timestamped logs

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

- **"Package not found" errors:** Usually indicates workspace not built or sourced
  ```bash
  cd ~/projects/b4m_yahboom
  colcon build --symlink-install
  source install/setup.bash
  ```

- **Missing install directory:** Run colcon build first:
  ```bash
  colcon build --symlink-install
  ```

- **Launch failures:** Check step-specific log files in `logs/` directory for detailed error messages

- **Node restart required:** After code changes, restart ROS2 nodes to load new code:
  ```bash
  # Stop running nodes, then rebuild and restart
  colcon build --symlink-install
  ```

### Verification Commands

```bash
# Test package availability
source install/setup.bash
ros2 pkg list | grep yahboomcar_bringup

# Test launch file
source install/setup.bash
ros2 launch yahboomcar_bringup yahboomcar_bringup_launch.py --show-args
```

## 14. Uninstallation (Optional)

If you need to uninstall ROS2 Humble:

```bash
# Remove ROS2 packages
sudo apt remove ~nros-humble-* && sudo apt autoremove

# Remove ROS2 repository
sudo rm /etc/apt/sources.list.d/ros2.list

# Update package cache and clean up
sudo apt update
sudo apt autoremove

# Consider upgrading packages previously shadowed
sudo apt upgrade
```

## Next Steps

1. Clone the B4M Yahboom repository
2. Follow the `WORKSPACE_README.md` for workspace setup
3. Review the `CLAUDE.md` for development guidelines
4. Execute the launch sequence as described in the project documentation

For project-specific setup and usage instructions, refer to the other documentation files in this repository.

## Additional macOS Considerations

### File Sharing Between macOS and VM

To share files between macOS and the Ubuntu VM:

1. **Enable UTM file sharing**
   - In UTM VM settings → Sharing
   - Enable "Directory Share"
   - Choose a folder on macOS to share

2. **Mount shared folder in Ubuntu**
   ```bash
   # Create mount point
   sudo mkdir /mnt/shared
   
   # Mount the shared folder
   sudo mount -t virtiofs share /mnt/shared
   
   # Auto-mount on boot (optional)
   echo "share /mnt/shared virtiofs defaults 0 0" | sudo tee -a /etc/fstab
   ```

### Backup and Snapshots

UTM supports VM snapshots for easy backup:

1. **Create snapshot before major changes**
   - In UTM, select your VM
   - Click "Save" to create snapshot
   - Name it appropriately (e.g., "ROS2_Installed")

2. **Restore from snapshot if needed**
   - Select snapshot from UTM interface
   - Click "Restore"
