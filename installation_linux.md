# ROS2 Humble Installation Guide - Native Linux

This guide provides step-by-step instructions to install ROS2 Humble on native Linux systems to support the B4M Yahboom robot project.

## Prerequisites

- Ubuntu 22.04 LTS (Jammy Jellyfish) recommended
- Internet connection for downloading packages
- Terminal access with sudo privileges

## 1. Set Language Environment

First, ensure your system supports UTF-8 encoding:

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

**Note:** The locale can be different, but must support UTF-8 encoding.

## 2. Configure Software Sources

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

## 3. Install ROS2 Humble

### Update Package Cache

```bash
sudo apt update
```

### Upgrade System Packages

```bash
sudo apt upgrade
```

**Important:** ROS2 packages are built on frequently updated Ubuntu systems. Ensure your system is up to date before installing new packages.

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
sudo apt install ros-humble-imu-complementary-filter ros-humble-imu-filter-madgwick ros-humble-imu-tools ros-humble-robot-localization ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui ros-humble-xacro
```

## 4. Configure Environment

### Manual Sourcing

For each terminal session, you need to source the ROS2 environment:

```bash
source /opt/ros/humble/setup.bash
```

### Automatic Sourcing (Recommended)

To automatically source ROS2 in every new terminal, add it to your `.bashrc`:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

This eliminates the need to manually configure the environment each time you open a new terminal.

## 5. Verification

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

## 6. Additional Requirements for B4M Yahboom Project

After installing ROS2 Humble, you'll need additional packages for the Yahboom robot:

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
sudo apt install python3-pip
pip3 install pillow numpy
```

### Network Analysis Tools (for troubleshooting)

```bash
# Install network diagnostic tools
sudo apt install nmap netstat-nat tcpdump wireshark

# Install system monitoring tools
sudo apt install htop
```

## 7. Serial Device Access

### USB Device Permissions

Devices should appear automatically as `/dev/ttyUSB0` or `/dev/ttyACM0`. Add user to dialout group:

```bash
sudo usermod -a -G dialout $USER
```

**Note:** Log out and log back in for group changes to take effect.

### Verify Serial Devices

```bash
# List available serial devices
ls -la /dev/ttyUSB* /dev/ttyACM*

# Check USB devices
lsusb
```

## 8. Network Configuration

### Static IP Configuration (REQUIRED)

**CRITICAL:** DHCP networks assign different IP addresses after restart/hibernation, breaking robot connectivity. Static IP configuration is REQUIRED for reliable operation.

**Solution:** Configure Linux to use a static IP address within your network range.

**Step 1: Determine your network range and choose a static IP**

1. **Get your current network information:**
   ```bash
   # Find current IP and network info
   ip route get 1.1.1.1 | grep -oP 'src \K\S+'  # Current IP
   ip route | grep default | awk '{print $3}'    # Gateway
   
   # Check current network interface (WiFi or Ethernet)
   ip route get 1.1.1.1 | grep -oP 'dev \K\S+'  # Network interface
   
   # Get detailed network info
   ip addr show $(ip route get 1.1.1.1 | grep -oP 'dev \K\S+')
   ```

2. **Note down these values:**
   - Current IP address (e.g., `192.168.1.150`)
   - Gateway/Router (e.g., `192.168.1.1`)
   - Network interface (e.g., `wlan0` for WiFi, `eth0` for Ethernet)
   - Subnet (usually `/24` or `255.255.255.0`)

3. **Choose your static IP address** (this is the IP you'll use everywhere):
   - Pick an address in the same network range as your current IP
   - Choose something easy to remember and unlikely to be used by other devices
   - **Example:** If your current IP is `192.168.1.150`, choose `192.168.1.100`
   - **Your chosen static IP will be used for:**
     - Linux network configuration
     - Robot configuration in `config_robot.py`
     - All future connections

**📝 Write down your chosen static IP:** `192.168.1.100` (example - use your chosen IP)

**Step 2: Configure Linux with your chosen static IP**

**Method A: Using NetworkManager (Ubuntu Desktop with GUI):**

1. **Open Network Settings:**
   - Click on network icon in system tray
   - Select "Settings" or "Network Settings"
   - Or open Settings → Network

2. **Configure WiFi/Ethernet:**
   - Click the gear icon next to your connection
   - Go to "IPv4" tab
   - Change from "Automatic (DHCP)" to "Manual"

3. **Set your chosen static IP:**
   - **Address:** `192.168.1.100` (use YOUR chosen static IP)
   - **Netmask:** `255.255.255.0`
   - **Gateway:** `192.168.1.1` (from step 1)
   - **DNS:** `192.168.1.1, 8.8.8.8`
   - Click "Apply"

**Method B: Using Netplan (Ubuntu Server or if GUI method doesn't work):**

1. **Edit netplan configuration:**
   ```bash
   # Find your netplan config file
   sudo find /etc/netplan -name "*.yaml"
   
   # Edit the configuration (replace filename with your actual file)
   sudo nano /etc/netplan/00-installer-config.yaml
   ```

2. **Update configuration** (replace `wlan0` with your interface from Step 1):
   ```yaml
   network:
     version: 2
     wifis:
       wlan0:  # Use your actual interface name
         dhcp4: no
         addresses:
           - 192.168.1.100/24  # YOUR chosen static IP
         gateway4: 192.168.1.1  # Your gateway
         nameservers:
           addresses: [192.168.1.1, 8.8.8.8]
         access-points:
           "your_wifi_name":  # Your WiFi network name
             password: "your_wifi_password"
   ```

3. **Apply the configuration:**
   ```bash
   sudo netplan apply
   ```

**Step 3: Verify your static IP is working**

```bash
# Verify your new static IP is active
ip addr show $(ip route get 1.1.1.1 | grep -oP 'dev \K\S+') | grep 'inet '
# Should show your chosen static IP (e.g., 192.168.1.100)

# Test internet connectivity
ping -c 3 google.com

# Verify gateway connectivity
ping -c 3 192.168.1.1  # Your gateway IP
```

**✅ Static IP Configuration Complete!**

Your Linux machine now has a fixed IP address that will never change after restart/hibernation.

**Step 4: Configure Robot with Your Static IP**

Now configure the robot to connect to your chosen static IP address.

Edit the `config_robot.py` file with your network settings:

```python
# Use YOUR chosen static IP from Step 1
robot.set_wifi_config("your_wifi_name", "your_wifi_password")  
robot.set_udp_config([192, 168, 1, 100], 8090)  # YOUR static IP here
robot.set_car_type(robot.CAR_TYPE_COMPUTER)
```

**⚠️ IMPORTANT:** Use the exact same static IP you configured in Linux (from Step 1).

**Step 5: Verify Everything Works**

```bash
# Start the Micro-ROS agent (it will be accessible on your static IP)
docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:humble udp4 --port 8090

# In another terminal, verify agent is listening
sudo netstat -tulpn | grep 8090

# Check network connectivity
ping -c 3 192.168.1.100  # Your static IP
```

**✅ Network Configuration Complete!**

Your setup now has:
- Linux with fixed static IP address
- Robot configured to connect to your static IP
- Micro-ROS agent accessible on your static IP

## 9. microROS Control Board Configuration

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

1. Connect the robot via USB
2. Briefly press the reset button on the microROS control board
3. The robot enters configuration mode within 5 seconds of booting (MCU indicator flashes every 300ms)
4. Run the configuration script:
   ```bash
   python3 config_robot.py
   ```
5. Verify the returned data matches your settings

## 10. Troubleshooting

### Common Issues

**Issue: Permission denied for serial device**
- Solution: Add user to dialout group and log out/in
- Command: `sudo usermod -a -G dialout $USER`

**Issue: Docker permission denied**
- Solution: Add user to docker group and log out/in
- Command: `sudo usermod -aG docker $USER`

**Issue: Robot not connecting to Micro-ROS agent**
- **Cause:** Either static IP not configured or robot configured with wrong IP
- **Solution:** Follow section 8 Network Configuration steps in order:
  1. Configure Linux with static IP
  2. Configure robot with the same static IP
- **Check:** Verify robot IP matches Linux static IP exactly
- **Verify:** Confirm static IP is active with `ip addr show`

### Network Diagnostics

```bash
# Test Micro-ROS agent connectivity
docker run --rm --net=host microros/micro-ros-agent:humble udp4 --port 8090 -v6

# Check port availability
sudo netstat -tulpn | grep 8090

# Monitor network traffic
sudo tcpdump -i any -n port 8090
```

## 11. Workspace Setup

After completing the installation, you'll need to build the workspaces. See the `WORKSPACE_README.md` for detailed instructions on workspace management.

## 12. Uninstallation (Optional)

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