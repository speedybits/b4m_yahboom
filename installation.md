# ROS2 Humble Installation Guide

This guide provides step-by-step instructions to install ROS2 Humble on various platforms to support the B4M Yahboom robot project.

## Platform-Specific Installation Guides

Choose the appropriate guide for your platform:

- **[Native Linux Installation](installation_linux.md)** - For Ubuntu/Debian systems running directly on hardware
- **[Windows WSL2 Installation](installation_windows_wsl.md)** - For Windows users running Ubuntu through WSL2
- **[macOS Installation](installation_mac.md)** - For macOS users running Ubuntu through UTM virtual machine

## General Overview

The installation process includes:
1. Setting up the base ROS2 Humble environment
2. Installing additional dependencies (Docker, Python packages)
3. Configuring serial device access
4. Setting up network configuration for robot communication
5. Installing troubleshooting tools

## Platform-Specific Considerations

### Native Linux
- Direct hardware access to USB devices
- Standard network configuration
- Best performance for robotics applications

### Windows WSL2
- Requires USB passthrough setup (usbipd)
- Special network configuration due to WSL2 virtual networking
- May need port forwarding between Windows and WSL2

### macOS (UTM)
- Requires virtual machine setup
- USB passthrough through UTM
- Network configuration depends on VM settings

## Common Issues Across Platforms

- **Serial Device Access**: Each platform handles USB devices differently
- **Network Configuration**: Robot must connect to correct IP address
- **Micro-ROS Agent**: Docker setup varies by platform
- **Performance**: Native Linux > macOS VM > WSL2 for robotics

## Quick Start

1. Follow your platform-specific installation guide
2. Build the workspaces using `WORKSPACE_README.md`
3. Configure robot with `config_robot.py`
4. Run launch sequence with `b4m_HA_launch.sh`

---

*The sections below contain the original unified installation guide for reference. Please use the platform-specific guides above for new installations.*

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

### Workspace Setup

After cloning the B4M Yahboom repository, you'll need to build the workspaces. See the `WORKSPACE_README.md` for detailed instructions on workspace management.

### Serial Device Access

The config_robot.py script requires serial communication with the robot. Setup varies by platform:

#### WSL2 on Windows

WSL2 requires USB passthrough to access serial devices. **Note:** The `/dev/ttyS*` ports in WSL2 are emulated and will give "Input/output error" when trying to connect to real hardware.

1. **Install usbipd on Windows** (run in PowerShell as Administrator):
   ```powershell
   winget install --interactive --exact dorssel.usbipd-win
   ```

   **Important:** After installation, close PowerShell completely and open a new PowerShell window as Administrator for the PATH changes to take effect.

2. **Connect your robot via USB** and list connected USB devices:
   ```powershell
   usbipd list
   ```
   
   Look for your robot device (might appear as "USB Serial Device", "CH340", "CP210x", or similar).

3. **Bind and attach your robot device** (replace `<BUSID>` with actual bus ID from step 2):
   ```powershell
   usbipd bind --busid <BUSID>
   usbipd attach --wsl --busid <BUSID>
   ```

4. **Add user to dialout group in WSL**:
   ```bash
   sudo usermod -a -G dialout $USER
   newgrp dialout
   ```

5. **Verify device appears** (should now show `/dev/ttyUSB0` or `/dev/ttyACM0`):
   ```bash
   ls -la /dev/ttyUSB* /dev/ttyACM*
   ```

**Troubleshooting WSL2 USB Issues:**
- If you get "Input/output error" with `/dev/ttyS*` ports, these are not real USB devices
- Ensure the robot is powered on and connected via USB before running `usbipd list`
- If device doesn't appear after attach, try detaching and reattaching:
  ```powershell
  usbipd detach --busid <BUSID>
  usbipd attach --wsl --busid <BUSID>
  ```
- The robot device should appear as `/dev/ttyUSB0` or `/dev/ttyACM0` after successful USB passthrough

#### UTM on macOS

USB passthrough should work automatically. Devices typically appear as:
- `/dev/cu.usbserial-*`
- `/dev/cu.wch*` (CH340/CH341 chips)
- `/dev/tty.usb*`

#### Native Linux

Devices should appear automatically as `/dev/ttyUSB0` or `/dev/ttyACM0`. Add user to dialout group:
```bash
sudo usermod -a -G dialout $USER
```

### microROS Control Board Configuration

The config_robot.py script now supports cross-platform serial port detection:

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

### Finding Your Computer's IP Address

The robot needs to connect to your computer's IP address on the local network. The method to find this varies by platform:

#### WSL2 on Windows

WSL2 uses a virtual network bridge. Use the WSL2 IP address:

```bash
# In WSL2 terminal, find the WSL2 IP address
ip addr show eth0 | grep 'inet ' | awk '{print $2}' | cut -d/ -f1
```

Alternatively, you can use the Windows host IP that WSL2 can reach:

```bash
# Find Windows host IP from WSL2
cat /etc/resolv.conf | grep nameserver | awk '{print $2}'
```

**Note:** The WSL2 internal IP (usually 172.x.x.x) is preferred for robot communication.

**WSL2 Networking Troubleshooting:**

WSL2 uses a virtual network that can cause connectivity issues with the robot. Common problems and solutions:

1. **Robot can't connect to Micro-ROS agent in WSL2:**
   - The robot (ESP32) connects to your Windows host's WiFi IP
   - The Micro-ROS agent runs in WSL2 with a different IP (172.x.x.x)
   - This creates a network mismatch

2. **Solution Options:**

   **Option A: Configure robot to use WSL2 IP (Recommended)**
   ```bash
   # Find your WSL2 IP address
   ip addr show eth0 | grep 'inet ' | awk '{print $2}' | cut -d/ -f1
   
   # Configure robot with this IP in config_robot.py
   robot.set_udp_config([172, 26, 165, 104], 8090)  # Use your WSL2 IP
   ```

   **Option B: Run Micro-ROS agent on Windows**
   ```powershell
   # In Windows PowerShell (install Docker Desktop first)
   docker run -it --rm -p 8090:8090/udp microros/micro-ros-agent:humble udp4 --port 8090
   ```

   **Option C: WSL2 Port Forwarding**
   ```powershell
   # In Windows PowerShell as Administrator
   # Forward port 8090 from Windows to WSL2
   netsh interface portproxy add v4tov4 listenport=8090 listenaddress=0.0.0.0 connectport=8090 connectaddress=172.26.165.104
   ```

3. **Verify WSL2 Network Configuration:**
   ```bash
   # Check if WSL2 can reach Windows host
   ping $(cat /etc/resolv.conf | grep nameserver | awk '{print $2}')
   
   # Check WSL2 network interfaces
   ip addr show
   
   # Test port availability
   sudo netstat -tulpn | grep 8090
   ```

4. **Testing Robot Connectivity:**
   ```bash
   # Start agent with verbose output to see connection attempts
   docker run --rm --net=host microros/micro-ros-agent:humble udp4 --port 8090 -v6
   
   # In another terminal, monitor network traffic
   sudo tcpdump -i any -n port 8090
   ```

**Important:** The robot must be configured with the IP address where the Micro-ROS agent is actually running, not just your Windows WiFi IP.

#### UTM on macOS

UTM typically uses bridged networking. Find your Mac's IP address:

```bash
# Find active network interface IP
ifconfig | grep 'inet ' | grep -v 127.0.0.1 | awk '{print $2}' | head -1
```

Or check specific interfaces:
```bash
# WiFi interface (common)
ifconfig en0 | grep 'inet ' | awk '{print $2}'

# Ethernet interface
ifconfig en1 | grep 'inet ' | awk '{print $2}'
```

#### Native Linux

Find your Linux machine's IP address:

```bash
# Find active network interface IP
hostname -I | awk '{print $1}'

# Or use ip command
ip route get 1.1.1.1 | grep -oP 'src \K\S+'

# Or check specific interface
ip addr show wlan0 | grep 'inet ' | awk '{print $2}' | cut -d/ -f1  # WiFi
ip addr show eth0 | grep 'inet ' | awk '{print $2}' | cut -d/ -f1   # Ethernet
```

### Network Configuration

Edit the config_robot.py file to configure your network settings:

1) Update the parameters of the set_wifi_config function according to your WiFi network name and password
2) Update the parameters of the set_udp_config function according to your computer's IP address (found above)
3) Update the 'set_car_type' to CAR_TYPE_COMPUTER

Example configuration:
```python
robot.set_wifi_config("your_wifi_name", "your_wifi_password")
robot.set_udp_config([192, 168, 1, 100], 8090)  # Replace with your IP
robot.set_car_type(robot.CAR_TYPE_COMPUTER)
```

**IP Address Examples by Platform:**
- WSL2: `[172, 20, 10, 2]` or similar 172.x.x.x address
- UTM/macOS: `[192, 168, 1, 100]` or your local network range
- Native Linux: `[192, 168, 1, 150]` or your local network range

**Important:** Ensure your computer and robot are on the same WiFi network for proper communication.

**Configuration Steps:**
1. Connect the robot via USB
2. Briefly press the reset button on the microROS control board
3. The robot enters configuration mode within 5 seconds of booting (MCU indicator flashes every 300ms)
4. Run the configuration script:
   ```bash
   python3 config_robot.py
   ```
5. Verify the returned data matches your settings


## 7. Uninstallation (Optional)

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
