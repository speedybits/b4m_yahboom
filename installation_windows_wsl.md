# ROS2 Humble Installation Guide - Windows WSL2

This guide provides step-by-step instructions to install ROS2 Humble on Windows WSL2 systems to support the B4M Yahboom robot project.

## Prerequisites

- Windows 10 version 2004 and higher (Build 19041 and higher) or Windows 11
- WSL2 installed and configured
- Ubuntu 22.04 LTS (Jammy Jellyfish) on WSL2 recommended
- Internet connection for downloading packages
- Terminal access with sudo privileges in WSL2

## 1. WSL2 Setup

### Install WSL2 (if not already installed)

Open PowerShell as Administrator and run:

```powershell
# Enable WSL and Virtual Machine Platform
dism.exe /online /enable-feature /featurename:Microsoft-Windows-Subsystem-Linux /all /norestart
dism.exe /online /enable-feature /featurename:VirtualMachinePlatform /all /norestart

# Restart Windows, then set WSL2 as default
wsl --set-default-version 2

# Install Ubuntu 22.04
wsl --install -d Ubuntu-22.04
```

### Verify WSL2 Installation

```bash
# In WSL2 terminal, check version
lsb_release -a
```

## 2. Set Language Environment

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
```

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

## 8. Serial Device Access (WSL2-Specific)

WSL2 requires USB passthrough to access serial devices. **Note:** The `/dev/ttyS*` ports in WSL2 are emulated and will give "Input/output error" when trying to connect to real hardware.

### Install usbipd on Windows

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

### USB Device Persistence (IMPORTANT)

**Critical:** USB devices attached via usbipd do NOT automatically reconnect in these situations:
- When you restart WSL2 or start a new WSL2 session
- When your PC hibernates, sleeps, or goes to standby
- When you unplug and replug the USB device
- After Windows updates that restart the system

**Every time ANY of the above happens, you must reattach the USB device:**

1. **Check if device is still attached** (in WSL2):
   ```bash
   ls -la /dev/ttyUSB* /dev/ttyACM*
   ```

2. **If no devices appear, reattach in Windows PowerShell as Administrator:**
   ```powershell
   # List devices to find your robot's BUSID
   usbipd list
   
   # Reattach the robot device
   usbipd attach --wsl --busid <BUSID>
   ```

3. **Verify device appears in WSL2:**
   ```bash
   ls -la /dev/ttyUSB* /dev/ttyACM*
   ```

### USB Workflow Tips

**Recommended daily workflow:**
1. Start your PC and open WSL2
2. **First thing:** Check if USB device is attached with `ls -la /dev/ttyUSB*`
3. If not found, immediately reattach via PowerShell before starting work
4. Consider keeping a PowerShell window open as Administrator for quick reattachment

**Quick reattachment script for PowerShell:**
Create a file `attach-robot.ps1` with your robot's BUSID:
```powershell
# Replace 1-1 with your actual BUSID
usbipd attach --wsl --busid 1-1
Write-Host "Robot attached to WSL2. Check with: ls -la /dev/ttyUSB*"
```

**Preventing sleep/hibernate during robot work:**
- Use Windows Power Settings to prevent sleep during active work
- Or create a PowerShell script to keep system awake during robot sessions

### Troubleshooting WSL2 USB Issues

- If you get "Input/output error" with `/dev/ttyS*` ports, these are not real USB devices
- Ensure the robot is powered on and connected via USB before running `usbipd list`
- **Device missing after WSL2 restart:** This is normal - reattach using `usbipd attach --wsl --busid <BUSID>`
- If device doesn't appear after attach, try detaching and reattaching:
  ```powershell
  usbipd detach --busid <BUSID>
  usbipd attach --wsl --busid <BUSID>
  ```
- The robot device should appear as `/dev/ttyUSB0` or `/dev/ttyACM0` after successful USB passthrough

## 9. Network Configuration (WSL2-Specific)

### Finding Your Windows WiFi IP Address

You need to find your Windows computer's WiFi IP address to configure both the robot and port forwarding.

**Find Windows WiFi IP using Windows Command Prompt (cmd):**

1. **Open Windows Command Prompt:**
   - Press `Windows + R`, type `cmd`, press Enter
   - Or search for "Command Prompt" in Start menu

2. **Get your WiFi IP address:**
   ```cmd
   ipconfig | findstr "IPv4.*192.168"
   ```
   Example output: `IPv4 Address. . . . . . . . . . . : 192.168.68.105`

3. **Alternative command (shows all network adapters):**
   ```cmd
   ipconfig
   ```
   Look for "Wireless LAN adapter Wi-Fi" section and note the IPv4 Address.

**Your Windows WiFi IP address** (e.g., `192.168.68.105`) is what you'll use for:
- Robot configuration in `config_robot.py`
- Port forwarding setup below

### WSL2 Networking Configuration (CRITICAL)

WSL2 uses a virtual network that creates connectivity issues with the ESP32 robot. The ESP32 robot connects to your Windows WiFi IP address, but the Micro-ROS agent runs inside WSL2 with a different IP address (172.x.x.x). This network mismatch prevents the robot from connecting.

**SOLUTION: WSL2 Mirrored Networking (Recommended)**

The simplest and most reliable solution is to use WSL2 mirrored networking, which makes WSL2 share the same network interface as Windows.

**Step 1: Create WSL2 configuration file**

1. **Open Windows Command Prompt or PowerShell:**
   ```powershell
   # Create or edit the .wslconfig file in your user directory
   notepad $env:USERPROFILE\.wslconfig
   ```

2. **Add this content to the file and save:**
   ```
   [wsl2]
   networkingMode=mirrored
   ```

**Step 2: Restart WSL2**
```powershell
# Shutdown WSL2 completely
wsl --shutdown

# Wait about 10 seconds for complete shutdown
Start-Sleep -Seconds 10

# Start WSL2 again
wsl
```

**Step 3: Verify mirrored networking is working**

After WSL2 restarts, the Micro-ROS agent will be directly accessible on your Windows WiFi IP address without any port forwarding.

```bash
# In WSL2, verify your IP configuration
ip addr show eth0

# Start the Micro-ROS agent
docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:humble udp4 --port 8090

# In another WSL2 terminal, verify agent is listening
netstat -tulpn | grep 8090
```

**What mirrored networking does:**
- WSL2 shares the same network interface as Windows
- The Micro-ROS agent becomes directly accessible at your Windows WiFi IP (e.g., `192.168.68.105:8090`)
- No port forwarding or firewall configuration needed
- Robot can connect directly to Windows IP address

**Alternative Solution (if mirrored networking doesn't work):**

**Port Forwarding Method** - Only use if mirrored networking fails:

```powershell
# In Windows PowerShell as Administrator
# Get WSL2 IP and set up port forwarding
$wslIP = (wsl hostname -I).Trim()
netsh interface portproxy add v4tov4 listenport=8090 listenaddress=192.168.68.105 connectport=8090 connectaddress=$wslIP

# Create firewall rule
netsh advfirewall firewall add rule name="WSL2 Micro-ROS Agent" dir=in action=allow protocol=UDP localport=8090

# Verify port forwarding
netsh interface portproxy show all
```

**Note:** Windows port forwarding has UDP limitations, so mirrored networking is strongly recommended.

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

### Robot Configuration

Edit the config_robot.py file to configure your network settings:

1) Update the parameters of the set_wifi_config function according to your WiFi network name and password
2) Update the parameters of the set_udp_config function with your **Windows WiFi IP address**
3) Update the 'set_car_type' to CAR_TYPE_COMPUTER

**With mirrored networking:** The robot connects directly to your Windows WiFi IP address, and WSL2 shares the same network interface.

Example configuration:
```python
robot.set_wifi_config("your_wifi_name", "your_wifi_password")
robot.set_udp_config([192, 168, 68, 105], 8090)  # Use your Windows WiFi IP
robot.set_car_type(robot.CAR_TYPE_COMPUTER)
```

**IP Address Configuration for WSL2 Mirrored Networking:**
- Robot configuration: Use Windows WiFi IP (e.g., `192.168.68.105`)
- Micro-ROS agent: Directly accessible on Windows IP (no forwarding needed)
- Simple and reliable connection

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

1. Connect the robot via USB
2. Briefly press the reset button on the microROS control board
3. The robot enters configuration mode within 5 seconds of booting (MCU indicator flashes every 300ms)
4. Run the configuration script:
   ```bash
   python3 config_robot.py
   ```
5. Verify the returned data matches your settings

## 11. Troubleshooting

### Common WSL2 Issues

**Issue: "No such file or directory" - /dev/ttyUSB0 not found**
- **Cause:** USB device not attached to WSL2 (common after WSL2 restart, PC sleep/hibernate, or system reboot)
- **Solution:** Reattach device using usbipd in Windows PowerShell as Administrator
- **Commands:**
  ```powershell
  usbipd list
  usbipd attach --wsl --busid <BUSID>
  ```
- **Check:** `ls -la /dev/ttyUSB* /dev/ttyACM*` in WSL2

**Issue: "Input/output error" on serial ports**
- Solution: Use proper USB passthrough with usbipd
- Check: `/dev/ttyS*` ports are emulated, need real `/dev/ttyUSB*` or `/dev/ttyACM*`

**Issue: Robot can't connect to Micro-ROS agent**
- **Cause:** Network mismatch between robot (uses Windows WiFi IP) and Micro-ROS agent (runs in WSL2)
- **Solution:** Use WSL2 mirrored networking (see section 9 above) - most reliable approach
- **Check:** Verify `.wslconfig` has `networkingMode=mirrored` and restart WSL2 with `wsl --shutdown`
- **Alternative:** Use port forwarding if mirrored networking doesn't work

**Issue: Docker permission denied**
- Solution: Add user to docker group and restart WSL2
- Command: `sudo usermod -aG docker $USER` then `wsl --shutdown` and restart

**Issue: GUI applications don't work**
- Solution: Install X11 server on Windows (VcXsrv or similar)
- Set: `export DISPLAY=:0` in WSL2

### Network Diagnostics

```bash
# Test Micro-ROS agent connectivity
docker run --rm --net=host microros/micro-ros-agent:humble udp4 --port 8090 -v6

# Check WSL2 network configuration
ip addr show eth0

# Test Windows host connectivity
ping $(cat /etc/resolv.conf | grep nameserver | awk '{print $2}')

# Monitor network traffic
sudo tcpdump -i any -n port 8090
```

## 12. Workspace Setup

After completing the installation, you'll need to build the workspaces. See the `WORKSPACE_README.md` for detailed instructions on workspace management.

## 13. Uninstallation (Optional)

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
