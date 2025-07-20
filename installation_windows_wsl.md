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

### Install Robot Packages

Install additional ROS2 packages required for the robot's functionality:

```bash
sudo apt install ros-humble-imu-complementary-filter ros-humble-imu-filter-madgwick ros-humble-imu-tools ros-humble-robot-localization ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui ros-humble-xacro ros-humble-nav2-bringup
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
sudo apt install python3-pip
pip3 install pillow numpy
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

### Static IP Configuration (REQUIRED)

**CRITICAL:** DHCP networks assign different IP addresses after hibernation/restart, breaking robot connectivity. Static IP configuration is REQUIRED for reliable operation.

**Solution:** Configure Windows to use a static IP address within your network range.

**Step 1: Determine your network range and choose a static IP**

1. **Get your current network information** (Windows Command Prompt):
   ```cmd
   ipconfig /all
   ```
   
2. **Look for "Wireless LAN adapter Wi-Fi" section and note:**
   - Current IPv4 Address (e.g., `192.168.68.105`) 
   - Subnet Mask (e.g., `255.255.255.0`)
   - Default Gateway (e.g., `192.168.68.1`)
   - DNS Servers (e.g., `192.168.68.1`)

3. **Choose your static IP address** (this is the IP you'll use everywhere):
   - Pick an address in the same network range as your current IP
   - Choose something easy to remember and unlikely to be used by other devices
   - **Example:** If your current IP is `192.168.68.105`, choose `192.168.68.100`
   - **Your chosen static IP will be used for:**
     - Windows network configuration
     - Robot configuration in `config_robot.py`
     - All future connections

**📝 Write down your chosen static IP:** `192.168.68.100` (example - use your chosen IP)

**Step 2: Configure Windows with your chosen static IP**

1. **Open Network Settings:**
   - Right-click network icon in system tray
   - Select "Open Network & Internet settings"
   - Click "Change adapter options"

2. **Configure WiFi adapter:**
   - Right-click your WiFi adapter
   - Select "Properties"
   - Double-click "Internet Protocol Version 4 (TCP/IPv4)"

3. **Set your chosen static IP:**
   - Select "Use the following IP address"
   - **IP address:** `192.168.68.100` (use YOUR chosen static IP)
   - **Subnet mask:** `255.255.255.0` (from step 1)
   - **Default gateway:** `192.168.68.1` (from step 1)
   - **Preferred DNS server:** `192.168.68.1` (from step 1)
   - **Alternate DNS server:** `8.8.8.8` (Google DNS as backup)
   - Click "OK" and "OK"

**Step 3: Verify your static IP is working**

```cmd
# Verify your new static IP is active
ipconfig | findstr "IPv4.*192.168"
# Should show your chosen static IP (e.g., 192.168.68.100)

# Test internet connectivity
ping google.com
```

**✅ Static IP Configuration Complete!**

Your Windows machine now has a fixed IP address that will never change after hibernation/restart.

**Step 4: Configure WSL2 Mirrored Networking**

WSL2 normally uses a separate virtual network, which prevents the robot from connecting to the Micro-ROS agent. We need to configure WSL2 to share the same network as Windows.

1. **Create WSL2 configuration file:**
   ```powershell
   # Create or edit the .wslconfig file in your user directory
   notepad $env:USERPROFILE\.wslconfig
   ```

2. **Add this content to the file and save:**
   ```
   [wsl2]
   networkingMode=mirrored
   ```

3. **Restart WSL2 to apply the changes:**
   ```powershell
   # Shutdown WSL2 completely
   wsl --shutdown
   
   # Wait about 10 seconds for complete shutdown
   Start-Sleep -Seconds 10
   
   # Start WSL2 again
   wsl
   ```

**Step 5: Configure Robot with Your Static IP**

Now configure the robot to connect to your chosen static IP address.

Edit the `config_robot.py` file with your network settings:

```python
# Use YOUR chosen static IP from Step 1
robot.set_wifi_config("your_wifi_name", "your_wifi_password")  
robot.set_udp_config([192, 168, 68, 100], 8090)  # YOUR static IP here
robot.set_car_type(robot.CAR_TYPE_COMPUTER)
```

**⚠️ IMPORTANT:** Use the exact same static IP you configured in Windows (from Step 1).

**Step 6: Verify Everything Works**

After WSL2 restarts with mirrored networking:

```bash
# In WSL2, verify network configuration
ip addr show eth0

# In another WSL2 terminal, verify agent is listening
netstat -tulpn | grep 8090
```

**✅ Network Configuration Complete!**

Your setup now has:
- Windows with fixed static IP address
- WSL2 sharing the same network as Windows  
- Robot configured to connect to your static IP
- Micro-ROS agent accessible on your static IP

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


# Test: Start the Micro-ROS agent
docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:humble udp4 --port 8090


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
- **Cause:** Either static IP not configured, WSL2 mirrored networking not set up, or robot configured with wrong IP
- **Solution:** Follow section 9 Network Configuration steps in order:
  1. Configure Windows with static IP
  2. Set up WSL2 mirrored networking  
  3. Configure robot with the same static IP
- **Check:** Verify robot IP matches Windows static IP exactly
- **Verify:** Confirm `.wslconfig` has `networkingMode=mirrored` and restart WSL2

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
