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
   - Download from: https://ubuntu.com/download/desktop
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

### Finding Your Mac's IP Address

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

### UTM Networking Configuration (CRITICAL)

UTM virtual machines use separate networking that creates connectivity issues with the ESP32 robot. The ESP32 robot connects to your Mac's WiFi IP address, but the Micro-ROS agent runs inside the UTM VM with a different IP address. This network mismatch prevents the robot from connecting.

**SOLUTION OPTIONS:**

**Option A: Use Bridged Networking (Recommended)**

1. **Configure UTM for bridged networking:**
   - Power off your Ubuntu VM
   - In UTM, edit VM settings → Network
   - Change from "Shared Network" to "Bridged Network"
   - Select your Mac's active network interface (usually en0 for WiFi)
   - Start the VM

2. **Verify VM gets IP on same network as Mac:**
   ```bash
   # Check VM IP address
   ip addr show | grep 'inet ' | grep -v 127.0.0.1
   ```
   The VM should now get an IP in the same range as your Mac (e.g., 192.168.1.x)

3. **Configure robot with VM's bridged IP:**
   ```python
   # Use the VM's bridged network IP, not the Mac's IP
   robot.set_udp_config([192, 168, 1, 105], 8090)  # VM's bridged IP
   ```

**Option B: Port Forwarding (if bridged doesn't work)**

1. **Find network addresses:**
   ```bash
   # In UTM VM, find VM IP
   ip addr show | grep 'inet ' | grep -v 127.0.0.1 | awk '{print $2}' | cut -d/ -f1
   # Example: 192.168.64.15
   
   # On Mac, find Mac IP
   ifconfig en0 | grep 'inet ' | awk '{print $2}'
   # Example: 192.168.1.100
   ```

2. **Set up port forwarding (Mac Terminal):**
   ```bash
   # Forward port 8090 from Mac IP to VM IP
   # Replace IPs with your actual addresses
   sudo pfctl -e
   echo "rdr pass on en0 inet proto udp from any to 192.168.1.100 port 8090 -> 192.168.64.15 port 8090" | sudo pfctl -f -
   ```

**Option C: Run Micro-ROS Agent on macOS**

1. **Install Docker Desktop on macOS**
2. **Run agent on Mac instead of VM:**
   ```bash
   # On macOS (not in UTM VM)
   docker run -it --rm -p 8090:8090/udp microros/micro-ros-agent:humble udp4 --port 8090
   ```

3. **Configure robot with Mac IP:**
   ```python
   robot.set_udp_config([192, 168, 1, 100], 8090)  # Mac's actual IP
   ```

**Verify Network Configuration:**
```bash
# Test connectivity between VM and Mac
ping <mac_ip_address>

# Check if port 8090 is listening
netstat -tulpn | grep 8090

# Monitor network traffic
sudo tcpdump -i any -n port 8090
```

### Robot Configuration

Edit the config_robot.py file to configure your network settings:

1) Update the parameters of the set_wifi_config function according to your WiFi network name and password
2) Update the parameters of the set_udp_config function according to your chosen networking solution above
3) Update the 'set_car_type' to CAR_TYPE_COMPUTER

**IP Configuration depends on your networking solution:**

**For Bridged Networking (Option A):**
```python
robot.set_wifi_config("your_wifi_name", "your_wifi_password")
robot.set_udp_config([192, 168, 1, 105], 8090)  # Use VM's bridged IP
robot.set_car_type(robot.CAR_TYPE_COMPUTER)
```

**For Port Forwarding (Option B):**
```python
robot.set_wifi_config("your_wifi_name", "your_wifi_password")
robot.set_udp_config([192, 168, 1, 100], 8090)  # Use Mac's IP (forwarded to VM)
robot.set_car_type(robot.CAR_TYPE_COMPUTER)
```

**For Agent on macOS (Option C):**
```python
robot.set_wifi_config("your_wifi_name", "your_wifi_password")
robot.set_udp_config([192, 168, 1, 100], 8090)  # Use Mac's IP directly
robot.set_car_type(robot.CAR_TYPE_COMPUTER)
```

**Important:** Ensure your Mac and robot are on the same WiFi network for proper communication.

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
- **Cause:** Network mismatch between robot (uses Mac WiFi IP) and Micro-ROS agent (runs in UTM VM)
- **Solution:** Use one of the three networking solutions in section 9 above:
  - Bridged networking (recommended)
  - Port forwarding from Mac to VM
  - Run agent on macOS instead of VM
- **Check:** Verify robot IP configuration matches your chosen networking solution

**Issue: GUI applications don't work properly**
- Solution: Install additional graphics drivers in Ubuntu VM
- Command: `sudo apt install ubuntu-desktop-minimal`

### Network Diagnostics

```bash
# Test Micro-ROS agent connectivity
docker run --rm --net=host microros/micro-ros-agent:humble udp4 --port 8090 -v6

# Check network configuration
ip addr show

# Test Mac connectivity
ping <mac_ip_address>

# Monitor network traffic
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