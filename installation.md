# ROS2 Humble Installation Guide

This guide provides step-by-step instructions to install ROS2 Humble on Ubuntu systems to support the B4M Yahboom robot project.

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

### microROS control board WIFI setup
Edit the config_robot.py file:
1) update the parameters of the set_wifi_config function according to your own WiFi network name and password
2) update the parameters of the set_udp_config function according to the IP address of the virtual machine/computer
3) Update the 'set_car_type' to CAR_TYPE_COMPUTER

Example:
robot.set_wifi_config("ssid123", "passwd123")
robot.set_udp_config([192, 168, 2, 116], 8090)
robot_set_car_type(robot.CAR_TYPE_COMPUTER)

First, briefly press the reset button on the microROS control board. It will be in the configuration state within 5 seconds of booting (the MCU indicator light flashes once every 300 milliseconds). Then run the following command to configure the robot. At this time, check whether the returned data is consistent with your own settings. If it is consistent, the setting is successful.

python3 config_robot.py


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
