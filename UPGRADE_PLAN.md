# B4M Robot Upgrade Plan: ROS2 Humble to Jazzy

## Overview

This document outlines a practical upgrade path from **ROS2 Humble** to **ROS2 Jazzy**, focusing on the micro-ROS ESP32 firmware migration which was previously identified as the primary blocker.

**Key Finding:** The micro-ROS Jazzy ecosystem is more mature than initially assessed. The upgrade is feasible with approximately 2-4 hours of work for the micro-ROS components.

---

## Current System

| Component | Version |
|-----------|---------|
| Ubuntu | 22.04 LTS (Jammy) |
| ROS2 | Humble Hawksbill |
| Python | 3.10 |
| Gazebo | Classic 11.10.2 |
| micro-ROS Agent | `microros/micro-ros-agent:humble` |
| ESP32 | ESP32-S3 via ESP-IDF |
| Transport | UDP over WiFi, port 8090 |

---

## Target System

| Component | Version |
|-----------|---------|
| Ubuntu | 24.04 LTS (Noble) |
| ROS2 | Jazzy Jalisco |
| Python | 3.12 |
| Gazebo | Harmonic |
| micro-ROS Agent | `microros/micro-ros-agent:jazzy` |
| ESP32 | ESP32-S3 via ESP-IDF (Jazzy branch) |
| Transport | UDP over WiFi, port 8090 (unchanged) |

---

## micro-ROS Jazzy Status (Verified January 2025)

| Component | Status | Notes |
|-----------|--------|-------|
| micro-ROS Agent Docker | Available | `microros/micro-ros-agent:jazzy` |
| ESP-IDF Component | Ready | Jazzy branch fully functional |
| Arduino Library | Limited | USB serial only (no WiFi/UDP) |
| UDP/WiFi Transport | Supported | Via ESP-IDF component |

---

## ESP32 Firmware Source Code

### Repository Location

The Yahboom ESP32 firmware source code is available at:

**[Project-03-roscar/04-yahboom-esp32-microrosdemo](https://github.com/Project-03-roscar/04-yahboom-esp32-microrosdemo)**

### Repository Structure

```
04-yahboom-esp32-microrosdemo/
├── 01-publisher/
├── 02-subscriber/
├── 03-multi_topic/
├── 04-buzzer/
├── 05-pwm_servo/
├── 06-speed_control/
├── 07-speed_publisher/
├── 08-imu_publisher/
├── 09-lidar_publisher/
├── 10-custom_transport/
└── sample_project/
    ├── main/              # Contains main.c with app_main()
    ├── CMakeLists.txt     # Build configuration
    └── sdkconfig          # ESP32 configuration
```

### Additional Resources

- [Official YahboomTechnology/MicroROS-Board](https://github.com/YahboomTechnology/MicroROS-Board)
- [micro_ros_espidf_component (Jazzy branch)](https://github.com/micro-ROS/micro_ros_espidf_component/tree/jazzy)
- [Yahboom Study Page](https://www.yahboom.net/study/MicroROS-ESP32)

---

## Migration Steps

### Phase 1: micro-ROS Migration (2-4 hours)

#### Step 1.1: Clone Firmware Repository (10 min)

```bash
cd ~/projects
git clone https://github.com/Project-03-roscar/04-yahboom-esp32-microrosdemo.git yahboom_esp32_firmware

cd yahboom_esp32_firmware
git checkout -b humble-backup
git checkout -b jazzy-migration
```

#### Step 1.2: Update micro-ROS Component to Jazzy (30 min)

The project references the micro_ros_espidf_component. Update to Jazzy branch:

```bash
# Clone Jazzy version of micro-ROS component
cd ~/esp/Samples/extra_components
git clone -b jazzy https://github.com/micro-ROS/micro_ros_espidf_component.git micro_ros_espidf_component_jazzy

# Copy existing colcon.meta configuration (preserves publisher/subscriber limits)
cp micro_ros_espidf_component/colcon.meta micro_ros_espidf_component_jazzy/
```

#### Step 1.3: Rebuild ESP32 Firmware (1-2 hours)

```bash
# Activate ESP-IDF v5.x
source ~/esp/esp-idf/export.sh

# Install dependencies
pip3 install catkin_pkg lark-parser empy colcon-common-extensions

# Navigate to your robot's firmware project
cd ~/projects/yahboom_esp32_firmware/sample_project

# Clean and rebuild with Jazzy micro-ROS
idf.py clean-microros
idf.py set-target esp32s3
idf.py menuconfig  # Configure: WiFi SSID/password, Agent IP, Port 8090
idf.py build
idf.py flash
idf.py monitor  # Verify firmware boots correctly
```

#### Step 1.4: Update Agent Docker Image (5 min)

Edit `b4m_launch.sh` lines 2263 and 3315:

```bash
# Change FROM:
docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:humble udp4 --port 8090

# TO:
docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:jazzy udp4 --port 8090 -v6
```

#### Step 1.5: Verify Communication (30 min)

```bash
# Start Jazzy agent
docker run -it --rm --net=host microros/micro-ros-agent:jazzy udp4 --port 8090 -v6

# Power on robot, then verify topics
ros2 topic list

# Expected topics:
# /odom_raw
# /imu/data
# /scan
# /cmd_vel

# Test motor control
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}" --once
```

---

### Phase 2: Ubuntu & ROS2 Upgrade (3-5 weeks)

Once micro-ROS is working on Jazzy, proceed with the full system upgrade as documented in `UPGRADE_UBUNTU.md`.

#### Priority Order:

1. **Core ROS2 packages** (3-5 days)
2. **Gazebo Classic to Harmonic** (5-10 days) - Largest effort
3. **SLAM migration** (2-3 days) - gmapping to slam_toolbox
4. **Navigation stack** (2-3 days) - Nav2 parameter updates
5. **Integration testing** (3-5 days)

---

## Risk Assessment

| Risk | Level | Mitigation |
|------|-------|------------|
| Agent compatibility | Low | Docker image is official, tested |
| ESP-IDF build issues | Medium | Keep Humble backup branch, can rollback |
| Message format changes | Low | Standard ROS2 messages unchanged |
| WiFi/UDP transport | Low | Fully supported in Jazzy ESP-IDF component |
| Gazebo migration | High | Budget extra time, extensive testing |
| Python 3.12 compatibility | Medium | Test all scripts early |

---

## Rollback Plan

### micro-ROS Rollback

```bash
# Restore Humble agent
# Edit b4m_launch.sh to use :humble instead of :jazzy

# Reflash Humble firmware
cd ~/projects/yahboom_esp32_firmware
git checkout humble-backup
# Rebuild and flash with Humble micro-ROS component
```

### Full System Rollback

1. Keep Ubuntu 22.04 partition/VM available
2. Maintain git branches for both ROS2 versions
3. Document all changes for potential reversal

---

## Alternative: Test on VirtualBox First

For Mac users (Apple Silicon M1/M2/M3/M4):

### Recommended Setup

| Option | Cost | Notes |
|--------|------|-------|
| **UTM** | Free | Best free option, 3D acceleration |
| VirtualBox 7.1+ | Free | ARM64 support, Ubuntu Server only |
| Parallels 20 | $99/yr | Best performance |
| Docker | Free | Lightweight, good for build testing |

### VirtualBox Setup (Apple Silicon)

1. Download [Ubuntu 24.04 Server ARM64](https://ubuntu.com/download/server/arm)
2. Create VM: Type=Linux, Version="Ubuntu ARM 64-bit"
3. Allocate 8GB+ RAM, 4+ CPUs, 50GB+ disk
4. After install: `sudo apt install ubuntu-desktop`
5. Install ROS2 Jazzy and test micro-ROS agent

---

## References

### Official Documentation

- [ROS2 Jazzy Installation](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- [Gazebo Harmonic ROS Installation](https://gazebosim.org/docs/latest/ros_installation/)
- [Nav2 Jazzy Migration](https://docs.nav2.org/migration/Jazzy.html)
- [slam_toolbox Documentation](https://docs.ros.org/en/ros2_packages/jazzy/api/slam_toolbox/)

### micro-ROS Resources

- [micro_ros_espidf_component](https://github.com/micro-ROS/micro_ros_espidf_component)
- [micro-ROS Jazzy Release Discussion](https://github.com/micro-ROS/micro_ros_setup/discussions/694)
- [micro_ros_setup Jazzy README](https://github.com/micro-ROS/micro_ros_setup/blob/jazzy/README.md)

### Yahboom Resources

- [Yahboom ESP32 Firmware Source](https://github.com/Project-03-roscar/04-yahboom-esp32-microrosdemo)
- [YahboomTechnology/MicroROS-Board](https://github.com/YahboomTechnology/MicroROS-Board)
- [Yahboom MicroROS Study Page](https://www.yahboom.net/study/MicroROS-ESP32)

---

## Conclusion

The micro-ROS Jazzy migration is **more feasible than initially assessed**. The ESP32 firmware source code is available, and the Jazzy micro-ROS components are mature enough for production use.

**Recommended approach:**

1. Start with micro-ROS migration (Phase 1) - 2-4 hours
2. Validate hardware communication works with Jazzy agent
3. Then proceed with full Ubuntu/ROS2 upgrade (Phase 2)

This approach de-risks the largest unknown (micro-ROS compatibility) before committing to the full system upgrade.

---

*Document created: January 2025*
*Related document: UPGRADE_UBUNTU.md*
