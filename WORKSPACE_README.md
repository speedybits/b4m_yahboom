# Yahboom Robot Workspaces

This document provides instructions for working with the ROS2 workspaces that have been integrated into this repository.

## Workspace Overview

The following workspaces have been moved into this repository:

1. **gmapping_ws**: Contains SLAM Gmapping packages
   - `openslam_gmapping`
   - `slam_gmapping`

2. **imu_ws**: Contains IMU tools and filters
   - `imu_complementary_filter`
   - `imu_filter_madgwick`
   - `imu_tools`
   - Note: `rviz_imu_plugin` is not built due to a tinyxml2 linking issue

3. **uros_ws**: Contains Micro-ROS setup
   - `micro_ros_setup`
   - `micro_ros_agent`
   - `micro_ros_msgs`

## Using the Workspaces

### Sourcing the Workspaces

To use these workspaces, source the provided script:

```bash
source /home/yahboom/b4m_yahboom/source_workspaces.sh
```

This script will:
1. Source ROS2 Humble
2. Source the main workspace
3. Source the gmapping workspace
4. Source the IMU workspace
5. Source the uROS workspace

After sourcing, all packages from these workspaces will be available in your ROS2 environment.

### When to Source the Workspaces

**Important**: The workspaces must be sourced in each new terminal session before running any ROS2 commands related to these packages.

**Sourcing Sequence**:

1. **Before starting the Micro-ROS agent**: Source the workspaces in the terminal where you'll run the Micro-ROS agent.

2. **Before launching car's underlying data processing**: Source the workspaces in the terminal where you'll run the yahboomcar_bringup launch file.

3. **Before visualization or navigation**: Source the workspaces in any terminal where you'll run RViz, Gmapping, or waypoint navigation commands.

4. **After system reboot**: The sourcing must be done again after any system reboot as the environment variables are not persistent.

Essentially, any terminal that will execute ROS2 commands related to the Yahboom robot needs to have the workspaces sourced first.

### Automatic Sourcing with .bashrc

For convenience, the `source_workspaces.sh` script can be added to your `.bashrc` file to automatically source all workspaces when opening a new terminal:

```bash
# Add to your .bashrc
source /home/yahboom/b4m_yahboom/source_workspaces.sh
```

**Note**: A backup of the original `.bashrc` file has been created at `/home/yahboom/.bashrc.original`. If you need to restore the original configuration, run:

```bash
cp /home/yahboom/.bashrc.original /home/yahboom/.bashrc
```

A reference copy of the `.bashrc` file is also available at `/home/yahboom/b4m_yahboom/bashrc_reference.txt`.

### Rebuilding the Workspaces

If you need to rebuild the workspaces (e.g., after making changes to the source code), use the provided rebuild script:

```bash
/home/yahboom/b4m_yahboom/rebuild_workspaces.sh
```

This script will:
1. Clean the build, install, and log directories for each workspace
2. Rebuild each workspace with the correct paths
3. Skip the problematic `rviz_imu_plugin` package in the imu_ws

## Known Issues

### IMU Workspace

The `rviz_imu_plugin` package in the imu_ws fails to build due to a linking issue with tinyxml2:

```
/usr/bin/ld: /usr/local/lib/libtinyxml2.a(tinyxml2.cpp.o): relocation R_X86_64_PC32 against symbol `_ZN8tinyxml27XMLUtil13writeBoolTrueE' can not be used when making a shared object; recompile with -fPIC
```

This package is skipped during the build process. The core IMU processing packages (`imu_complementary_filter` and `imu_filter_madgwick`) are built successfully and provide the essential IMU functionality.

### Path References

The setup.bash files generated during the build process may still contain references to the original workspace paths. This is a limitation of the ROS2 build system. To ensure correct path usage, always use the provided `source_workspaces.sh` script rather than sourcing the setup.bash files directly.

## Yahboom Robot Navigation

### Power-On Sequence

1. Start the Micro-ROS agent:
   ```bash
   docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:humble udp4 --port 8090
   ```

2. Power on the physical Yahboom robot

3. Launch the car's underlying data processing:
   ```bash
   ros2 launch yahboomcar_bringup yahboomcar_bringup_launch.py
   ```

4. Start RViz for visualization (if needed):
   ```bash
   ros2 launch yahboomcar_nav display_launch.py
   ```

5. Launch the control system (keyboard teleop or waypoint navigation)

### Gmapping Mapping

For Gmapping mapping, follow these steps:

1. Start Micro-ROS agent
2. Launch car's underlying data processing
3. Start RViz for visualization
4. Launch Gmapping mapping node:
   ```bash
   ros2 launch yahboomcar_nav map_gmapping_launch.py
   ```
5. Control the robot with keyboard or joystick
6. Save the map when complete:
   ```bash
   ros2 launch yahboomcar_nav save_map_launch.py
   ```

### Waypoint Navigation

For waypoint navigation, use these keyboard commands:
- s: save waypoint
- l: list waypoints
- g: go to waypoint
- c: cancel navigation
