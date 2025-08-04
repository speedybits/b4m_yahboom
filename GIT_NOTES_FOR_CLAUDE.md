# Git Repository Structure and Organization

This document explains how the files in this repository are organized and committed, focusing on the ROS2 workspaces integration.

## Repository Organization

The repository follows these principles:

1. **Self-contained**: All dependencies are contained within the `/home/yahboom/b4m_yahboom` directory structure
2. **No external path references**: No references to paths outside the repository (e.g., `/home/yahboom/yahboomcar_ws`)
3. **Clean version control**: Build artifacts are excluded from git tracking

## ROS2 Workspaces

The repository includes three ROS2 workspaces that were moved from their original locations:

- `gmapping_ws/` - SLAM Gmapping packages
- `imu_ws/` - IMU sensor processing packages
- `uros_ws/` - Micro-ROS agent and related packages

### Workspace Tracking Strategy

Following git best practices, we've chosen to:

1. **Track the workspace structure but not the content**:
   - Only `.gitkeep` files are committed in the `src/` directories
   - Build artifacts (`build/`, `install/`, `log/`) are excluded via `.gitignore`
   - This avoids issues with embedded git repositories in the source code

### How `.gitkeep` Files Work

The `.gitkeep` files serve a special purpose in this repository:

1. **Empty Directory Tracking**: Git doesn't track empty directories by default. The `.gitkeep` files (which can be any name, but `.gitkeep` is a convention) allow us to track the directory structure without tracking the contents.

2. **Implementation in .gitignore**: We use a pattern in `.gitignore` that excludes everything in a directory but keeps the directory structure:
   ```
   workspace_name/*          # Ignore all files in the workspace
   !workspace_name/src/      # But don't ignore the src directory
   workspace_name/src/*      # Ignore all files in the src directory
   !workspace_name/src/.gitkeep  # But don't ignore the .gitkeep file
   ```

3. **Benefits**:
   - Preserves the expected directory structure for ROS2 workspaces
   - Avoids tracking hundreds of source files that might change frequently
   - Prevents issues with embedded git repositories (nested .git directories)
   - Makes it clear which directories should exist even if they're empty after cloning

### Workspace Contents Location

**Important**: While the repository tracks the workspace directory structure, the actual source code contents are not included in git. The contents are physically present on the system at:

- `/home/yahboom/b4m_yahboom/gmapping_ws/src/` - Contains SLAM Gmapping packages
- `/home/yahboom/b4m_yahboom/imu_ws/src/` - Contains IMU sensor processing packages
- `/home/yahboom/b4m_yahboom/uros_ws/src/` - Contains Micro-ROS agent and related packages

**Setting Up Workspace Contents**

After cloning this repository, you need to populate the workspace source directories. Follow these steps to clone the required packages from their original repositories:

### 1. Setting up gmapping_ws

```bash
# Create the workspace directory structure
mkdir -p /home/yahboom/b4m_yahboom/gmapping_ws/src
cd /home/yahboom/b4m_yahboom/gmapping_ws/src

# Clone the slam_gmapping repository (ROS2 port)
git clone https://github.com/Project-MANAS/slam_gmapping.git

# The openslam_gmapping package is included in the slam_gmapping repository
# No need to clone it separately
```

### 2. Setting up imu_ws

```bash
# Create the workspace directory structure
mkdir -p /home/yahboom/b4m_yahboom/imu_ws/src
cd /home/yahboom/b4m_yahboom/imu_ws/src

# Clone the imu_tools repository (humble branch)
git clone -b humble https://github.com/CCNYRoboticsLab/imu_tools.git imu_tools-humble
```

### 3. Setting up uros_ws

```bash
# Create the workspace directory structure
mkdir -p /home/yahboom/b4m_yahboom/uros_ws/src
cd /home/yahboom/b4m_yahboom/uros_ws/src

# Clone the micro_ros_setup repository
git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git

# Create an empty ros2.repos file (required by the build process)
echo 'repositories: {}' > ros2.repos

# Set up the micro-ROS agent
cd /home/yahboom/b4m_yahboom/uros_ws
colcon build
source install/local_setup.sh
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
```

After setting up all the workspaces, run the rebuild script to ensure everything is built correctly with the proper paths:

```bash
cd /home/yahboom/b4m_yahboom
./rebuild_workspaces.sh
```

This will clean and rebuild all workspaces with the correct install prefixes, ensuring they work properly within the repository structure.

### Dependencies and Known Issues

1. **Required ROS2 Packages**
   - Make sure you have the following ROS2 packages installed:
     ```bash
     sudo apt install ros-humble-tf2-ros ros-humble-nav2-* ros-humble-geometry-msgs ros-humble-sensor-msgs ros-humble-rclcpp
     ```

2. **Known Issue with rviz_imu_plugin**
   - The `rviz_imu_plugin` package in `imu_ws` may fail to build due to a linking error with `tinyxml2` (missing `-fPIC` flag)
   - This is handled in the `rebuild_workspaces.sh` script by skipping this package
   - If you need this plugin, you may need to modify the CMakeLists.txt to add the `-fPIC` flag

3. **Micro-ROS Agent Setup**
   - For the physical Yahboom robot, remember the correct power-on sequence:
     1. Start the Micro-ROS agent: `docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:humble udp4 --port 8090`
     2. Power on the physical robot
     3. Launch the car's underlying data processing: `ros2 launch yahboomcar_bringup yahboomcar_bringup_launch.py`

Refer to the `WORKSPACE_README.md` for more details on workspace usage and operation sequences.

2. **Document instead of duplicate**:
   - `WORKSPACE_README.md` provides comprehensive documentation on workspace usage
   - Helper scripts handle rebuilding and sourcing

## Helper Scripts

The repository includes scripts to manage the workspaces:

- `rebuild_workspaces.sh` - Cleans and rebuilds all workspaces with correct paths
- `source_workspaces.sh` - Sources all workspaces in the correct order

## Documentation

- `WORKSPACE_README.md` - Detailed instructions for using the workspaces
- `bashrc_reference.txt` - Reference copy of the `.bashrc` configuration
- `GIT_README.md` (this file) - Explains the git repository organization

## Excluded Files

The following files are excluded from git tracking:

- Build artifacts (`build/`, `install/`, `log/`)
- ROS transform frame visualization files (`frames*.gv`, `frames*.pdf`)
- Other standard exclusions (object files, libraries, etc.)

## Commit Strategy

When making changes to this repository:

1. **Scripts and configuration**: Commit any changes to scripts, launch files, or configuration
2. **Documentation**: Keep the documentation up to date with any changes
3. **Workspace structure**: If adding new workspaces, follow the same pattern of tracking structure but not content

## Rebuilding After Clone

After cloning this repository, you'll need to:

1. Run `./rebuild_workspaces.sh` to build all the workspaces
2. Source the workspaces using `source ./source_workspaces.sh`
3. Optionally, add the sourcing script to your `.bashrc` for automatic setup

See `WORKSPACE_README.md` for detailed instructions on workspace usage.
