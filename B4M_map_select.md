# Map Selector for Yahboom Navigation

This document specifies the requirements and usage for the `map_selector.sh` script, which allows users to select which map to use for navigation with the Yahboom robot.

## Purpose

The map selector script provides a text-based menu interface that allows users to:
1. View all available maps across all map directories
2. Select a map to use for navigation
3. Copy the selected map files to the default `yahboom_map` name used by the navigation system

## Map Storage in ROS2

In the ROS2 ecosystem, maps are managed through a source-to-install workflow:

1. **Source Directory**: `/home/yahboom/b4m_yahboom/yahboomcar_nav/maps/`
   - This is where maps are stored in the source code repository
   - Changes made here will be tracked by Git
   - Maps should be managed in this directory

2. **Install Directory**: `/home/yahboom/b4m_yahboom/install/yahboomcar_nav/share/yahboomcar_nav/maps/`
   - This is where maps are installed when the package is built with `colcon build`
   - ROS2 nodes like the waypoint manager look for maps in this location
   - This directory is automatically populated during the build process

Following ROS2 best practices, we should only manage maps in the source directory and use `colcon build` to propagate those changes to the install directory. This ensures proper integration with the ROS2 build system and avoids manual synchronization issues.

## Requirements

1. The script must scan the source map directory:
   - `/home/yahboom/b4m_yahboom/yahboomcar_nav/maps/`
2. Only map files with both `.pgm` and `.yaml` extensions should be considered valid maps
3. The script must provide a numbered menu of available maps
4. When a map is selected, the script must:
   - Copy the selected `.pgm` file to `yahboom_map.pgm` in the source directory
   - Copy the selected `.yaml` file to `yahboom_map.yaml` in the source directory
   - Run `colcon build --packages-select yahboomcar_nav` to update the install directory
5. The script must provide feedback on the success or failure of the operation
6. The script must handle errors gracefully (e.g., missing files, permission issues)
7. The script must provide an option to exit without making changes

## Usage

```bash
./map_selector.sh
```

## Example Interaction

```
===== Yahboom Robot Map Selector =====

Available maps:
1) home
2) office
3) bak
4) test1219
5) testaa
6) Exit without changes

Enter the number of the map you want to use: 1

Selected map: home
Copying map files to source directory...
Running colcon build to update installed maps...
Map selection complete! The robot will now use the 'home' map for navigation.

To use this map, launch the navigation system with:
ros2 launch yahboomcar_nav waypoint_navigation_launch.py
```

## Integration with B4M_launch.md Workflow

The map selector script should be run before step 5 in the B4M_launch.md workflow:

```
4. Start RViz for Visualization (Optional)
   ...

4a. [NEW] Select the map to use for navigation
   ```bash
   cd /home/yahboom/b4m_yahboom
   ./map_selector.sh
   ```

5. Launch the Navigation System
   ...
```

## Implementation Notes

1. The script should be placed in the root directory of the b4m_yahboom project
2. The script must have executable permissions (`chmod +x map_selector.sh`)
3. The script should source any necessary ROS2 environment files if needed
4. The script should be compatible with bash shell
5. The script integrates with the ROS2 build system:
   - Creates the source directory if it doesn't exist
   - Updates the source directory with the selected map
   - Runs colcon build to update the install directory
   - Follows ROS2 best practices for package management

## Technical Details

### ROS2 Build System Workflow

The ROS2 build system follows a specific workflow:

1. **Development workflow**: During development, you modify files in the source directory
2. **Build process**: When you run `colcon build`, files are copied from source to install
3. **Runtime**: ROS2 nodes use files from the install directory at runtime

### How the Script Works with ROS2

The `map_selector.sh` script integrates with this workflow by:

1. **Finding maps**: It searches the source directory for valid maps (those with both .pgm and .yaml files)
2. **Managing source files**: It copies the selected map to the standard name in the source directory
3. **Building the package**: It runs `colcon build --packages-select yahboomcar_nav` to update the install directory
4. **Proper integration**: This approach follows ROS2 best practices and ensures the map is available to all ROS2 components

By using the ROS2 build system instead of manually copying files to the install directory, we avoid potential synchronization issues and ensure that the map selection works correctly with all ROS2 components.
