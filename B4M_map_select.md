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

2. **Runtime Access**: Unlike code and launch files, maps are loaded directly from the source directory:
   - `/home/yahboom/b4m_yahboom/yahboomcar_nav/maps/`
   - ROS2 navigation nodes load map files directly from this location at runtime
   - No build step is required when changing map files

Following ROS2 best practices for data files, we manage maps in the source directory and they are accessed directly at runtime. This ensures immediate availability of map changes without requiring a build step.

## Requirements

1. The script must scan the source map directory:
   - `/home/yahboom/b4m_yahboom/yahboomcar_nav/maps/`
2. Only map files with both `.pgm` and `.yaml` extensions should be considered valid maps
3. The script must provide a numbered menu of available maps
4. When a map is selected, the script must:
   - Copy the selected `.pgm` file to `yahboom_map.pgm` in the source directory
   - Copy the selected `.yaml` file to `yahboom_map.yaml` in the source directory
   - No need to run `colcon build` as maps are loaded directly from the filesystem at runtime
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
Map selection complete! The robot will now use the 'home' map for navigation.

To use this map, launch the navigation system with:
ros2 launch yahboomcar_nav waypoint_navigation_launch.py maps:=/home/yahboom/b4m_yahboom/yahboomcar_nav/maps/yahboom_map.yaml
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

### Map File Handling in ROS2

While most ROS2 resources follow the source-to-install workflow, map files are handled differently:

1. **Development workflow**: Maps are created and stored in the source directory
2. **Runtime access**: Navigation2 and related components load map files directly from the source directory
3. **No build required**: Unlike code changes, map file changes do not require rebuilding the package

### How the Script Works with ROS2

The `map_selector.sh` script integrates with this workflow by:

1. **Finding maps**: It searches the source directory for valid maps (those with both .pgm and .yaml files)
2. **Managing source files**: It copies the selected map to the standard name in the source directory
3. **No build required**: Unlike other ROS2 resources, map files are loaded directly from the filesystem at runtime, so no `colcon build` step is needed
4. **Proper integration**: This approach follows ROS2 best practices for data files and ensures the map is immediately available to all ROS2 components

By directly accessing map files from the filesystem rather than going through the ROS2 build system, we achieve faster map switching and avoid unnecessary build steps. Navigation2 and related components load map files directly at runtime, so changes are immediately available without rebuilding.
