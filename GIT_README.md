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
