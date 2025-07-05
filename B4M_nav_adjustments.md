# B4M Robot Navigation Parameter Adjustments

## Overview
This document records the navigation parameter adjustments made on July 4, 2025, to address issues with the robot losing track of its position during navigation and aborting waypoint navigation tasks.

## Problem Description
The B4M robot was experiencing localization issues during navigation:
- The robot would frequently lose track of its position in the room
- Navigation tasks would abort unexpectedly
- Position tracking was unstable, especially when moving through challenging areas

## Parameter Adjustments

### AMCL (Adaptive Monte Carlo Localization) Parameters
These changes improve the robot's ability to maintain an accurate position estimate:

| Parameter | Original Value | New Value | Purpose |
|-----------|---------------|-----------|---------|
| alpha1-5 | 0.2 | 0.1 | Reduced motion model noise parameters for more stable position tracking |
| max_particles | 2000 | 3000 | Increased particle count for better position estimation |
| do_beamskip | false | true | Enabled beam skipping to better handle dynamic obstacles |
| transform_tolerance | 2.0 | 1.0 | Reduced for more responsive position updates |
| update_min_a | 0.1 | 0.05 | Lower angular threshold for position updates |
| update_min_d | 0.1 | 0.05 | Lower distance threshold for position updates |
| pf_err | 0.05 | 0.01 | Reduced particle filter error for more accurate localization |
| recovery_alpha_fast | 0.2 | 0.1 | Adjusted recovery parameters for better recovery from kidnapped robot problem |
| recovery_alpha_slow | 0.1 | 0.05 | Adjusted recovery parameters for better recovery from kidnapped robot problem |
| z_hit | 0.5 | 0.7 | Increased weight for scan matching |
| z_rand | 0.5 | 0.3 | Decreased weight for random measurements |

### Navigation Controller Parameters
These changes improve the robot's navigation behavior:

| Parameter | Original Value | New Value | Purpose |
|-----------|---------------|-----------|---------|
| required_movement_radius | 0.5 | 0.3 | Reduced required movement radius for progress checking |
| movement_time_allowance | 10.0 | 15.0 | Increased time allowance for movement |
| xy_goal_tolerance | 0.25 | 0.3 | Increased position tolerance for goal reaching |
| yaw_goal_tolerance | 0.25 | 0.3 | Increased orientation tolerance for goal reaching |
| transform_tolerance (controller) | 0.2 | 0.5 | Increased transform tolerance for more robust navigation |

## Expected Improvements
These parameter adjustments should result in:
- More robust localization during navigation
- Fewer instances of the robot losing track of its position
- Better recovery when encountering unexpected obstacles or sensor noise
- Reduced likelihood of navigation tasks being aborted
- Overall more reliable waypoint navigation

## Implementation
The changes were implemented by modifying the `/home/yahboom/b4m_yahboom/yahboomcar_nav/params/dwb_nav_params.yaml` file and rebuilding the package using:
```bash
cd /home/yahboom/b4m_yahboom
colcon build --packages-select yahboomcar_nav
. install/setup.bash
```

## Testing
After implementing these changes, the robot should be tested with the standard navigation sequence:
1. Start the Micro-ROS agent
2. Power on the physical robot
3. Launch the car's underlying data processing
4. Start RViz for visualization
5. Launch the waypoint navigation system
6. Test navigation to multiple waypoints to verify improved stability

## Future Considerations
If localization issues persist, consider:
- Further tuning of AMCL parameters
- Improving the quality of the map
- Checking for sensor issues or calibration problems
- Adding additional landmarks or features to the environment to aid in localization
