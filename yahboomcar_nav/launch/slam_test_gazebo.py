import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directories
    nav_package_path = get_package_share_directory('yahboomcar_nav')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # SLAM parameters file
    slam_params_file = os.path.join(nav_package_path, 'params', 'slam_params_gazebo.yaml')
    
    return LaunchDescription([
        # Launch Ignition Gazebo using same approach as working test
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(nav_package_path, 'launch', 'ignition_gazebo_launch.py')
            ]),
            launch_arguments={'use_sim_time': 'true'}.items()
        ),
        
        # Spawn robot using same approach as working test
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(nav_package_path, 'launch', 'spawn_robot_with_controllers_ignition.py')
            ]),
            launch_arguments={'use_sim_time': 'true'}.items()
        ),
        
        # SLAM Toolbox Node (added for SLAM testing)
        TimerAction(
            period=10.0,  # Wait for Gazebo and robot to be ready
            actions=[
                Node(
                    package='slam_toolbox',
                    executable='async_slam_toolbox_node',
                    name='slam_toolbox',
                    output='screen',
                    parameters=[
                        slam_params_file,
                        {'use_sim_time': use_sim_time}
                    ],
                ),
            ]
        ),
    ])