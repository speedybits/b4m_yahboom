#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_dir = get_package_share_directory('b4m_waypoint_nav')
    
    # Define the path to the RViz configuration file
    rviz_config_file = os.path.join(package_dir, 'rviz', 'b4m_waypoint_nav.rviz')
    
    # Create a launch argument for using RViz
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz'
    )

    # Include the Navigation2 launch file
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('yahboomcar_nav'), 
                        'launch', 
                        'navigation_dwb_launch.py')
        ])
    )
    
    # Start the waypoint navigation node
    waypoint_navigation_node = Node(
        package='b4m_waypoint_nav',
        executable='b4m_waypoint_nav',
        name='b4m_waypoint_nav',
        output='screen'
    )
    
    # Launch RViz with our custom configuration if requested
    rviz_node = Node(
        condition=LaunchConfiguration('use_rviz'),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    
    return LaunchDescription([
        use_rviz_arg,
        nav2_launch,
        waypoint_navigation_node,
        rviz_node
    ])
