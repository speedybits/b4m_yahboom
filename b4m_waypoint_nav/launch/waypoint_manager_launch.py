#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('b4m_waypoint_nav')
    
    # Launch arguments
    connected_mode = LaunchConfiguration('connected_mode')
    
    # Declare launch arguments
    declare_connected_mode = DeclareLaunchArgument(
        'connected_mode',
        default_value='false',
        description='Enable connection to running robot/simulation'
    )
    
    # Create the waypoint manager node
    waypoint_manager_node = Node(
        package='b4m_waypoint_nav',
        executable='waypoint_manager',
        name='waypoint_manager',
        output='screen',
        parameters=[{
            'connected_mode': connected_mode
        }]
    )
    
    # Return the launch description
    return LaunchDescription([
        declare_connected_mode,
        waypoint_manager_node
    ])
