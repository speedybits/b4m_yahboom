#!/usr/bin/env python3
"""
B4M LiDAR Navigator Launch File

This launch file starts the B4M LiDAR-based intelligent navigation system.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    debug_mode_arg = DeclareLaunchArgument(
        'debug_mode',
        default_value='false',
        description='Enable debug logging'
    )
    
    enable_api_arg = DeclareLaunchArgument(
        'enable_api',
        default_value='true',
        description='Enable B4M API for turn decisions'
    )
    
    api_cooldown_arg = DeclareLaunchArgument(
        'api_cooldown',
        default_value='20.0',
        description='Cooldown time between API calls (seconds)'
    )
    
    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    debug_mode = LaunchConfiguration('debug_mode')
    enable_api = LaunchConfiguration('enable_api')
    api_cooldown = LaunchConfiguration('api_cooldown')
    
    # B4M LiDAR Navigator node
    b4m_lidar_navigator = Node(
        package='b4m_lidar',
        executable='b4m_lidar_navigator',
        name='b4m_lidar_navigator',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'debug_mode': debug_mode,
            'enable_api': enable_api,
            'api_cooldown': api_cooldown,
            'api_endpoint': 'https://app.bike4mind.com/api/chat',
            'api_key': 'b4m_live_c491719bd23cc716e2db2c5182f4f900',
            'api_timeout': 2.0,
            'linear_speed': 0.08,
            'angular_speed': 0.3,
            'stop_distance': 0.3048,
            'safe_distance': 0.4,
            'min_turn_time': 1.0,
            'required_clear_readings': 3,
        }],
        remappings=[
            ('/scan', '/scan'),
            ('/cmd_vel', '/cmd_vel'),
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        debug_mode_arg,
        enable_api_arg,
        api_cooldown_arg,
        b4m_lidar_navigator,
    ])