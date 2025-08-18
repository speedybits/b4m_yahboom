#!/usr/bin/env python3
"""
Launch file for B4M LLM Navigation API

Starts the FastAPI server node with configurable parameters for
real robot operation, simulated B4M mode, or full Gazebo simulation.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for B4M LLM Navigation API"""
    
    # Declare launch arguments
    simulated_b4m_arg = DeclareLaunchArgument(
        'simulated_b4m',
        default_value='false',
        description='Enable simulated B4M mode (no real LLM calls)'
    )
    
    verbose_logging_arg = DeclareLaunchArgument(
        'verbose_logging',
        default_value='true',
        description='Enable verbose console logging'
    )
    
    grid_display_mode_arg = DeclareLaunchArgument(
        'grid_display_mode',
        default_value='standard',
        description='Grid display mode: compact, standard, detailed'
    )
    
    api_port_arg = DeclareLaunchArgument(
        'api_port',
        default_value='8080',
        description='FastAPI server port'
    )
    
    def launch_setup(context, *args, **kwargs):
        """Setup launch nodes with runtime configuration"""
        
        # Get launch configuration values
        simulated_b4m = LaunchConfiguration('simulated_b4m').perform(context)
        verbose_logging = LaunchConfiguration('verbose_logging').perform(context)
        grid_display_mode = LaunchConfiguration('grid_display_mode').perform(context)
        api_port = LaunchConfiguration('api_port').perform(context)
        
        # Configuration file path
        config_file = PathJoinSubstitution([
            FindPackageShare('b4m_llm_nav_api'),
            'config',
            'llm_nav_api.yaml'
        ])
        
        # API server node
        api_server_node = Node(
            package='b4m_llm_nav_api',
            executable='api_server.py',
            name='b4m_llm_nav_api_server',
            output='screen',
            parameters=[
                config_file,
                {
                    'simulated_b4m.enabled': simulated_b4m.lower() == 'true',
                    'console_output.show_decision_reasoning': verbose_logging.lower() == 'true',
                    'console_output.grid_display_mode': grid_display_mode,
                    'api_server.port': int(api_port)
                }
            ],
            emulate_tty=True,  # Enable color output in console
        )
        
        return [api_server_node]
    
    return LaunchDescription([
        simulated_b4m_arg,
        verbose_logging_arg, 
        grid_display_mode_arg,
        api_port_arg,
        OpaqueFunction(function=launch_setup)
    ])