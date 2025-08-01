import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_path = get_package_share_directory('yahboomcar_nav')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    namespace = LaunchConfiguration('namespace', default='')
    
    # Use slam_toolbox navigation parameters (modified to exclude AMCL)
    nav2_param_path = LaunchConfiguration('params_file', default=os.path.join(
        package_path, 'params', 'slam_nav_params.yaml'))
    
    # SLAM Toolbox parameters
    slam_params_file = LaunchConfiguration('slam_params_file', 
        default=os.path.join(package_path, 'params', 'slam_toolbox_params.yaml'))

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value=use_sim_time,
                              description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('namespace', default_value=namespace,
                              description='Robot namespace'),
        DeclareLaunchArgument('params_file', default_value=nav2_param_path,
                              description='Full path to nav2 param file to load'),
        DeclareLaunchArgument('slam_params_file', default_value=slam_params_file,
                              description='Full path to slam_toolbox params file'),

        # SLAM Toolbox Launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [package_path, '/launch', '/slam_toolbox_launch.py']),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'slam_params_file': slam_params_file}.items(),
        ),

        # Nav2 Launch (without map_server and AMCL)
        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[nav2_param_path, {'use_sim_time': use_sim_time}],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_param_path, {'use_sim_time': use_sim_time}],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        ),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[nav2_param_path, {'use_sim_time': use_sim_time}],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_param_path, {'use_sim_time': use_sim_time}],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        ),

        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[nav2_param_path, {'use_sim_time': use_sim_time}],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        ),

        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[nav2_param_path, {'use_sim_time': use_sim_time}],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static'),
                        ('/cmd_vel', 'cmd_vel_nav'), ('/cmd_vel_smoothed', 'cmd_vel')],
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': ['controller_server',
                                       'planner_server',
                                       'behavior_server',
                                       'bt_navigator',
                                       'waypoint_follower',
                                       'velocity_smoother']}],
        ),
        
        # Waypoint navigation node
        Node(
            package='yahboomcar_nav',
            executable='waypoint_navigation',
            name='waypoint_navigation_node',
            output='screen'
        ),
        
        # Stop car node for safe shutdown
        Node(
            package='yahboomcar_nav',
            executable='stop_car'
        )
    ])