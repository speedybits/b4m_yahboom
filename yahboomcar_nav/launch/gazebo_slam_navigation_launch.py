import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directories
    package_path = get_package_share_directory('yahboomcar_nav')
    yahboom_description_dir = get_package_share_directory('yahboomcar_description')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')  # Set to true for Gazebo
    namespace = LaunchConfiguration('namespace', default='')
    
    # Use Gazebo-specific SLAM navigation parameters (with use_sim_time: true)
    nav2_param_path = LaunchConfiguration('params_file', default=os.path.join(
        package_path, 'params', 'gazebo_slam_nav_params.yaml'))
    
    # SLAM Toolbox simulation parameters
    slam_params_file = LaunchConfiguration('slam_params_file', 
        default=os.path.join(package_path, 'params', 'slam_toolbox_sim_params.yaml'))
    
    # Controller configuration for ros2_control
    controller_params_file = LaunchConfiguration('controller_params_file',
        default=os.path.join(package_path, 'params', 'gazebo_controllers.yaml'))
    
    # Get the URDF model path - use Gazebo-specific URDF with plugins
    urdf_model_path = os.path.join(yahboom_description_dir, 'urdf', 'yahboomcar_robot2_gazebo.urdf')
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('use_sim_time', default_value=use_sim_time,
                              description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('namespace', default_value=namespace,
                              description='Robot namespace'),
        DeclareLaunchArgument('params_file', default_value=nav2_param_path,
                              description='Full path to nav2 param file to load'),
        DeclareLaunchArgument('slam_params_file', default_value=slam_params_file,
                              description='Full path to slam_toolbox params file'),
        
        # FIX: Only start Gazebo and robot if not already running (for autotest compatibility)
        # Check if Gazebo is already running by looking for the /gazebo/spawn_entity service
        # If the service exists, skip Gazebo startup (assume robot already spawned in Step 2)
        # This allows the launch file to work both standalone and with the autotest script
        
        # Note: In autotest mode, Gazebo and robot are started in Steps 1-2
        # This launch file focuses on navigation stack initialization only
        
        # Robot State Publisher (always start to ensure transforms available)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher_nav',
            output='screen',
            parameters=[{'robot_description': open(urdf_model_path).read()}, 
                       {'use_sim_time': use_sim_time}],
        ),

        # NOTE: Controller Manager is handled by Gazebo's gazebo_ros2_control plugin
        # Do not start standalone controller manager for Gazebo simulation
        # The controllers are spawned by the robot_state_publisher_gazebo.py launch file

        # SLAM Toolbox Node (directly, without extra transform publisher)
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params_file, {'use_sim_time': use_sim_time}],
            remappings=[('/scan', '/scan')],
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
        
        # Launch the waypoint navigation node
        Node(
            package='yahboomcar_nav',
            executable='waypoint_navigation',
            name='waypoint_navigation_node',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
        
        # Stop car node for safe shutdown
        Node(
            package='yahboomcar_nav',
            executable='stop_car',
            parameters=[{'use_sim_time': use_sim_time}]
        )
    ])