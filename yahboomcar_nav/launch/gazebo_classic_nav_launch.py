import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directories
    nav_package_path = get_package_share_directory('yahboomcar_nav')
    desc_package_path = get_package_share_directory('yahboomcar_description')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    verbose = LaunchConfiguration('verbose')
    world_name = LaunchConfiguration('world_name')
    
    # World file - dynamic selection based on world_name parameter
    world_file = PathJoinSubstitution([
        nav_package_path, 
        'worlds', 
        [world_name, '.world']
    ])
    
    # Robot description  
    urdf_file = os.path.join(desc_package_path, 'urdf', 'yahboomcar_robot_classic_nav.urdf')
    
    # Read URDF content
    with open(urdf_file, 'r') as urdf_file_handle:
        robot_description_content = urdf_file_handle.read()
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'verbose',
            default_value='true',
            description='Enable verbose output'
        ),
        DeclareLaunchArgument(
            'world_name',
            default_value='navigation_test_classic',
            description='Name of the world file (without .world extension)'
        ),
        
        # Launch Gazebo Classic with the selected world
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', 
                 '-s', 'libgazebo_ros_factory.so', world_file],
            output='screen',
            shell=False
        ),
        
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace='',
            output='screen',
            parameters=[
                {'robot_description': robot_description_content},
                {'use_sim_time': use_sim_time}
            ],
        ),
        
        # Joint State Publisher (for visualization)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            namespace='',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time}
            ],
        ),
        
        # Spawn robot in Gazebo Classic
        TimerAction(
            period=5.0,  # Wait for Gazebo to fully start
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=[
                        '-entity', 'yahboomcar',
                        '-topic', 'robot_description',
                        '-x', '0', '-y', '0', '-z', '0.1'
                    ],
                    output='screen'
                ),
            ]
        ),
        
        # Static transform publisher for laser frame (if needed for compatibility)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_tf_publisher',
            output='screen',
            arguments=['0.04', '0', '0.05', '0', '0', '0', 'base_link', 'laser'],
            parameters=[
                {'use_sim_time': use_sim_time}
            ],
        ),
    ])