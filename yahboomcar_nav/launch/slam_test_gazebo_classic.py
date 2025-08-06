import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directories
    nav_package_path = get_package_share_directory('yahboomcar_nav')
    desc_package_path = get_package_share_directory('yahboomcar_description')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # World file
    world_file = os.path.join(nav_package_path, 'worlds', 'slam_test_classic.world')
    
    # Robot description  
    urdf_file = os.path.join(desc_package_path, 'urdf', 'yahboomcar_robot_classic_slam.urdf')
    
    # SLAM parameters file
    slam_params_file = os.path.join(nav_package_path, 'params', 'slam_params_gazebo.yaml')
    
    # Read URDF content
    with open(urdf_file, 'r') as urdf_file_handle:
        robot_description_content = urdf_file_handle.read()
    
    return LaunchDescription([
        # Launch Gazebo Classic with the SLAM test world
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
        
        # SLAM Toolbox Node
        TimerAction(
            period=10.0,  # Wait for robot to spawn and stabilize
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