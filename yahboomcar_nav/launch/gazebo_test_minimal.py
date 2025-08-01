import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directories
    yahboom_description_dir = get_package_share_directory('yahboomcar_description')
    package_path = get_package_share_directory('yahboomcar_nav')
    
    # URDF file path
    urdf_model_path = os.path.join(yahboom_description_dir, 'urdf', 'yahboomcar_robot2_gazebo.urdf')
    
    # Controller configuration
    controller_params_file = os.path.join(package_path, 'params', 'gazebo_controllers.yaml')
    
    return LaunchDescription([
        # Start Gazebo server
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        
        # Spawn the robot in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'yahboomcar', '-file', urdf_model_path],
            output='screen'
        ),
        
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_model_path).read()}, 
                       {'use_sim_time': True}],
        ),

        # Controller Manager
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[controller_params_file, {'use_sim_time': True}],
            output='screen',
        ),

        # Joint State Broadcaster
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen',
        ),

        # Differential Drive Controller
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_drive_controller'],
            output='screen',
        ),
    ])