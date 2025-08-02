import os
import shlex
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directories
    package_path = get_package_share_directory('yahboomcar_nav')
    yahboom_description_dir = get_package_share_directory('yahboomcar_description')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Controller configuration
    controller_params_file = os.path.join(package_path, 'params', 'gazebo_controllers.yaml')
    
    # Get the URDF model path
    urdf_model_path = os.path.join(yahboom_description_dir, 'urdf', 'yahboomcar_robot2_gazebo.urdf')
    
    # Read URDF content
    with open(urdf_model_path, 'r') as urdf_file:
        robot_description_content = urdf_file.read()
    
    return LaunchDescription([
        # FIX: Start Robot State Publisher first so gazebo_ros2_control plugin can find robot_description
        # Publish to global namespace (empty namespace) for gazebo_ros2_control to find
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace='',  # Global namespace
            output='screen',
            parameters=[
                {'robot_description': robot_description_content},
                {'use_sim_time': use_sim_time}
            ],
        ),
        
        # Second: Spawn the robot in Gazebo - back to using file which works
        # The gazebo_ros2_control plugin will find robot_description from robot_state_publisher
        TimerAction(
            period=2.0,  # Wait 2 seconds for robot_state_publisher to start
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                         '-entity', 'yahboomcar',
                         '-file', urdf_model_path,
                         '-x', '0', '-y', '0', '-z', '0.1'],
                    output='screen'
                ),
            ]
        ),
        
        # Fourth: Spawn controllers after gazebo_ros2_control plugin initializes controller manager
        TimerAction(
            period=6.0,  # Wait for robot to be spawned and gazebo plugin to initialize controller manager
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
                    output='screen',
                ),
            ]
        ),
        
        TimerAction(
            period=8.0,  # Additional delay for second controller
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
                    output='screen',
                ),
            ]
        ),
    ])