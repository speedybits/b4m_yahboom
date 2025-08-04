import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
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
        # Start Robot State Publisher for gz_ros2_control
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
        
        # ROS-Ignition Bridge for cmd_vel and odometry
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
                '/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
                '/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V'
            ],
            output='screen',
            remappings=[
                ('/tf', '/tf')
            ]
        ),
        
        # Spawn the robot in Ignition Gazebo using ros_gz_sim
        TimerAction(
            period=2.0,  # Wait for robot_state_publisher
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    arguments=[
                        '-name', 'yahboomcar',
                        '-file', urdf_model_path,
                        '-x', '0', '-y', '0', '-z', '0'
                    ],
                    output='screen'
                ),
            ]
        ),
        
        # NOTE: Removed ros2_control spawners since we're using direct DiffDrive plugin
        # The URDF contains a direct ignition::gazebo::systems::DiffDrive plugin
        # which doesn't need controller_manager spawners
    ])