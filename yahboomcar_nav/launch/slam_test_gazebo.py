import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Get package directories
    nav_package_path = get_package_share_directory('yahboomcar_nav')
    desc_package_path = get_package_share_directory('yahboomcar_description')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file = os.path.join(nav_package_path, 'worlds', 'slam_test_world.sdf')
    
    # Robot description
    urdf_file = os.path.join(desc_package_path, 'urdf', 'yahboomcar_robot2_gazebo.urdf')
    
    # SLAM parameters file
    slam_params_file = os.path.join(nav_package_path, 'params', 'slam_params_gazebo.yaml')
    
    return LaunchDescription([
        # Include Gazebo with test world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
            ]),
            launch_arguments={
                'gz_args': ['-r ', world_file]
            }.items()
        ),
        
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time,
                 'robot_description': open(urdf_file).read()}
            ],
        ),
        
        # Spawn robot in Gazebo
        Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_yahboomcar',
            arguments=[
                '-name', 'yahboomcar',
                '-topic', '/robot_description',
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.1'
            ],
            output='screen',
        ),
        
        # ROS-Gazebo Bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='parameter_bridge',
            arguments=[
                '/model/yahboomcar/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/model/yahboomcar/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                '/model/yahboomcar/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
                '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock'
            ],
            remappings=[
                ('/model/yahboomcar/cmd_vel', '/cmd_vel'),
                ('/model/yahboomcar/odometry', '/odom'),
                ('/model/yahboomcar/tf', '/tf')
            ],
            output='screen',
        ),
        
        # Joint State Publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        
        # SLAM Toolbox Node
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
    ])