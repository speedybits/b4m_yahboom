import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directories
    yahboom_description_dir = get_package_share_directory('yahboomcar_description')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Get the URDF model path
    urdf_model_path = os.path.join(yahboom_description_dir, 'urdf', 'yahboomcar_robot2_gazebo.urdf')
    
    # Read URDF content
    with open(urdf_model_path, 'r') as urdf_file:
        robot_description_content = urdf_file.read()
    
    return LaunchDescription([
        # Robot State Publisher for transforms
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
        
        # Spawn the robot in Ignition Gazebo
        TimerAction(
            period=2.0,  # Wait for robot_state_publisher
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    arguments=[
                        '-name', 'yahboomcar',
                        '-topic', '/robot_description',
                        '-x', '0', '-y', '0', '-z', '0'
                    ],
                    output='screen'
                ),
            ]
        ),
        
        # SLAM FOCUS: No controller spawners - using direct differential drive plugin
        # Robot will respond to /cmd_vel commands via the DiffDrive plugin
        # Odometry will be published to /odom for SLAM
        
        # Add ROS-Gazebo bridge for robot control and odometry
        TimerAction(
            period=4.0,  # Give robot time to spawn
            actions=[
                Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    arguments=[
                        '/model/yahboomcar/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                        '/model/yahboomcar/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                        '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                        '/model/yahboomcar/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
                        '--ros-args', '-p', 'use_sim_time:=true'
                    ],
                    remappings=[
                        ('/model/yahboomcar/cmd_vel', '/cmd_vel'),
                        ('/model/yahboomcar/odometry', '/odom')
                    ],
                    output='screen'
                ),
                
                # Add joint state publisher for RViz visualization
                Node(
                    package='joint_state_publisher',
                    executable='joint_state_publisher',
                    name='joint_state_publisher',
                    output='screen',
                    parameters=[{'use_sim_time': use_sim_time}]
                ),
            ]
        ),
    ])