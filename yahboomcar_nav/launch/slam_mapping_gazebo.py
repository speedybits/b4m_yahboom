import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directory
    package_path = get_package_share_directory('yahboomcar_nav')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # SLAM parameters file
    slam_params_file = os.path.join(package_path, 'params', 'slam_params_gazebo.yaml')
    
    return LaunchDescription([
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