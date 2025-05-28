import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    package_path = get_package_share_directory('yahboomcar_nav')
    nav2_bringup_dir = get_package_share_directory('yahboomcar_multi')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    namespace = LaunchConfiguration('namespace', default='robot1')
    #name = PythonExpression(['str(',namespace, ')'])
    robot_name = PythonExpression(["'robot1' if ", namespace, "==robot1 else 'robot_2'"])
    print("namespace: ",robot_name)
    map_yaml_path = LaunchConfiguration(
        'maps', default=os.path.join(package_path, 'maps', 'yahboom_map.yaml')) 
    nav2_param_path = LaunchConfiguration('params_file', default=os.path.join(
        package_path, 'params', 'robot1_nav_params.yaml'))
    #use_sim_time = LaunchConfiguration('namespace', default='robot1')
    use_namespace = LaunchConfiguration('use_namespace', default='true')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value=use_sim_time,
                              description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('namespace', default_value=namespace,
                              description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('maps', default_value=map_yaml_path,
                              description='Full path to map file to load'),
        DeclareLaunchArgument('params_file', default_value=nav2_param_path,
                              description='Full path to param file to load'),
        DeclareLaunchArgument('namespace', default_value=namespace,
                              description='Full path to param file to load'),
        DeclareLaunchArgument('use_namespace', default_value=use_namespace,
                              description='Full path to param file to load'),                        

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_bringup_dir, '/launch', '/bringup_launch.py']),
            launch_arguments={
                'map': map_yaml_path,
                'use_sim_time': use_sim_time,
                'namespace': namespace,
                'params_file': nav2_param_path,
                'use_namespace': use_namespace,
                'namespace': namespace}.items(),
        ),
        Node(
            package='tf2_ros',
            namespace = 'robot1',
            executable='static_transform_publisher',
            name='base_link_to_base_laser',
            arguments=['-0.0046412', '0' , '0.094079','0','0','0','robot1/base_link','robot1/laser_frame']
        ),
        Node(
            package='yahboomcar_nav',
            executable='stop_robot1_car',
            namespace = 'robot1'
        ) 
    ])
