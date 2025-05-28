import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
	#package_path = get_package_share_directory('yahboomcar_multi')
	#nav2_bringup_dir = get_package_share_directory('nav2_bringup')
	lifecycle_nodes = ['map_server']
	map_file = os.path.join('/home/yahboom/yahboomcar_ws/src/yahboomcar_multi','maps','yahboom_map.yaml')
	map_node = Node(
    	name="map_server_node",
        package='nav2_map_server',
        executable='map_server',
        parameters=[{'use_sim_time':False},
        			{'yaml_filename':map_file}],
        output = "screen"
    )
    
	life_node = Node(
    	name="map_lifecycle_manager",
    	package='nav2_lifecycle_manager',
    	executable='lifecycle_manager',
    	output='screen',
    	parameters=[{'use_sim_time': False},{'autostart': True},{'node_names': ['map_server_node']}]
    	)
    
	return LaunchDescription([
    	#lifecycle_nodes,
    	#use_sim_time,
    	map_node,
    	life_node
    ])
    
    
