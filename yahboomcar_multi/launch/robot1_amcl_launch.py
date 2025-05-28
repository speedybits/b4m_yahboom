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
	param_file = os.path.join(get_package_share_directory('yahboomcar_multi'),'param','robot1_amcl_params.yaml')
	amcl_node = Node(
    	name="robot1_amcl",
        package='nav2_amcl',
        executable='amcl',
        parameters=[os.path.join(get_package_share_directory('yahboomcar_multi'),'param','robot1_amcl_params.yaml')],
        remappings=[('/initialpose', '/robot1/initialpose')],
        output = "screen"
    )
    
	life_node = Node(
    	name="robot1_amcl_lifecycle_manager",
    	package='nav2_lifecycle_manager',
    	executable='lifecycle_manager',
    	output='screen',
    	parameters=[{'use_sim_time': False},{'autostart': True},{'node_names': ['robot1_amcl']}]
    	)
    	
	base_link_to_laser_tf_node = Node(
    	package='tf2_ros',
    	executable='static_transform_publisher',
    	name='base_link_to_base_laser',
    	namespace = 'robot1',
    	arguments=['-0.0046412', '0' , '0.094079','0','0','0','robot1/base_link','robot1/laser_frame']
    )
    
	return LaunchDescription([
    	#lifecycle_nodes,
    	#use_sim_time,\
    	amcl_node,
    	life_node,
    	base_link_to_laser_tf_node
    ])
    
    
