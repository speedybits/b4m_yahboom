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
	bringup_dir = get_package_share_directory('nav2_bringup')
	nav_life_node = ['controller_node','planner_node','recoveries_node','bt_navigator_node','waypoint_follower_node']
	param_file = os.path.join(get_package_share_directory('yahboomcar_multi'),'param','robot2_nav_params.yaml')
	path_to_bt_xml_filename = DeclareLaunchArgument('params_file',default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),description='Full path to the ROS2 parameters file to use'),

	
	br_node = Node(
    	name="planner_server",
        package='yahboomcar_multi',
        executable='queue',
        output = "screen",
       
    )
    
	listen_node = Node(
    	name="controller_server",
    	package='yahboomcar_multi',
    	executable='listenline',
    	output = "screen",
     
    )
    
   
    
	return LaunchDescription([
    	#lifecycle_nodes,
    	#use_sim_time,\
    	#path_to_bt_xml_filename,
    	br_node,
    	listen_node,
    ])
    
    
