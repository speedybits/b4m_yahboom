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
	
	
	
	controller_node = Node(
    	name="controller_server",
        package='nav2_controller',
        executable='controller_server',
        parameters=[os.path.join(get_package_share_directory('yahboomcar_multi'),'param','robot2_nav_params.yaml')],
        remappings=[('/cmd_vel', '/robot2/cmd_vel'),('/robot2/map', '/map')],
        output = "screen",
        namespace = 'robot2'
    )
    
	planner_node = Node(
    	name="planner_server",
        package='nav2_planner',
        executable='planner_server',
        parameters=[os.path.join(get_package_share_directory('yahboomcar_multi'),'param','robot2_nav_params.yaml')],
        remappings=[('/robot2/map', '/map')],
        output = "screen",
        namespace = 'robot2'
    )
    
	recoveries_node = Node(
    	name="recoveries_server",
        package='nav2_recoveries',
        executable='recoveries_server',
        parameters=[os.path.join(get_package_share_directory('yahboomcar_multi'),'param','robot2_nav_params.yaml')],
        remappings=[('/cmd_vel', '/robot2/cmd_vel')],
        output = "screen",
        namespace = 'robot2'
    )
    
	bt_navigator_node = Node(
    	name="bt_navigator",
        package='nav2_bt_navigator',
        executable='bt_navigator',
        parameters=[os.path.join(get_package_share_directory('yahboomcar_multi'),'param','robot2_nav_params.yaml')],
        output = "screen",
        namespace = 'robot2',
        #remappings=[('/tf', 'tf'),('/tf_static', 'tf_static')]
    )
    
	waypoint_follower_node = Node(
    	name="waypoint_follower",
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        parameters=[os.path.join(get_package_share_directory('yahboomcar_multi'),'param','robot2_nav_params.yaml')],
        output = "screen",
        namespace = 'robot2',
        #remappings=[('/tf', 'tf'),('/tf_static', 'tf_static')]
    )
    
	life_node = Node(
    	name="nav_lifecycle_manager",
    	package='nav2_lifecycle_manager',
    	executable='lifecycle_manager',
    	output='screen',
    	parameters=[{'use_sim_time': False},{'autostart': True},{'node_names': ['/robot2/controller_server','/robot2/planner_server','/robot2/recoveries_server','/robot2/bt_navigator','/robot2/waypoint_follower']}],
    	#parameters=[{'use_sim_time': False},{'autostart': True},{'node_names': ['/robot2/controller_server','/robot2/waypoint_follower']}],
    	namespace = 'robot2'
    	)
    
	return LaunchDescription([
    	#lifecycle_nodes,
    	#use_sim_time,\
    	#path_to_bt_xml_filename,
    	controller_node,
    	planner_node,
    	recoveries_node,
    	bt_navigator_node,
    	waypoint_follower_node,
    	life_node
    ])
    
    
