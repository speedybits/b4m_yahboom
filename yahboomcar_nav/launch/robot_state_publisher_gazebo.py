import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    # Get package directories
    yahboom_description_dir = get_package_share_directory('yahboomcar_description')
    yahboom_nav_dir = get_package_share_directory('yahboomcar_nav')
    
    # URDF file path
    urdf_model_path = os.path.join(yahboom_description_dir, 'urdf', 'yahboomcar_robot2_gazebo.urdf')
    
    # Controller configuration
    controller_params_file = os.path.join(yahboom_nav_dir, 'params', 'gazebo_controllers.yaml')
    
    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_model_path).read()}, 
                       {'use_sim_time': True}],
        ),


        # Joint State Broadcaster (with delay to allow controller manager to initialize)
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
                    output='screen',
                ),
            ]
        ),

        # Differential Drive Controller (with delay to allow controller manager to initialize)
        TimerAction(
            period=7.0,
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