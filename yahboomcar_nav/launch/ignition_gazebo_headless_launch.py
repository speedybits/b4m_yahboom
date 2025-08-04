import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Get package directory
    package_path = get_package_share_directory('yahboomcar_nav')
    
    # World file path
    world_file = os.path.join('/home/mike/projects/b4m_yahboom/worlds', 'empty.sdf')
    
    # Launch configuration
    verbose = LaunchConfiguration('verbose', default='1')
    
    return LaunchDescription([
        # Set environment for Ignition Gazebo
        SetEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH',
            value=os.path.dirname(world_file)
        ),
        
        # Launch Ignition Gazebo headless with auto-start
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-v', verbose, '-r', '-s', '--headless-rendering', world_file],
            output='screen',
            shell=False
        ),
    ])