# b4m_waypoint_nav: Waypoint Navigation with Orientation Plan

## Overview

This document outlines the plan for implementing a waypoint navigation system with orientation for the Yahboom robot. The system will allow users to:

1. Drive the robot using keyboard controls
2. Store waypoints with orientation at the robot's current position
3. Name and manage stored waypoints
4. Navigate to stored waypoints by name, with proper orientation upon arrival

The implementation will build upon the existing Navigation2 framework as described in the Lidar course documentation.

## System Architecture

### Components

1. **Waypoint Management System**
   - Store waypoints with position (x, y) and orientation (quaternion)
   - Persist waypoints to file for reuse across sessions
   - Provide interface for adding, listing, and selecting waypoints

2. **Keyboard Control Interface**
   - Drive the robot using keyboard commands based on the VM Remote control scheme:
     - 'i': Go forward
     - ',': Move back
     - 'l': Right rotation
     - 'j': Left rotation
     - 'u': Turn left
     - 'o': Turn right
     - 'm': Reverse left
     - '.': Reverse right
   - Trigger waypoint storage with a specific key ('s')
   - Select waypoints for navigation via keyboard input ('g')
   - List available waypoints ('p')
   - Delete waypoints ('d')

3. **Navigation Integration**
   - Leverage Navigation2 for path planning and obstacle avoidance
   - Send goal poses (position + orientation) to the navigation stack
   - Monitor navigation status and provide feedback
   - Visualize waypoints in RViz using marker arrays

### Data Structure

Each waypoint will be stored with the following information:
```
{
  "name": "string",       // User-defined name for the waypoint
  "position": {
    "x": float,           // X coordinate in map frame
    "y": float            // Y coordinate in map frame
  },
  "orientation": {
    "x": float,           // Quaternion x component
    "y": float,           // Quaternion y component
    "z": float,           // Quaternion z component
    "w": float            // Quaternion w component
  },
  "timestamp": string,    // When the waypoint was created
  "visualization": {      // Properties for RViz visualization
    "color": {           // RGB color for the marker
      "r": float,        // Red component (0.0-1.0)
      "g": float,        // Green component (0.0-1.0)
      "b": float         // Blue component (0.0-1.0)
    },
    "scale": float       // Size of the waypoint marker
  }
}
```

## Implementation Plan

### 1. Create Waypoint Navigation Node

Develop a ROS2 node (`b4m_waypoint_nav.py`) that will:

- Subscribe to robot pose topics to get current position and orientation
- Publish goal poses to the Navigation2 stack
- Implement waypoint management logic
- Handle keyboard input for robot control and waypoint operations

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, String
from ament_index_python.packages import get_package_share_directory
import json
import os
from datetime import datetime
import threading
import sys
import tty
import termios
import select
import random

class B4MWaypointNav(Node):
    def __init__(self):
        super().__init__('b4m_waypoint_nav')
        
        # Initialize waypoints storage
        self.waypoints = {}
        # Store waypoints in the repository root
        self.pkg_share = get_package_share_directory('yahboomcar_nav')
        self.repo_root = os.path.abspath(os.path.join(self.pkg_share, '..', '..'))
        self.waypoints_file = os.path.join(self.repo_root, 'waypoints.json')
        self.load_waypoints()
        
        # Create custom RViz configuration if it doesn't exist
        self.create_rviz_config()
        
        # Publishers and subscribers
        self.current_pose = None
        self.pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            'amcl_pose',
            self.pose_callback,
            10)
            
        # Navigation action client
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Waypoint visualization publisher
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            'waypoint_markers',
            10)
            
        # MQTT error message publisher
        self.mqtt_error_publisher = self.create_publisher(
            String,
            'mqtt_home_assistant/error',
            10)
        
        # Timer for publishing waypoint markers
        self.marker_timer = self.create_timer(1.0, self.publish_waypoint_markers)
        
        # Keyboard control thread
        self.keyboard_thread = threading.Thread(target=self.keyboard_control)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()
        
        self.get_logger().info('b4m_waypoint_nav node initialized')
        
    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose
        
    def load_waypoints(self):
        if os.path.exists(self.waypoints_file):
            try:
                with open(self.waypoints_file, 'r') as f:
                    self.waypoints = json.load(f)
                self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints')
            except Exception as e:
                self.get_logger().error(f'Failed to load waypoints: {e}')
        
    def save_waypoints(self):
        try:
            with open(self.waypoints_file, 'w') as f:
                json.dump(self.waypoints, f, indent=2)
            self.get_logger().info(f'Saved {len(self.waypoints)} waypoints')
        except Exception as e:
            self.get_logger().error(f'Failed to save waypoints: {e}')
    
    def store_current_waypoint(self, name):
        if self.current_pose is None:
            self.get_logger().warn('Cannot store waypoint: Current pose unknown')
            return False
            
        # Generate random color for visualization
        r = random.random()
        g = random.random()
        b = random.random()
            
        waypoint = {
            'name': name,
            'position': {
                'x': self.current_pose.position.x,
                'y': self.current_pose.position.y
            },
            'orientation': {
                'x': self.current_pose.orientation.x,
                'y': self.current_pose.orientation.y,
                'z': self.current_pose.orientation.z,
                'w': self.current_pose.orientation.w
            },
            'timestamp': datetime.now().isoformat(),
            'visualization': {
                'color': {
                    'r': r,
                    'g': g,
                    'b': b
                },
                'scale': 0.3
            }
        }
        
        self.waypoints[name] = waypoint
        self.save_waypoints()
        self.get_logger().info(f'Stored waypoint: {name}')
        return True
        
    def navigate_to_waypoint(self, name):
        if name not in self.waypoints:
            self.get_logger().warn(f'Waypoint not found: {name}')
            self.send_mqtt_error(f'Waypoint not found: {name}')
            return False
            
        waypoint = self.waypoints[name]
        
        # Create goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_pose.pose.position.x = waypoint['position']['x']
        goal_pose.pose.position.y = waypoint['position']['y']
        goal_pose.pose.position.z = 0.0
        
        goal_pose.pose.orientation.x = waypoint['orientation']['x']
        goal_pose.pose.orientation.y = waypoint['orientation']['y']
        goal_pose.pose.orientation.z = waypoint['orientation']['z']
        goal_pose.pose.orientation.w = waypoint['orientation']['w']
        
        # Wait for action server
        self.get_logger().info('Waiting for navigation action server...')
        try:
            server_available = self.nav_to_pose_client.wait_for_server(timeout_sec=5.0)
            if not server_available:
                self.get_logger().error('Navigation action server not available')
                self.send_mqtt_error('Navigation action server not available')
                return False
        except Exception as e:
            self.get_logger().error(f'Error waiting for navigation server: {str(e)}')
            self.send_mqtt_error(f'Navigation server error: {str(e)}')
            return False
        
        # Send goal
        self.get_logger().info(f'Navigating to waypoint: {name}')
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        
        try:
            self._send_goal_future = self.nav_to_pose_client.send_goal_async(
                goal_msg,
                feedback_callback=self.feedback_callback
            )
            
            self._send_goal_future.add_done_callback(self.goal_response_callback)
            return True
        except Exception as e:
            self.get_logger().error(f'Error sending navigation goal: {str(e)}')
            self.send_mqtt_error(f'Navigation goal error: {str(e)}')
            return False
        
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected')
            self.send_mqtt_error('Navigation goal rejected')
            return
            
        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
        
    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == 4:
            self.get_logger().info('Goal succeeded!')
            # Send success message to MQTT if needed
            # self.send_mqtt_message('Navigation completed successfully')
        else:
            self.get_logger().warn(f'Goal failed with status: {status}')
            self.send_mqtt_error(f'Navigation failed with status: {status}')
            
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # Process navigation feedback
        # Could publish progress updates to MQTT if desired
        
    def send_mqtt_error(self, error_message):
        """Send error message to Home Assistant via MQTT"""
        msg = String()
        msg.data = error_message
        self.mqtt_error_publisher.publish(msg)
        self.get_logger().info(f'Sent MQTT error: {error_message}')
        
    def list_waypoints(self):
        if not self.waypoints:
            self.get_logger().info('No waypoints stored')
            return
            
        self.get_logger().info('Stored waypoints:')
        for name in sorted(self.waypoints.keys()):
            wp = self.waypoints[name]
            self.get_logger().info(f"- {name}: pos({wp['position']['x']:.2f}, {wp['position']['y']:.2f})")
    
    def delete_waypoint(self, name):
        if name not in self.waypoints:
            self.get_logger().warn(f'Waypoint not found: {name}')
            return False
            
        del self.waypoints[name]
        self.save_waypoints()
        self.get_logger().info(f'Deleted waypoint: {name}')
        return True
        
    def publish_waypoint_markers(self):
        """Publish markers for all waypoints to visualize in RViz"""
        marker_array = MarkerArray()
        
        for i, (name, waypoint) in enumerate(self.waypoints.items()):
            # Create text marker for the waypoint name
            text_marker = Marker()
            text_marker.header.frame_id = 'map'
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = 'waypoint_names'
            text_marker.id = i
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            # Position slightly above the waypoint
            text_marker.pose.position.x = waypoint['position']['x']
            text_marker.pose.position.y = waypoint['position']['y']
            text_marker.pose.position.z = 0.5  # Slightly above the ground
            
            text_marker.pose.orientation.x = 0.0
            text_marker.pose.orientation.y = 0.0
            text_marker.pose.orientation.z = 0.0
            text_marker.pose.orientation.w = 1.0
            
            text_marker.scale.z = 0.3  # Text size
            
            # Use visualization color if defined, or default to white
            if 'visualization' in waypoint and 'color' in waypoint['visualization']:
                color = waypoint['visualization']['color']
                text_marker.color = ColorRGBA(r=color['r'], g=color['g'], b=color['b'], a=1.0)
            else:
                text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
                
            text_marker.text = name
            text_marker.lifetime.sec = 0  # Persistent
            
            # Create arrow marker for orientation
            arrow_marker = Marker()
            arrow_marker.header.frame_id = 'map'
            arrow_marker.header.stamp = self.get_clock().now().to_msg()
            arrow_marker.ns = 'waypoint_orientations'
            arrow_marker.id = i
            arrow_marker.type = Marker.ARROW
            arrow_marker.action = Marker.ADD
            
            # Set position and orientation
            arrow_marker.pose.position.x = waypoint['position']['x']
            arrow_marker.pose.position.y = waypoint['position']['y']
            arrow_marker.pose.position.z = 0.1  # Slightly above the ground
            
            arrow_marker.pose.orientation.x = waypoint['orientation']['x']
            arrow_marker.pose.orientation.y = waypoint['orientation']['y']
            arrow_marker.pose.orientation.z = waypoint['orientation']['z']
            arrow_marker.pose.orientation.w = waypoint['orientation']['w']
            
            # Arrow size
            arrow_marker.scale.x = 0.5  # Length
            arrow_marker.scale.y = 0.1  # Width
            arrow_marker.scale.z = 0.1  # Height
            
            # Use same color as text
            arrow_marker.color = text_marker.color
            arrow_marker.lifetime.sec = 0  # Persistent
            
            marker_array.markers.append(text_marker)
            marker_array.markers.append(arrow_marker)
        
        # Publish the marker array
        self.marker_publisher.publish(marker_array)
    
    def create_rviz_config(self):
        """Create a custom RViz configuration file if it doesn't exist"""
        # Get the package share directory
        pkg_share = get_package_share_directory('yahboomcar_nav')
        
        # Create rviz directory if it doesn't exist
        rviz_dir = os.path.join(pkg_share, 'rviz')
        if not os.path.exists(rviz_dir):
            os.makedirs(rviz_dir)
        
        # Path to the RViz configuration file
        rviz_config_file = os.path.join(rviz_dir, 'waypoint_nav.rviz')
        
        # Only create the file if it doesn't exist
        if not os.path.exists(rviz_config_file):
            # We'll use a basic configuration and add our marker display
            self.get_logger().info(f'Creating custom RViz configuration at {rviz_config_file}')
            
            # In a real implementation, we would either:
            # 1. Copy an existing RViz config and modify it
            # 2. Use the rviz Python API to create a configuration programmatically
            # 3. Write a basic config file with the necessary displays
            
            # Basic RViz configuration with waypoint markers
            rviz_config = '''
            Panels:  # Standard panels configuration
              - Class: rviz_common/Displays
                Name: Displays
            Displays:
              - Class: rviz_common/Group
                Name: Navigation
                Displays:
                  - Class: rviz_default_plugins/Map
                    Name: Map
                    Topic: /map
                    Value: true
                  - Class: rviz_default_plugins/Path
                    Name: Path
                    Topic: /plan
                    Value: true
              - Class: rviz_default_plugins/MarkerArray
                Name: Waypoint Markers
                Topic: /waypoint_markers
                Value: true
              - Class: rviz_default_plugins/TF
                Name: TF
                Value: true
            '''
            
            # Write the configuration to file
            with open(rviz_config_file, 'w') as f:
                f.write(rviz_config)
            
            self.get_logger().info(f'Created custom RViz configuration with waypoint markers display')
    
    def keyboard_control(self):
    # Keyboard control implementation
    self.get_logger().info("""
    Keyboard Control:
    ---------------------------
    Movement Controls:
    'i': Go forward
    ',': Move back
    'l': Right rotation
    'j': Left rotation
    'u': Turn left
    'o': Turn right
    'm': Reverse left
    '.': Reverse right
    
    Waypoint Controls:
    's': Store waypoint
    'g': Go to waypoint
    'p': List waypoints
    'd': Delete waypoint
    'q': Quit
    ---------------------------
    """)
    
    # Set terminal to raw mode to process key presses directly
    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setraw(sys.stdin.fileno())
        
        while True:
            # Check if key is pressed
            if select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.read(1)
                
                if key == 'q':  # Quit
                    self.get_logger().info('Exiting...')
                    break
                    
                elif key == 's':  # Store waypoint
                    # Reset terminal for input
                    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
                    print("\nEnter waypoint name: ", end='', flush=True)
                    name = input().strip()
                    if name:
                        self.store_current_waypoint(name)
                    # Set terminal back to raw mode
                    tty.setraw(sys.stdin.fileno())
                    
                elif key == 'g':  # Go to waypoint
                    # Reset terminal for input
                    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
                    print("\nEnter waypoint name to navigate to: ", end='', flush=True)
                    name = input().strip()
                    if name:
                        self.navigate_to_waypoint(name)
                    # Set terminal back to raw mode
                    tty.setraw(sys.stdin.fileno())
                    
                elif key == 'p':  # List waypoints
                    # Reset terminal for display
                    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
                    self.list_waypoints()
                    print("Press any key to continue...", end='', flush=True)
                    input()
                    # Set terminal back to raw mode
                    tty.setraw(sys.stdin.fileno())
                
                elif key == 'd':  # Delete waypoint
                    # Reset terminal for input
                    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
                    print("\nEnter waypoint name to delete: ", end='', flush=True)
                    name = input().strip()
                    if name:
                        self.delete_waypoint(name)
                    # Set terminal back to raw mode
                    tty.setraw(sys.stdin.fileno())
                
                # Movement controls based on VM Remote control scheme
                elif key == 'i':  # Go forward
                    # Send forward command
                    pass
                elif key == ',':  # Move back
                    # Send backward command
                    pass
                elif key == 'l':  # Right rotation
                    # Send right rotation command
                    pass
                elif key == 'j':  # Left rotation
                    # Send left rotation command
                    pass
                elif key == 'u':  # Turn left
                    # Send turn left command
                    pass
                elif key == 'o':  # Turn right
                    # Send turn right command
                    pass
                elif key == 'm':  # Reverse left
                    # Send reverse left command
                    pass
                elif key == '.':  # Reverse right
                    # Send reverse right command
                    pass
                
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def main(args=None):
    rclpy.init(args=args)
    node = B4MWaypointNav()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 3. Create Launch File

Create a launch file (`b4m_waypoint_nav_launch.py`) to start the waypoint navigation node along with the required Navigation2 components:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_dir = get_package_share_directory('yahboomcar_nav')
    
    # Define the path to the RViz configuration file
    rviz_config_file = os.path.join(package_dir, 'rviz', 'waypoint_nav.rviz')
    
    # Create a launch argument for using RViz
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz'
    )

    # Include the Navigation2 launch file
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('yahboomcar_nav'), 
                        'launch', 
                        'navigation_dwb_launch.py')
        ])
    )
    
    # Start the waypoint navigation node
    waypoint_navigation_node = Node(
        package='yahboomcar_nav',
        executable='b4m_waypoint_nav',
        name='b4m_waypoint_nav',
        output='screen'
    )
    
    # Launch RViz with our custom configuration if requested
    rviz_node = Node(
        condition=LaunchConfiguration('use_rviz'),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    
    return LaunchDescription([
        use_rviz_arg,
        nav2_launch,
        waypoint_navigation_node,
        rviz_node
    ])
```

Update the package's `setup.py` to include the new node:

```python
entry_points={
    'console_scripts': [
        'b4m_waypoint_nav = yahboomcar_nav.b4m_waypoint_nav:main',
    ],
},
```

## Usage Instructions

1. **Start the System**
   ```bash
   # Terminal 1: Start the car's base functionality
   ros2 launch yahboomcar_bringup yahboomcar_bringup_launch.py
   
   # Terminal 2: Start the waypoint navigation system
   ros2 launch yahboomcar_nav b4m_waypoint_nav_launch.py
   ```

2. **Control the Robot**
   - Use the following keys to drive the robot:
     - 'i': Go forward
     - ',': Move back
     - 'l': Right rotation
     - 'j': Left rotation
     - 'u': Turn left
     - 'o': Turn right
     - 'm': Reverse left
     - '.': Reverse right
   - Press 's' to store a waypoint at the current position
   - Enter a name for the waypoint when prompted
   - Press 'p' to list all stored waypoints
   - Press 'g' to navigate to a waypoint
   - Enter the name of the waypoint to navigate to
   - Press 'd' to delete a waypoint
   - Enter the name of the waypoint to delete

3. **Navigation Process**
   - The robot will plan a path to the selected waypoint
   - Navigation2 will handle obstacle avoidance
   - Upon arrival, the robot will orient itself according to the stored orientation

4. **Waypoint Visualization**
   - Waypoints are visualized in RViz as text labels with their names
   - Arrows show the orientation of each waypoint
   - Each waypoint has a unique color for easy identification

## Future Enhancements

1. **Waypoint Management UI**
   - Develop a graphical interface for waypoint management
   - Visualize waypoints on a map
   - Allow drag-and-drop waypoint creation and editing

2. **Waypoint Sequences**
   - Define and execute sequences of waypoints
   - Create patrol routes or guided tours

3. **Simulation Integration**
   - Adapt the system to work in Gazebo simulation
   - Share waypoints between physical robot and simulation
   - Use the same waypoint file for both environments
   - Add configuration option to specify different waypoint files if needed

4. **Enhanced Waypoint Features**
   - Add actions to perform at waypoints (e.g., wait time, sensor readings)
   - Support different arrival behaviors (e.g., approach direction)
   - Add waypoint categories or tags for organization

## Conclusion

This plan provides a comprehensive approach to implementing waypoint navigation with orientation for the Yahboom robot. By leveraging the Navigation2 framework and adding custom waypoint management features, we can create a flexible and user-friendly system for robot navigation.
