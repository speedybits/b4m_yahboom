#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
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
        self.pkg_share = get_package_share_directory('b4m_waypoint_nav')
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
            
        # Velocity publisher for keyboard control
        self.vel_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10)
        
        # Movement speed settings
        self.linear_speed = 0.2  # m/s
        self.angular_speed = 1.0  # rad/s
        self.linear_speed_limit = 1.0  # m/s
        self.angular_speed_limit = 5.0  # rad/s
        self.speed_increment = 0.1  # Speed adjustment increment
        
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
        if result.result == 1:  # SUCCEEDED
            self.get_logger().info('Navigation succeeded!')
            # Publish MQTT message for successful arrival
            mqtt_msg = String()
            mqtt_msg.data = f"Navigation to waypoint succeeded at {datetime.now().isoformat()}"
            self.mqtt_error_publisher.publish(mqtt_msg)
        elif result.result == 2:  # CANCELED
            self.get_logger().info('Navigation was canceled')
            # Publish MQTT message for navigation canceled
            mqtt_msg = String()
            mqtt_msg.data = f"Navigation canceled at {datetime.now().isoformat()}"
            self.mqtt_error_publisher.publish(mqtt_msg)
        elif result.result == 3:  # FAILED
            self.get_logger().info('Navigation failed!')
            self.send_mqtt_error("Navigation failed")
        else:
            self.get_logger().info(f'Navigation returned with unknown result code: {result.result}')
            self.send_mqtt_error(f"Navigation returned unknown result: {result.result}")
            
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
        pkg_share = get_package_share_directory('b4m_waypoint_nav')
        
        # Create rviz directory if it doesn't exist
        rviz_dir = os.path.join(pkg_share, 'rviz')
        if not os.path.exists(rviz_dir):
            os.makedirs(rviz_dir)
        
        # Path to the RViz configuration file
        rviz_config_file = os.path.join(rviz_dir, 'b4m_waypoint_nav.rviz')
        
        # Only create the file if it doesn't exist
        if not os.path.exists(rviz_config_file):
            # We'll use a basic configuration and add our marker display
            self.get_logger().info(f'Creating custom RViz configuration at {rviz_config_file}')
            
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
        ' ': Stop
        
        Speed Controls:
        'q': Increase both speeds
        'z': Decrease both speeds
        'w': Increase linear speed
        'x': Decrease linear speed
        'e': Increase angular speed
        'c': Decrease angular speed
        
        Waypoint Controls:
        's': Store waypoint
        'g': Go to waypoint
        'p': List waypoints
        'c': Cancel navigation
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
                        
                    elif key == 'c':  # Cancel navigation
                        # Reset terminal for display
                        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
                        self.get_logger().info('Canceling navigation...')
                        # Cancel the navigation goal if one is active
                        if hasattr(self, '_send_goal_future') and self._send_goal_future is not None:
                            try:
                                goal_handle = self._send_goal_future.result()
                                if goal_handle is not None and goal_handle.accepted:
                                    self.get_logger().info('Canceling goal...')
                                    goal_handle.cancel_goal_async()
                                    self.get_logger().info('Goal canceled')
                            except Exception as e:
                                self.get_logger().error(f'Error canceling navigation: {str(e)}')
                        print("Navigation canceled. Press any key to continue...", end='', flush=True)
                        input()
                        # Set terminal back to raw mode
                        tty.setraw(sys.stdin.fileno())
                    
                    # Movement controls based on VM Remote control scheme
                    elif key == 'i':  # Go forward
                        twist = Twist()
                        twist.linear.x = self.linear_speed
                        twist.angular.z = 0.0
                        self.vel_publisher.publish(twist)
                        self.get_logger().info('Moving forward')
                    elif key == ',':  # Move back
                        twist = Twist()
                        twist.linear.x = -self.linear_speed
                        twist.angular.z = 0.0
                        self.vel_publisher.publish(twist)
                        self.get_logger().info('Moving backward')
                    elif key == 'l':  # Right rotation
                        twist = Twist()
                        twist.linear.x = 0.0
                        twist.angular.z = -self.angular_speed
                        self.vel_publisher.publish(twist)
                        self.get_logger().info('Rotating right')
                    elif key == 'j':  # Left rotation
                        twist = Twist()
                        twist.linear.x = 0.0
                        twist.angular.z = self.angular_speed
                        self.vel_publisher.publish(twist)
                        self.get_logger().info('Rotating left')
                    elif key == 'u':  # Turn left
                        twist = Twist()
                        twist.linear.x = self.linear_speed
                        twist.angular.z = self.angular_speed
                        self.vel_publisher.publish(twist)
                        self.get_logger().info('Turning left')
                    elif key == 'o':  # Turn right
                        twist = Twist()
                        twist.linear.x = self.linear_speed
                        twist.angular.z = -self.angular_speed
                        self.vel_publisher.publish(twist)
                        self.get_logger().info('Turning right')
                    elif key == 'm':  # Reverse left
                        twist = Twist()
                        twist.linear.x = -self.linear_speed
                        twist.angular.z = self.angular_speed
                        self.vel_publisher.publish(twist)
                        self.get_logger().info('Reverse left')
                    elif key == '.':  # Reverse right
                        twist = Twist()
                        twist.linear.x = -self.linear_speed
                        twist.angular.z = -self.angular_speed
                        self.vel_publisher.publish(twist)
                        self.get_logger().info('Reverse right')
                    elif key == ' ':  # Stop
                        twist = Twist()
                        twist.linear.x = 0.0
                        twist.angular.z = 0.0
                        self.vel_publisher.publish(twist)
                        self.get_logger().info('Stopping')
                        
                    # Speed adjustment commands
                    elif key == 'q':  # Increase both speeds
                        self.linear_speed = min(self.linear_speed + self.speed_increment, self.linear_speed_limit)
                        self.angular_speed = min(self.angular_speed + self.speed_increment, self.angular_speed_limit)
                        self.get_logger().info(f'Speed increased to linear: {self.linear_speed:.2f}, angular: {self.angular_speed:.2f}')
                    elif key == 'z':  # Decrease both speeds
                        self.linear_speed = max(self.linear_speed - self.speed_increment, 0.1)
                        self.angular_speed = max(self.angular_speed - self.speed_increment, 0.1)
                        self.get_logger().info(f'Speed decreased to linear: {self.linear_speed:.2f}, angular: {self.angular_speed:.2f}')
                    elif key == 'w':  # Increase linear speed
                        self.linear_speed = min(self.linear_speed + self.speed_increment, self.linear_speed_limit)
                        self.get_logger().info(f'Linear speed increased to {self.linear_speed:.2f}')
                    elif key == 'x':  # Decrease linear speed
                        self.linear_speed = max(self.linear_speed - self.speed_increment, 0.1)
                        self.get_logger().info(f'Linear speed decreased to {self.linear_speed:.2f}')
                    elif key == 'e':  # Increase angular speed
                        self.angular_speed = min(self.angular_speed + self.speed_increment, self.angular_speed_limit)
                        self.get_logger().info(f'Angular speed increased to {self.angular_speed:.2f}')
                    elif key == 'c':  # Decrease angular speed
                        self.angular_speed = max(self.angular_speed - self.speed_increment, 0.1)
                        self.get_logger().info(f'Angular speed decreased to {self.angular_speed:.2f}')
                    
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
