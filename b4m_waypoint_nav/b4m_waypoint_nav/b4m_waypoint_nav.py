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
import paho.mqtt.client as mqtt
import logging

class B4MWaypointNav(Node):
    """Waypoint Navigation Node for Yahboom robot
    
    This node provides waypoint navigation capabilities for the Yahboom robot.
    It uses a single map called 'yahboom_map' and stores waypoints in a JSON file.
    Navigation commands can be sent via MQTT in either JSON format or plain text.
    
    JSON format: {"command": "goto", "waypoint_id": "waypoint_name"}
    Plain text format: "waypoint_name"
    
    All waypoints are stored under the 'yahboom_map' key in the waypoints dictionary.
    """
    def __init__(self):
        super().__init__('b4m_waypoint_nav')
        
        # Set up file logging
        log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'logs')
        os.makedirs(log_dir, exist_ok=True)
        log_file = os.path.join(log_dir, 'waypoint_nav.log')
        logging.basicConfig(filename=log_file, level=logging.INFO,
                            format='%(asctime)s - %(levelname)s - %(message)s')
        logging.info('B4MWaypointNav node starting')
        
        # Initialize waypoints storage
        # The waypoints dictionary has a single key 'yahboom_map' containing all waypoints
        self.waypoints = {}
        # Store waypoints in the install directory
        self.pkg_share = get_package_share_directory('b4m_waypoint_nav')
        self.repo_root = os.path.abspath(os.path.join(self.pkg_share, '..', '..'))
        # Use the waypoints file in the install directory
        self.waypoints_file = os.path.join(self.pkg_share, 'waypoints.json')
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
            
        # MQTT status publisher
        self.mqtt_status_publisher = self.create_publisher(
            String,
            'mqtt_status',
            10)
            
        # Velocity publisher for robot control
        self.vel_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10)
        
        # MQTT parameters
        self.declare_parameter('mqtt_broker', 'localhost')
        self.declare_parameter('mqtt_port', 1883)
        self.declare_parameter('mqtt_topic_prefix', 'yahboom')
        self.declare_parameter('mqtt_username', '')
        self.declare_parameter('mqtt_password', '')
        
        self.mqtt_broker = self.get_parameter('mqtt_broker').value
        self.mqtt_port = self.get_parameter('mqtt_port').value
        self.mqtt_topic_prefix = self.get_parameter('mqtt_topic_prefix').value
        self.mqtt_username = self.get_parameter('mqtt_username').value
        self.mqtt_password = self.get_parameter('mqtt_password').value
        
        # Log MQTT connection parameters (excluding password)
        self.get_logger().info(f'MQTT Configuration - Broker: {self.mqtt_broker}, Port: {self.mqtt_port}, Username: {self.mqtt_username}')
        logging.info(f'MQTT Configuration - Broker: {self.mqtt_broker}, Port: {self.mqtt_port}, Username: {self.mqtt_username}')
        self.mqtt_client = None
        self.mqtt_connected = False
        
        # Timer for publishing waypoint markers
        self.marker_timer = self.create_timer(1.0, self.publish_waypoint_markers)
        
        # Set up MQTT client
        self.setup_mqtt_client()
        
        self.get_logger().info('b4m_waypoint_nav node initialized')
        
    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose
        
    def load_waypoints(self):
        """Load waypoints from JSON file"""
        self.get_logger().info(f'Loading waypoints from {self.waypoints_file}')
        logging.info(f'Loading waypoints from {self.waypoints_file}')
        
        if os.path.exists(self.waypoints_file):
            try:
                with open(self.waypoints_file, 'r') as f:
                    file_content = f.read()
                    self.get_logger().info(f'File content length: {len(file_content)} bytes')
                    logging.info(f'File content length: {len(file_content)} bytes')
                    
                    if len(file_content.strip()) == 0:
                        self.get_logger().error('Waypoints file is empty')
                        logging.error('Waypoints file is empty')
                        self.waypoints = {}
                        return False
                    
                    # Try to parse the JSON
                    self.waypoints = json.loads(file_content)
                    
                self.get_logger().info(f'Loaded waypoints for {len(self.waypoints)} maps: {list(self.waypoints.keys())}')
                logging.info(f'Loaded waypoints for {len(self.waypoints)} maps: {list(self.waypoints.keys())}')
                
                # Print all loaded waypoints for debugging
                if 'yahboom_map' in self.waypoints:
                    self.get_logger().info(f'Loaded waypoints for yahboom_map: {list(self.waypoints["yahboom_map"].keys())}')
                    logging.info(f'Loaded waypoints for yahboom_map: {list(self.waypoints["yahboom_map"].keys())}')
                else:
                    self.get_logger().info('No waypoints found for yahboom_map')
                    logging.info('No waypoints found for yahboom_map')
                    
                return True
            except Exception as e:
                self.get_logger().error(f'Error loading waypoints: {e}')
                logging.error(f'Error loading waypoints: {e}')
                import traceback
                self.get_logger().error(f'Traceback: {traceback.format_exc()}')
                logging.error(f'Traceback: {traceback.format_exc()}')
                self.waypoints = {}
                return False
        else:
            self.get_logger().warn(f'Waypoints file not found: {self.waypoints_file}')
            logging.warn(f'Waypoints file not found: {self.waypoints_file}')
            
            # Create an empty waypoints file with a default structure
            self.waypoints = {
                'yahboom_map': {
                    'Kitchen': {'position': {'x': 1.0, 'y': 1.0, 'z': 0.0}, 
                                'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}},
                    'Dining Room': {'position': {'x': 2.0, 'y': 2.0, 'z': 0.0}, 
                                   'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}}
                }
            }
            
            # Save the default waypoints
            try:
                # Ensure directory exists
                os.makedirs(os.path.dirname(self.waypoints_file), exist_ok=True)
                with open(self.waypoints_file, 'w') as f:
                    json.dump(self.waypoints, f, indent=2)
                self.get_logger().info(f'Created default waypoints file: {self.waypoints_file}')
                logging.info(f'Created default waypoints file: {self.waypoints_file}')
                return True
            except Exception as e:
                self.get_logger().error(f'Failed to create default waypoints file: {e}')
                logging.error(f'Failed to create default waypoints file: {e}')
                return False
    def save_waypoints(self):
        try:
            with open(self.waypoints_file, 'w') as f:
                json.dump(self.waypoints, f, indent=2)
            self.get_logger().info(f'Saved {len(self.waypoints)} waypoints')
        except Exception as e:
            self.get_logger().error(f'Failed to save waypoints: {e}')
    
    def store_current_waypoint(self, name):
        """Store current robot pose as a waypoint
        
        All waypoints are stored under the yahboom_map key in the waypoints dictionary.
        """
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
        
        # Ensure yahboom_map key exists
        if 'yahboom_map' not in self.waypoints:
            self.waypoints['yahboom_map'] = {}
        
        # Store waypoint under yahboom_map
        self.waypoints['yahboom_map'][name] = waypoint
        self.save_waypoints()
        self.get_logger().info(f'Stored waypoint: {name} under yahboom_map')
        return True
        
    def navigate_to_coordinates(self, name, pos_x, pos_y, orient_x, orient_y, orient_z, orient_w):
        """Navigate directly to specified coordinates
        
        This method allows navigation to arbitrary coordinates without requiring
        the waypoint to be pre-defined in the waypoints.json file.
        
        Args:
            name: A name for this navigation goal (for status reporting)
            pos_x: X coordinate in the map frame
            pos_y: Y coordinate in the map frame
            orient_x: X component of orientation quaternion
            orient_y: Y component of orientation quaternion
            orient_z: Z component of orientation quaternion
            orient_w: W component of orientation quaternion
        """
        # Create goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        
        # Set position
        goal_pose.pose.position.x = float(pos_x)
        goal_pose.pose.position.y = float(pos_y)
        goal_pose.pose.position.z = 0.0
        
        # Set orientation
        goal_pose.pose.orientation.x = float(orient_x)
        goal_pose.pose.orientation.y = float(orient_y)
        goal_pose.pose.orientation.z = float(orient_z)
        goal_pose.pose.orientation.w = float(orient_w)
        
        # Wait for action server
        self.get_logger().info('Waiting for action server...')
        self.nav_to_pose_client.wait_for_server()
        
        # Send goal
        self.get_logger().info(f'Sending goal to coordinates: x={goal_pose.pose.position.x}, y={goal_pose.pose.position.y}')
        self.current_waypoint = name
        
        # Create and send goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        
        self.publish_mqtt_status(f'Navigating to coordinates: {name} (x={pos_x}, y={pos_y})')
        
        # Send the goal and register callbacks
        self._send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
    def navigate_to_waypoint(self, name):
        # We always use yahboom_map
        current_map = 'yahboom_map'
        
        # Log the waypoint name and available waypoints for debugging
        self.get_logger().info(f'Attempting to navigate to waypoint: "{name}"')
        self.get_logger().info(f'Available waypoints: {list(self.waypoints[current_map].keys())}')
        
        # Check if waypoint exists
        if name not in self.waypoints[current_map]:
            self.get_logger().warn(f'Waypoint not found: "{name}"')
            self.send_mqtt_error(f'Waypoint not found: {name}')
            return
        
        # Get waypoint data
        waypoint = self.waypoints[current_map][name]
        
        # Create goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        
        # Set position
        goal_pose.pose.position.x = waypoint['position']['x']
        goal_pose.pose.position.y = waypoint['position']['y']
        goal_pose.pose.position.z = 0.0
        
        # Set orientation
        goal_pose.pose.orientation.x = waypoint['orientation']['x']
        goal_pose.pose.orientation.y = waypoint['orientation']['y']
        goal_pose.pose.orientation.z = waypoint['orientation']['z']
        goal_pose.pose.orientation.w = waypoint['orientation']['w']
        
        # Wait for action server
        self.get_logger().info('Waiting for action server...')
        self.nav_to_pose_client.wait_for_server()
        
        # Send goal
        self.get_logger().info(f'Sending goal: x={goal_pose.pose.position.x}, y={goal_pose.pose.position.y}')
        self.current_waypoint = name
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
            self.get_logger().error('Goal rejected by navigation server')
            
            # Check if the robot is properly localized
            if self.current_pose is None:
                self.get_logger().error('Robot is not localized - no pose data available')
                self.send_mqtt_error('Navigation goal rejected: Robot is not localized')
            else:
                self.get_logger().error(f'Robot is at position: x={self.current_pose.pose.pose.position.x}, y={self.current_pose.pose.pose.position.y}')
                self.send_mqtt_error('Navigation goal rejected: Check RViz for possible issues with the goal location')
                
            # Check if the navigation stack is properly initialized
            self.get_logger().error('Checking navigation stack status...')
            self.publish_mqtt_status('Error: Navigation goal rejected - checking system status')
            return

        self.get_logger().info('Goal accepted')
        self.publish_mqtt_status('Navigation goal accepted')
        
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
            
        # We always use yahboom_map
        current_map = 'yahboom_map'
        
        self.get_logger().info(f'Stored waypoints:')
        if current_map not in self.waypoints or not self.waypoints[current_map]:
            self.get_logger().info('No waypoints stored')
            return
            
        for name in sorted(self.waypoints[current_map].keys()):
            wp = self.waypoints[current_map][name]
            self.get_logger().info(f"- {name}: pos({wp['position']['x']:.2f}, {wp['position']['y']:.2f})")
            
        self.get_logger().info(f'Total waypoints: {len(self.waypoints[current_map])}')
        
    
    def delete_waypoint(self, name):
        # We always use yahboom_map
        current_map = 'yahboom_map'
        
        # Check if waypoint exists
        if name not in self.waypoints[current_map]:
            self.get_logger().warn(f'Waypoint not found: {name}')
            return False
                
        del self.waypoints[current_map][name]
        self.save_waypoints()
        self.get_logger().info(f'Deleted waypoint {name}')
        return True
        
    def publish_waypoint_markers(self):
        """Publish markers for all waypoints to visualize in RViz"""
        try:
            marker_array = MarkerArray()
            
            # We always use yahboom_map
            current_map = 'yahboom_map'
            
            map_waypoints = self.waypoints[current_map]
            
            for i, (name, waypoint) in enumerate(map_waypoints.items()):
                try:
                    # Skip waypoints without position data
                    if 'position' not in waypoint or not isinstance(waypoint['position'], dict):
                        self.get_logger().warn(f"Waypoint {name} has no valid position data, skipping")
                        continue
                    
                    # Create text marker for the waypoint name
                    text_marker = Marker()
                    text_marker.header.frame_id = 'map'
                    text_marker.header.stamp = self.get_clock().now().to_msg()
                    text_marker.ns = 'waypoint_names'
                    text_marker.id = i
                    text_marker.type = Marker.TEXT_VIEW_FACING
                    text_marker.action = Marker.ADD
                    
                    # Position slightly above the waypoint
                    text_marker.pose.position.x = waypoint['position'].get('x', 0.0)
                    text_marker.pose.position.y = waypoint['position'].get('y', 0.0)
                    text_marker.pose.position.z = 0.5  # Slightly above the ground
                    
                    text_marker.pose.orientation.x = 0.0
                    text_marker.pose.orientation.y = 0.0
                    text_marker.pose.orientation.z = 0.0
                    text_marker.pose.orientation.w = 1.0
                    
                    text_marker.scale.z = 0.3  # Text size
                    
                    # Use visualization color if defined, or default to white
                    if 'visualization' in waypoint and 'color' in waypoint['visualization']:
                        color = waypoint['visualization']['color']
                        text_marker.color = ColorRGBA(r=color.get('r', 1.0), g=color.get('g', 1.0), b=color.get('b', 1.0), a=1.0)
                    else:
                        text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
                        
                    text_marker.text = name
                    text_marker.lifetime.sec = 0  # Persistent
                    
                    # Add text marker to array
                    marker_array.markers.append(text_marker)
                    
                    # Create arrow marker for orientation if orientation data exists
                    if 'orientation' in waypoint and isinstance(waypoint['orientation'], dict):
                        arrow_marker = Marker()
                        arrow_marker.header.frame_id = 'map'
                        arrow_marker.header.stamp = self.get_clock().now().to_msg()
                        arrow_marker.ns = 'waypoint_orientations'
                        arrow_marker.id = i
                        arrow_marker.type = Marker.ARROW
                        arrow_marker.action = Marker.ADD
                        
                        # Set position and orientation
                        arrow_marker.pose.position.x = waypoint['position'].get('x', 0.0)
                        arrow_marker.pose.position.y = waypoint['position'].get('y', 0.0)
                        arrow_marker.pose.position.z = 0.1  # Slightly above the ground
                        
                        arrow_marker.pose.orientation.x = waypoint['orientation'].get('x', 0.0)
                        arrow_marker.pose.orientation.y = waypoint['orientation'].get('y', 0.0)
                        arrow_marker.pose.orientation.z = waypoint['orientation'].get('z', 0.0)
                        arrow_marker.pose.orientation.w = waypoint['orientation'].get('w', 1.0)
                        
                        # Arrow size
                        arrow_marker.scale.x = 0.5  # Length
                        arrow_marker.scale.y = 0.1  # Width
                        arrow_marker.scale.z = 0.1  # Height
                        
                        # Use same color as text
                        arrow_marker.color = text_marker.color
                        arrow_marker.lifetime.sec = 0  # Persistent
                        
                        # Add arrow marker to array
                        marker_array.markers.append(arrow_marker)
                    else:
                        self.get_logger().warn(f"Waypoint {name} has no valid orientation data, skipping orientation marker")
                except Exception as e:
                    self.get_logger().error(f"Error creating marker for waypoint {name}: {str(e)}")
                    continue
            
            # Publish the marker array
            self.marker_publisher.publish(marker_array)
        except Exception as e:
            self.get_logger().error(f"Error publishing waypoint markers: {str(e)}")
            import traceback
            traceback.print_exc()
    
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
    
    # keyboard_control method removed as it's no longer needed with GUI and MQTT control
        
    def setup_mqtt_client(self):
        # Set up MQTT client and connection
        client_id = f'b4m_waypoint_nav_{random.randint(0, 1000)}'
        self.get_logger().info(f'Setting up MQTT client with ID: {client_id}')
        logging.info(f'Setting up MQTT client with ID: {client_id}')
        
        self.mqtt_client = mqtt.Client(client_id=client_id, protocol=mqtt.MQTTv311)
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_disconnect = self.on_mqtt_disconnect
        self.mqtt_client.on_message = self.on_mqtt_message
        
        # Connect to MQTT broker
        try:
            self.get_logger().info(f'Connecting to MQTT broker at {self.mqtt_broker}:{self.mqtt_port}')
            logging.info(f'Connecting to MQTT broker at {self.mqtt_broker}:{self.mqtt_port}')
            
            # Set username and password if provided
            if self.mqtt_username and self.mqtt_password:
                self.get_logger().info(f'Using authentication for MQTT broker with username: {self.mqtt_username}')
                logging.info(f'Using authentication for MQTT broker with username: {self.mqtt_username}')
                self.mqtt_client.username_pw_set(self.mqtt_username, self.mqtt_password)
            
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port)
            self.mqtt_client.loop_start()
        except Exception as e:
            self.get_logger().error(f'Failed to connect to MQTT broker: {e}')
            logging.error(f'Failed to connect to MQTT broker: {e}')

    def on_mqtt_disconnect(self, client, userdata, rc):
        """Callback when disconnected from MQTT broker"""
        self.get_logger().warn(f'Disconnected from MQTT broker with code {rc}')
        logging.warn(f'Disconnected from MQTT broker with code {rc}')
        self.mqtt_connected = False
        self.publish_mqtt_status('Disconnected from MQTT broker')

    def on_mqtt_connect(self, client, userdata, flags, rc, properties=None):
        """Callback when connected to MQTT broker"""
        if rc == 0:
            self.get_logger().info('Connected to MQTT broker')
            logging.info('Connected to MQTT broker')
            self.mqtt_connected = True
            
            # Subscribe to navigation command topic
            command_topic = f"{self.mqtt_topic_prefix}/navigation/command"
            self.get_logger().info(f'Subscribing to MQTT topic: {command_topic}')
            logging.info(f'Subscribing to MQTT topic: {command_topic}')
            
            try:
                client.subscribe(command_topic)
                self.get_logger().info(f'Successfully subscribed to {command_topic}')
                logging.info(f'Successfully subscribed to {command_topic}')
                
                # Publish status message
                status_topic = f"{self.mqtt_topic_prefix}/navigation/status"
                client.publish(status_topic, "Waypoint navigation system ready")
                self.get_logger().info(f'Published ready message to {status_topic}')
                logging.info(f'Published ready message to {status_topic}')
                
                # Publish MQTT command format documentation
                info_topic = f"{self.mqtt_topic_prefix}/navigation/info"
                info_message = '{"info": "Command format: {\"command\": \"goto\", \"waypoint_id\": \"debug_name\", \"position\": {\"x\": float, \"y\": float}, \"orientation\": {\"x\": float, \"y\": float, \"z\": float, \"w\": float}}. The waypoint_id is for debugging only."}'
                client.publish(info_topic, info_message)
                self.get_logger().info(f'Published command format info to {info_topic}')
                logging.info(f'Published command format info to {info_topic}')
            except Exception as e:
                self.get_logger().error(f'Error subscribing to MQTT topics: {e}')
                logging.error(f'Error subscribing to MQTT topics: {e}')
        else:
            self.get_logger().error(f'Failed to connect to MQTT broker with code {rc}')
            logging.error(f'Failed to connect to MQTT broker with code {rc}')
            self.mqtt_connected = False
            
    def on_mqtt_message(self, client, userdata, msg):
        """Callback when a message is received
        
        The waypoint manager GUI must use coordinate-based navigation commands.
        The only supported format is:
        
        {"command": "goto", "waypoint_id": "debug_name", 
         "position": {"x": float, "y": float}, 
         "orientation": {"x": float, "y": float, "z": float, "w": float}}
        
        The waypoint_id is included only for debugging and logging purposes.
        The navigation node uses the coordinates directly without any waypoint lookup.
        """
        try:
            topic = msg.topic
            payload = msg.payload.decode('utf-8')
            self.get_logger().info(f'MQTT message received on topic: {topic}, payload: "{payload}"')
            logging.info(f'MQTT message received on topic: {topic}, payload: "{payload}"')
            
            if topic == f"{self.mqtt_topic_prefix}/navigation/command":
                self.get_logger().info(f'Received navigation command: "{payload}"')
                logging.info(f'Received navigation command: "{payload}"')
                
                try:
                    # Parse the JSON message
                    data = json.loads(payload)
                    
                    # Validate the command format
                    if 'command' not in data or data['command'] != 'goto':
                        self.get_logger().warn(f'Invalid command format: {payload}')
                        logging.warn(f'Invalid command format: {payload}')
                        self.publish_mqtt_status('Error: Invalid command format. Expected {"command": "goto", ...}')
                        return
                        
                    # Validate required fields
                    if 'position' not in data or 'orientation' not in data:
                        self.get_logger().warn(f'Missing position or orientation data: {payload}')
                        logging.warn(f'Missing position or orientation data: {payload}')
                        self.publish_mqtt_status(f'Error: Navigation command must include position and orientation data')
                        return
                    
                    # Extract position and orientation
                    position = data['position']
                    orientation = data['orientation']
                    waypoint_name = data.get('waypoint_id', 'Custom Waypoint')
                    
                    self.get_logger().info(f'Using coordinate-based navigation for: "{waypoint_name}"')
                    logging.info(f'Using coordinate-based navigation for: "{waypoint_name}"')
                    
                    # Navigate directly to the provided coordinates
                    self.navigate_to_coordinates(
                        waypoint_name,
                        position.get('x', 0.0),
                        position.get('y', 0.0),
                        orientation.get('x', 0.0),
                        orientation.get('y', 0.0),
                        orientation.get('z', 0.0),
                        orientation.get('w', 1.0)
                    )
                except json.JSONDecodeError:
                    self.get_logger().error(f'Invalid JSON format: {payload}')
                    logging.error(f'Invalid JSON format: {payload}')
                    self.publish_mqtt_status('Error: Invalid JSON format. Expected {"command": "goto", "position": {...}, "orientation": {...}}')
                except Exception as e:
                    self.get_logger().error(f'Error processing navigation command: {e}')
                    logging.error(f'Error processing navigation command: {e}')
                    self.publish_mqtt_status(f'Error: Failed to process navigation command: {e}')
            else:
                self.get_logger().info(f'Ignoring message on topic: {topic}')
                logging.info(f'Ignoring message on topic: {topic}')
        except Exception as e:
            self.get_logger().error(f'Error processing MQTT message: {e}')
            logging.error(f'Error processing MQTT message: {e}')
            import traceback
            self.get_logger().error(f'Traceback: {traceback.format_exc()}')
            logging.error(f'Traceback: {traceback.format_exc()}')
            self.publish_mqtt_status(f'Error: Failed to process MQTT message: {e}')
            
    def handle_navigation_command(self, waypoint_name):
        """Handle navigation command received via MQTT
        
        All waypoints are stored under the yahboom_map key in the waypoints dictionary.
        This method checks if the requested waypoint exists and initiates navigation.
        """
        self.get_logger().info(f'Handling navigation command for waypoint: "{waypoint_name}"')
        
        waypoint_name = waypoint_name.strip()
        
        # We always use yahboom_map
        current_map = 'yahboom_map'
        
        self.get_logger().info(f'Available waypoints: {list(self.waypoints[current_map].keys())}')
        
        # Check if the waypoint exists
        if waypoint_name not in self.waypoints[current_map]:
            self.get_logger().error(f'Waypoint "{waypoint_name}" not found')
            self.publish_mqtt_status(f'Error: Waypoint "{waypoint_name}" not found')
            return
        
        # Navigate to the waypoint
        self.get_logger().info(f'Sending navigation goal for waypoint: "{waypoint_name}"')
        self.navigate_to_waypoint(waypoint_name)

    def publish_mqtt_status(self, status_message):
        """Publish status message to MQTT status topic
        
        Status messages are published to the yahboom/navigation/status topic
        and follow a simple string format for maximum compatibility.
        """
        if hasattr(self, 'mqtt_status_publisher'):
            status_msg = String()
            status_msg.data = status_message
            self.mqtt_status_publisher.publish(status_msg)
    
    def get_current_map_name(self):
        """Get the name of the current map"""
        # We only use yahboom_map in this simplified version
        self.get_logger().info('Using yahboom_map as the current map')
        logging.info('Using yahboom_map as the current map')
        return 'yahboom_map'

def main(args=None):
    rclpy.init(args=args)
    node = B4MWaypointNav()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
