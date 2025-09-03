#!/usr/bin/env python3
"""
Ollama Basic Spatial Analysis for Navigation
Simple integration of Ollama spatial context generation with Nav2
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
import numpy as np
import json
import requests
import math
import yaml
import os
import time
import random

class OllamaBasicSpatial(Node):
    """Basic Ollama spatial analysis for navigation goal suggestions"""
    
    def __init__(self):
        super().__init__('ollama_basic_spatial')
        
        # Load configuration
        self.load_config()
        
        # Initialize Ollama connection
        self.ollama_url = f"http://{self.config['ollama_nav']['host']}:{self.config['ollama_nav']['port']}/api/generate"
        
        # ROS2 subscribers and publishers
        self.laser_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )
        
        self.goal_publisher = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10
        )
        
        # Navigation action client for proper goal handling
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # State tracking
        self.last_analysis_time = 0
        self.analysis_interval = self.config.get('navigation', {}).get('analysis_interval', 30.0)
        self.latest_laser_data = None
        self.current_goal_handle = None
        self.navigating = False
        
        rotation_prob = self.config.get('navigation', {}).get('rotation_probability', 0.5)
        max_backward = self.config.get('navigation', {}).get('max_backward_distance', 0.61)
        timeout_val = self.config['ollama_nav']['timeout']
        self.get_logger().info('✅ Ollama Basic Spatial Analysis initialized')
        self.get_logger().info(f'📡 Connected to Ollama at {self.ollama_url}')
        self.get_logger().info(f'⏱️  Analysis interval: {self.analysis_interval}s (3x longer for goal completion)')
        self.get_logger().info(f'⏳ LLM timeout: {timeout_val}s ({timeout_val/60:.1f} minutes) - will wait for response')
        self.get_logger().info(f'🔄 Rotation probability: {rotation_prob*100:.0f}% (robot will rotate in place vs move)')
        self.get_logger().info(f'⬅️  Max backward movement: {max_backward:.2f}m ({max_backward/0.3048:.1f} feet)')
        
    def load_config(self):
        """Load configuration from ollama_nav_config.yaml"""
        config_path = '/home/mike/projects/b4m_yahboom/config/ollama_nav_config.yaml'
        
        try:
            with open(config_path, 'r') as f:
                self.config = yaml.safe_load(f)
            self.get_logger().info(f'✅ Configuration loaded from {config_path}')
        except Exception as e:
            self.get_logger().error(f'❌ Failed to load config: {e}')
            # Fallback configuration
            self.config = {
                'ollama_nav': {
                    'host': 'localhost',
                    'port': 11434,
                    'model': 'llama3.2:latest',
                    'timeout': 10.0
                },
                'generation': {
                    'temperature': 0.2,
                    'max_tokens': 200
                },
                'navigation': {
                    'min_goal_distance': 1.0,
                    'max_goal_distance': 5.0,
                    'analysis_interval': 30.0,
                    'rotation_probability': 0.5
                }
            }
    
    def laser_callback(self, msg):
        """Store latest laser scan data"""
        self.latest_laser_data = msg
        
        # Check if it's time for spatial analysis AND not currently navigating
        current_time = time.time()
        if current_time - self.last_analysis_time > self.analysis_interval:
            # Only analyze if not currently navigating or goal is complete
            if not self.navigating or self.is_navigation_complete():
                self.perform_spatial_analysis()
                self.last_analysis_time = current_time
    
    def perform_spatial_analysis(self):
        """Perform spatial analysis and suggest navigation goal"""
        if self.latest_laser_data is None:
            return
            
        self.get_logger().info('🔍 Performing Ollama spatial analysis...')
        
        # Generate spatial context from laser data
        spatial_context = self.generate_spatial_context(self.latest_laser_data)
        
        if spatial_context:
            # Send to Ollama for goal suggestion
            goal_suggestion = self.query_ollama_for_goal(spatial_context)
            
            if goal_suggestion:
                self.publish_navigation_goal(goal_suggestion)
    
    def generate_spatial_context(self, laser_data):
        """Generate basic spatial context from laser scan"""
        ranges = np.array(laser_data.ranges)
        ranges[np.isinf(ranges)] = laser_data.range_max
        ranges[np.isnan(ranges)] = laser_data.range_max
        ranges[ranges == 0.0] = laser_data.range_max
        
        # Divide into 8 sectors (45° each) for simple analysis
        num_readings = len(ranges)
        sectors = []
        sector_size = 45  # degrees
        
        for i in range(8):  # 8 sectors of 45° each
            start_angle = i * sector_size - 180  # Start from -180°
            end_angle = start_angle + sector_size
            
            # Convert to indices
            start_rad = math.radians(start_angle)
            end_rad = math.radians(end_angle)
            
            angle_increment = (laser_data.angle_max - laser_data.angle_min) / (num_readings - 1)
            start_idx = max(0, int((start_rad - laser_data.angle_min) / angle_increment))
            end_idx = min(num_readings - 1, int((end_rad - laser_data.angle_min) / angle_increment))
            
            if start_idx <= end_idx:
                sector_ranges = ranges[start_idx:end_idx + 1]
                min_dist = np.min(sector_ranges)
                avg_dist = np.mean(sector_ranges)
                
                # Determine status
                if min_dist < 0.5:
                    status = 'BLOCKED'
                elif min_dist < 2.0:
                    status = 'PARTIAL'
                else:
                    status = 'CLEAR'
                
                sectors.append({
                    'direction': self.angle_to_direction(start_angle + sector_size/2),
                    'angle': start_angle + sector_size/2,
                    'status': status,
                    'min_distance': min_dist,
                    'avg_distance': avg_dist
                })
        
        return {
            'sectors': sectors,
            'timestamp': time.time()
        }
    
    def angle_to_direction(self, angle):
        """Convert angle to direction name"""
        directions = [
            (0, "front"),
            (45, "front-right"), 
            (90, "right"),
            (135, "back-right"),
            (180, "back"),
            (-135, "back-left"),
            (-90, "left"),
            (-45, "front-left")
        ]
        
        angle = angle % 360
        if angle > 180:
            angle -= 360
            
        closest = min(directions, key=lambda x: abs(x[0] - angle))
        return closest[1]
    
    def query_ollama_for_goal(self, spatial_context):
        """Query Ollama for navigation goal suggestion with fallback"""
        
        # Check if we should rotate in place (configurable probability)
        rotation_prob = self.config.get('navigation', {}).get('rotation_probability', 0.5)
        if random.random() < rotation_prob:
            self.get_logger().info('🔄 Selecting rotation-in-place goal for better scanning')
            return self.generate_rotation_goal()
        
        # First try simple rule-based goal selection
        fallback_goal = self.generate_fallback_goal(spatial_context)
        
        try:
            # Create spatial description
            sectors_desc = []
            for sector in spatial_context['sectors']:
                sectors_desc.append(f"{sector['direction']}: {sector['status']} (min: {sector['min_distance']:.1f}m)")
            
            spatial_summary = "\\n".join(sectors_desc)
            
            # Simplified prompt for faster response
            prompt = f"Robot sensors: {spatial_summary}\\n\\nJSON goal: {{\"goal_x\": 1.5, \"goal_y\": 0.0}}"

            payload = {
                "model": self.config['ollama_nav']['model'],
                "prompt": prompt,
                "stream": False,
                "options": {
                    "temperature": 0.1,
                    "num_predict": 30,
                    "top_p": 0.9,
                    "top_k": 10
                }
            }
            
            timeout_val = self.config['ollama_nav']['timeout']
            self.get_logger().info(f'🧠 Querying Ollama for goal suggestion (timeout: {timeout_val}s)...')
            self.get_logger().info('⏳ Waiting for LLM response... this may take up to 2 minutes')
            
            response = requests.post(
                self.ollama_url,
                json=payload,
                timeout=timeout_val
            )
            
            if response.status_code == 200:
                result = response.json()
                response_text = result.get('response', '')
                
                # Try to extract JSON from response
                try:
                    start = response_text.find('{')
                    end = response_text.rfind('}') + 1
                    if start >= 0 and end > start:
                        json_str = response_text[start:end]
                        goal_data = json.loads(json_str)
                        
                        # Validate goal coordinates
                        x, y = goal_data.get('goal_x', 0), goal_data.get('goal_y', 0)
                        distance = math.sqrt(x*x + y*y)
                        
                        # Check backward movement limit (2 feet = 0.61 meters)
                        MAX_BACKWARD_DISTANCE = self.config['navigation'].get('max_backward_distance', 0.61)
                        if x < -MAX_BACKWARD_DISTANCE:
                            self.get_logger().warn(f'⚠️  Ollama goal would move backward {abs(x):.1f}m (exceeds 2ft limit), adjusting...')
                            # Adjust to maximum allowed backward distance
                            old_x = x
                            x = -MAX_BACKWARD_DISTANCE
                            # Scale y proportionally to maintain direction
                            if old_x != 0:
                                y = y * (x / old_x)
                            goal_data['goal_x'] = x
                            goal_data['goal_y'] = y
                            distance = math.sqrt(x*x + y*y)
                        
                        if 0.5 <= distance <= 4.0:  # Reasonable distance
                            goal_data['reasoning'] = f'Ollama suggested goal at distance {distance:.1f}m (backward limited to 2ft)'
                            self.get_logger().info(f'✅ Ollama goal: ({x}, {y})')
                            return goal_data
                        else:
                            self.get_logger().warn(f'⚠️  Ollama goal distance {distance:.1f}m unreasonable, using fallback')
                            
                except (json.JSONDecodeError, KeyError) as e:
                    self.get_logger().warn(f'⚠️  Ollama JSON parse error, using fallback: {e}')
            else:
                self.get_logger().warn(f'⚠️  Ollama API error {response.status_code}, using fallback')
                
        except requests.exceptions.Timeout:
            self.get_logger().warn(f'⚠️  Ollama timeout after {self.config["ollama_nav"]["timeout"]}s, using fallback goal')
            self.get_logger().info('💡 Tip: Check if Ollama service is running: "systemctl status ollama" or "ollama list"')
        except Exception as e:
            self.get_logger().warn(f'⚠️  Ollama error: {e}, using fallback')
        
        # Return fallback goal
        self.get_logger().info(f'🎯 Using fallback goal: ({fallback_goal["goal_x"]}, {fallback_goal["goal_y"]})')
        return fallback_goal
    
    def generate_fallback_goal(self, spatial_context):
        """Generate a safe navigation goal using simple rules"""
        # Find best clear direction
        clear_sectors = [s for s in spatial_context['sectors'] if s['status'] == 'CLEAR']
        partial_sectors = [s for s in spatial_context['sectors'] if s['status'] == 'PARTIAL']
        
        # Filter out backward sectors if they would exceed 2 feet (0.61m)
        MAX_BACKWARD_DISTANCE = self.config['navigation'].get('max_backward_distance', 0.61)  # 2 feet in meters
        
        def is_acceptable_sector(sector):
            """Check if sector doesn't violate backward movement limit"""
            angle_rad = math.radians(sector['angle'])
            # Backward is when x < 0 (angles between 90 and 270 degrees)
            if -180 <= sector['angle'] < -90 or 90 < sector['angle'] <= 180:
                # This is a backward-facing sector
                distance = min(2.0, sector['avg_distance'] * 0.7)
                goal_x = distance * math.cos(angle_rad)
                # If it would go backward more than 2 feet, reject it
                if goal_x < -MAX_BACKWARD_DISTANCE:
                    return False
            return True
        
        # Filter sectors based on backward limit
        clear_sectors = [s for s in clear_sectors if is_acceptable_sector(s)]
        partial_sectors = [s for s in partial_sectors if is_acceptable_sector(s)]
        
        target_sector = None
        
        # Prefer clear sectors, especially forward-facing ones
        if clear_sectors:
            # Prioritize forward directions
            forward_sectors = [s for s in clear_sectors if 'front' in s['direction']]
            if forward_sectors:
                target_sector = max(forward_sectors, key=lambda x: x['avg_distance'])
            else:
                target_sector = max(clear_sectors, key=lambda x: x['avg_distance'])
        elif partial_sectors:
            target_sector = max(partial_sectors, key=lambda x: x['avg_distance'])
        
        if target_sector:
            # Convert direction to coordinates
            angle_rad = math.radians(target_sector['angle'])
            distance = min(2.0, target_sector['avg_distance'] * 0.7)  # Conservative distance
            
            goal_x = distance * math.cos(angle_rad)
            goal_y = distance * math.sin(angle_rad)
            
            # Final check: limit backward movement to 2 feet
            if goal_x < -MAX_BACKWARD_DISTANCE:
                goal_x = -MAX_BACKWARD_DISTANCE
                # Recalculate goal_y to maintain direction
                if distance > 0:
                    scale = abs(goal_x / (distance * math.cos(angle_rad)))
                    goal_y = goal_y * scale
            
            return {
                'goal_x': goal_x,
                'goal_y': goal_y,
                'reasoning': f'Fallback: Move toward {target_sector["direction"]} (clearest path, limited backward)'
            }
        else:
            # Emergency fallback - move forward a small amount
            return {
                'goal_x': 1.0,
                'goal_y': 0.0,
                'reasoning': 'Emergency fallback: Move forward 1m'
            }
    
    def generate_rotation_goal(self):
        """Generate a rotation-in-place goal (same position, new orientation)"""
        
        # Choose a random new orientation (0-360 degrees)
        rotation_angles = [45, 90, 135, 180, 225, 270, 315]  # 8 cardinal/ordinal directions
        target_angle = random.choice(rotation_angles)
        target_angle_rad = math.radians(target_angle)
        
        return {
            'goal_x': 0.0,  # Stay in current position
            'goal_y': 0.0,  # Stay in current position  
            'goal_orientation': target_angle_rad,  # New orientation
            'reasoning': f'Rotation in place to face {target_angle}° for better spatial scanning'
        }
    
    def is_navigation_complete(self):
        """Check if current navigation goal is complete"""
        if not self.current_goal_handle:
            return True
            
        # Check goal status
        status = self.current_goal_handle.status
        if status in [GoalStatus.STATUS_SUCCEEDED, 
                     GoalStatus.STATUS_ABORTED,
                     GoalStatus.STATUS_CANCELED]:
            self.navigating = False
            self.current_goal_handle = None
            return True
        return False
    
    def goal_response_callback(self, future):
        """Handle goal response from Nav2"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('⚠️  Navigation goal was rejected by Nav2')
            self.navigating = False
            return
        
        self.current_goal_handle = goal_handle
        self.get_logger().info('✅ Navigation goal accepted by Nav2')
        
        # Get result asynchronously
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)
    
    def goal_result_callback(self, future):
        """Handle goal completion result"""
        result = future.result().result
        status = future.result().status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('✅ Navigation goal reached successfully!')
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn('⚠️  Navigation goal was aborted')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('🛑 Navigation goal was canceled')
        else:
            self.get_logger().warn(f'⚠️  Navigation ended with status: {status}')
        
        self.navigating = False
        self.current_goal_handle = None
    
    def publish_navigation_goal(self, goal_data):
        """Publish navigation goal using Nav2 action client"""
        try:
            # Cancel any existing goal first
            if self.navigating and self.current_goal_handle:
                self.get_logger().info('🛑 Canceling previous navigation goal...')
                self.current_goal_handle.cancel_goal_async()
                self.navigating = False
                self.current_goal_handle = None
                time.sleep(1)  # Give time for cancellation
            
            # Prepare goal message
            goal_msg = PoseStamped()
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.header.frame_id = 'map'
            
            # Set position from goal data
            goal_msg.pose.position.x = float(goal_data['goal_x'])
            goal_msg.pose.position.y = float(goal_data['goal_y'])
            goal_msg.pose.position.z = 0.0
            
            # Set orientation (handle both movement and rotation goals)
            if 'goal_orientation' in goal_data:
                # Rotation goal - convert angle to quaternion
                yaw = goal_data['goal_orientation']
                goal_msg.pose.orientation.x = 0.0
                goal_msg.pose.orientation.y = 0.0
                goal_msg.pose.orientation.z = math.sin(yaw / 2.0)
                goal_msg.pose.orientation.w = math.cos(yaw / 2.0)
            else:
                # Movement goal - face forward (default orientation)
                goal_msg.pose.orientation.x = 0.0
                goal_msg.pose.orientation.y = 0.0
                goal_msg.pose.orientation.z = 0.0
                goal_msg.pose.orientation.w = 1.0
            
            # Validate goal distance (skip validation for rotation-in-place goals)
            distance = math.sqrt(goal_data['goal_x']**2 + goal_data['goal_y']**2)
            is_rotation_goal = distance < 0.1  # Very small movement = rotation goal
            
            if is_rotation_goal:
                # Always allow rotation goals
                should_publish = True
                goal_type = "rotation"
            else:
                # Validate movement goals
                min_dist = self.config['navigation'].get('min_goal_distance', 1.0)
                max_dist = self.config['navigation'].get('max_goal_distance', 5.0)
                should_publish = min_dist <= distance <= max_dist
                goal_type = "movement"
            
            if should_publish:
                # Publish to goal_pose topic for visualization
                self.goal_publisher.publish(goal_msg)
                
                # Send goal via action client for proper Nav2 handling
                if self.nav_client.wait_for_server(timeout_sec=5.0):
                    nav_goal = NavigateToPose.Goal()
                    nav_goal.pose = goal_msg
                    
                    # Send goal asynchronously
                    self.navigating = True
                    send_goal_future = self.nav_client.send_goal_async(nav_goal)
                    send_goal_future.add_done_callback(self.goal_response_callback)
                    
                    if goal_type == "rotation":
                        angle_deg = math.degrees(goal_data.get('goal_orientation', 0))
                        self.get_logger().info(f'🔄 Sent rotation goal: face {angle_deg:.0f}° (staying at current position)')
                    else:
                        self.get_logger().info(f'🎯 Sent movement goal: ({goal_data["goal_x"]}, {goal_data["goal_y"]})')
                        self.get_logger().info('⏳ Waiting for goal completion before next analysis...')
                else:
                    self.get_logger().warn('⚠️  Nav2 action server not available, publishing to topic only')
                    self.navigating = False
            else:
                self.get_logger().warn(f'⚠️  Goal distance {distance:.1f}m outside safe range [{min_dist}, {max_dist}]')
                
        except Exception as e:
            self.get_logger().error(f'❌ Failed to publish navigation goal: {e}')
            self.navigating = False

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = OllamaBasicSpatial()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\\n🛑 Ollama Basic Spatial Analysis stopped by user')
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()