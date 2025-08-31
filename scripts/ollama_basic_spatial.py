#!/usr/bin/env python3
"""
Ollama Basic Spatial Analysis for Navigation
Simple integration of Ollama spatial context generation with Nav2
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import numpy as np
import json
import requests
import math
import yaml
import os
import time

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
        
        # State tracking
        self.last_analysis_time = 0
        self.analysis_interval = 10.0  # Analyze every 10 seconds
        self.latest_laser_data = None
        
        self.get_logger().info('✅ Ollama Basic Spatial Analysis initialized')
        self.get_logger().info(f'📡 Connected to Ollama at {self.ollama_url}')
        
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
                    'max_goal_distance': 5.0
                }
            }
    
    def laser_callback(self, msg):
        """Store latest laser scan data"""
        self.latest_laser_data = msg
        
        # Check if it's time for spatial analysis
        current_time = time.time()
        if current_time - self.last_analysis_time > self.analysis_interval:
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
        # First try simple rule-based goal selection
        fallback_goal = self.generate_fallback_goal(spatial_context)
        
        try:
            # Create spatial description
            sectors_desc = []
            for sector in spatial_context['sectors']:
                sectors_desc.append(f"{sector['direction']}: {sector['status']} (min: {sector['min_distance']:.1f}m)")
            
            spatial_summary = "\\n".join(sectors_desc)
            
            # Very simple prompt
            prompt = f"Robot sees: {spatial_summary}\\n\\nSuggest goal coordinates (x,y) in JSON: {{\"goal_x\": 2.0, \"goal_y\": 0.0}}"

            payload = {
                "model": self.config['ollama_nav']['model'],
                "prompt": prompt,
                "stream": False,
                "options": {
                    "temperature": 0.1,
                    "num_predict": 50
                }
            }
            
            self.get_logger().info('🧠 Querying Ollama for goal suggestion...')
            
            response = requests.post(
                self.ollama_url,
                json=payload,
                timeout=10.0
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
                        
                        if 0.5 <= distance <= 4.0:  # Reasonable distance
                            goal_data['reasoning'] = f'Ollama suggested goal at distance {distance:.1f}m'
                            self.get_logger().info(f'✅ Ollama goal: ({x}, {y})')
                            return goal_data
                        else:
                            self.get_logger().warn(f'⚠️  Ollama goal distance {distance:.1f}m unreasonable, using fallback')
                            
                except (json.JSONDecodeError, KeyError) as e:
                    self.get_logger().warn(f'⚠️  Ollama JSON parse error, using fallback: {e}')
            else:
                self.get_logger().warn(f'⚠️  Ollama API error {response.status_code}, using fallback')
                
        except requests.exceptions.Timeout:
            self.get_logger().warn('⚠️  Ollama timeout, using fallback goal')
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
            
            return {
                'goal_x': goal_x,
                'goal_y': goal_y,
                'reasoning': f'Fallback: Move toward {target_sector["direction"]} (clearest path)'
            }
        else:
            # Emergency fallback - move forward a small amount
            return {
                'goal_x': 1.0,
                'goal_y': 0.0,
                'reasoning': 'Emergency fallback: Move forward 1m'
            }
    
    def publish_navigation_goal(self, goal_data):
        """Publish navigation goal to RViz"""
        try:
            goal_msg = PoseStamped()
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.header.frame_id = 'map'
            
            # Set position from Ollama suggestion
            goal_msg.pose.position.x = float(goal_data['goal_x'])
            goal_msg.pose.position.y = float(goal_data['goal_y'])
            goal_msg.pose.position.z = 0.0
            
            # Set orientation (facing forward)
            goal_msg.pose.orientation.x = 0.0
            goal_msg.pose.orientation.y = 0.0
            goal_msg.pose.orientation.z = 0.0
            goal_msg.pose.orientation.w = 1.0
            
            # Validate goal distance
            distance = math.sqrt(goal_data['goal_x']**2 + goal_data['goal_y']**2)
            min_dist = self.config['navigation']['min_goal_distance']
            max_dist = self.config['navigation']['max_goal_distance']
            
            if min_dist <= distance <= max_dist:
                self.goal_publisher.publish(goal_msg)
                self.get_logger().info(f'🎯 Published navigation goal: ({goal_data["goal_x"]}, {goal_data["goal_y"]})')
            else:
                self.get_logger().warn(f'⚠️  Goal distance {distance:.1f}m outside safe range [{min_dist}, {max_dist}]')
                
        except Exception as e:
            self.get_logger().error(f'❌ Failed to publish navigation goal: {e}')

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