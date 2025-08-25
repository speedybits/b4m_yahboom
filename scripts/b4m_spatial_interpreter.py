#!/usr/bin/env python3
"""
B4M Spatial Interpreter - Step 1 Implementation

This script provides text-based spatial descriptions when the robot encounters obstacles.
Instead of making random turning decisions, it stops and presents a console-based interface
for manual navigation decisions.

Features:
- Blocking console mode - waits for user input when obstacles detected
- Stops at every obstacle within 30cm
- Quiet operation - no output during normal movement
- Pure text-based spatial descriptions
- LaserScan-based turn validation
- Works in both simulation and real robot modes

Author: B4M Robot System
"""

class OllamaNavigator:
    """Handles Ollama API communication for navigation decisions"""
    
    def __init__(self, config):
        self.config = config
        self.api_url = f"http://{config['ollama']['host']}:{config['ollama']['port']}/api/generate"
        self.model = config['ollama']['model']
        self.timeout = config['ollama']['timeout']
        
    def generate_prompt(self, spatial_context):
        """Generate navigation prompt from spatial context"""
        # Format spatial description
        description = []
        description.append(f"FRONT: {spatial_context['front']['description']}")
        description.append(f"LEFT: {spatial_context['left']['description']}")
        description.append(f"RIGHT: {spatial_context['right']['description']}")
        description.append(f"BEHIND: {spatial_context['behind']['description']}")
        
        if spatial_context.get('distance_traveled', 0) > 0.01:
            description.append(f"\nDISTANCE TRAVELED: {spatial_context['distance_traveled']:.2f}m since last stop")
        
        spatial_desc = "\n".join(description)
        
        prompt = f"""You are a navigation AI for a robot. Based on the following spatial description, 
decide the best action for the robot to take.

CURRENT SITUATION:
{spatial_desc}

AVAILABLE ACTIONS:
- "turn_left": Rotate 90 degrees to the left
- "turn_right": Rotate 90 degrees to the right
- "go_straight": Continue moving forward
- "turn_around": Rotate 180 degrees

SAFETY RULES:
1. Never move forward if FRONT is BLOCKED (obstacle < 30cm)
2. Prefer turning toward the direction with more open space
3. Turn around only if all other directions are blocked
4. When path is clear, prefer going straight

Respond with a JSON object containing:
- "action": one of the available actions
- "reason": brief explanation for the decision
- "confidence": confidence level (0.0 to 1.0)

Example response:
{{"action": "turn_left", "reason": "Front blocked, left side clear", "confidence": 0.95}}"""
        
        return prompt
    
    def get_navigation_decision(self, spatial_context):
        """Get navigation decision from Ollama"""
        try:
            prompt = self.generate_prompt(spatial_context)
            
            payload = {
                "model": self.model,
                "prompt": prompt,
                "format": "json",
                "stream": False,
                "options": {
                    "temperature": self.config['generation']['temperature'],
                    "top_p": self.config['generation']['top_p'],
                    "num_predict": self.config['generation']['max_tokens']
                }
            }
            
            response = requests.post(
                self.api_url,
                json=payload,
                timeout=self.timeout
            )
            
            if response.status_code == 200:
                result = response.json()
                decision = json.loads(result['response'])
                
                # Validate response
                if self.validate_response(decision):
                    return decision, prompt
                else:
                    return None, prompt
            else:
                return None, prompt
                
        except (requests.Timeout, requests.RequestException, json.JSONDecodeError) as e:
            print(f"\n⚠️ Ollama error: {str(e)}")
            return None, None
    
    def validate_response(self, response):
        """Validate Ollama response format"""
        valid_actions = ["turn_left", "turn_right", "go_straight", "turn_around"]
        
        if not isinstance(response, dict):
            return False
            
        if "action" not in response:
            return False
            
        if response["action"] not in valid_actions:
            return False
            
        # Validate confidence if present
        if "confidence" in response:
            try:
                confidence = float(response["confidence"])
                if not 0.0 <= confidence <= 1.0:
                    response["confidence"] = 0.5
            except (ValueError, TypeError):
                response["confidence"] = 0.5
        else:
            response["confidence"] = 0.5
            
        return True

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import math
import time
import sys
import select
import termios
import tty
import argparse
import os
import yaml
import requests
import json

class B4MSpatialInterpreter(Node):
    def __init__(self, ollama_mode=False):
        super().__init__('b4m_spatial_interpreter')
        
        # Check if running in Ollama mode
        self.ollama_mode = ollama_mode
        self.ollama_navigator = None
        
        if self.ollama_mode:
            # Load Ollama configuration
            self.config = self.load_ollama_config()
            if self.config:
                self.ollama_navigator = OllamaNavigator(self.config)
                self.get_logger().info("🦙 Ollama mode activated")
                print("\n🦙 OLLAMA MODE ACTIVATED")
                print("=" * 63)
                print("Robot will use Ollama LLM for navigation decisions")
                print(f"Model: {self.config['ollama']['model']}")
                print(f"API: {self.config['ollama']['host']}:{self.config['ollama']['port']}")
                print("=" * 63)
            else:
                self.get_logger().error("Failed to load Ollama configuration")
                self.ollama_mode = False
        
        # Create QoS profile for reliable communication
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.RELIABLE
        
        # Publishers and subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', qos_profile)
        self.laser_sub = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.laser_callback, 
            qos_profile
        )
        
        # Robot parameters
        self.linear_speed = 0.08  # Slow speed for safety (m/s)
        self.angular_speed = 0.5  # Turning speed (rad/s)
        self.stop_distance = 0.30  # Stop at 30cm (1 foot)
        self.safe_distance = 0.40  # Resume when clear at 40cm
        
        # State management
        self.state = "moving_forward"  # States: moving_forward, stopped_waiting, turning, moving_forward_manual
        self.turn_direction = 0  # 1 for left, -1 for right
        self.laser_data = None
        self.obstacle_detected = False
        self.path_clear = True
        self.user_decision_pending = False
        self.just_turned = False  # Flag to ignore side obstacles after turning
        self.forward_start_time = 0  # Track forward movement after turning
        
        # Movement tracking
        self.distance_traveled = 0.0  # Distance since last stop
        self.movement_start_time = time.time()  # When forward movement started
        self.last_movement_time = time.time()  # For incremental distance calculation
        
        # Turn tracking
        self.turn_start_time = 0
        self.min_turn_time = 1.0  # Minimum turn duration
        self.clear_readings_count = 0
        self.required_clear_readings = 3  # Need 3 consecutive clear readings
        self.total_rotation = 0.0  # Total radians rotated during turn
        self.turn_direction_text = ""  # LEFT, RIGHT, or AROUND
        
        # Manual forward movement tracking
        self.manual_forward_distance = 1.52  # 5 feet in meters
        self.emergency_stop_distance = 0.10  # 10cm emergency detection
        self.forward_target_distance = 0.0
        self.forward_start_distance = 0.0
        self.manual_forward_start_time = 0.0
        
        # Control timer - runs at 10Hz for smooth control
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        # Initial startup message (only output during startup)
        if not self.ollama_mode:
            self.get_logger().info("🔌 B4M Spatial Interpreter initialized")
            self.get_logger().info("   Robot will move forward until obstacle detected")
            self.get_logger().info("   Console will stay quiet during normal operation")
            print("\n✅ B4M SPATIAL INTERPRETER ACTIVE")
            print("=" * 63)
            print("🔌 Robot starting with B4M Spatial Interpreter\n")
        # After this, stay quiet until obstacle detected
    
    def load_ollama_config(self):
        """Load Ollama configuration from YAML file"""
        config_path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            'config', 'ollama_config.yaml'
        )
        
        try:
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
                return config
        except Exception as e:
            print(f"⚠️ Error loading Ollama config: {e}")
            return None
        
    def laser_callback(self, msg):
        """Process laser scan data for obstacle detection"""
        self.laser_data = msg
        
        # Analyze laser data for obstacles
        ranges = np.array(msg.ranges)
        
        # Handle infinite/invalid values
        ranges[np.isinf(ranges)] = msg.range_max
        ranges[np.isnan(ranges)] = msg.range_max
        ranges[ranges == 0.0] = msg.range_max
        
        # Calculate laser parameters
        num_readings = len(ranges)
        angle_range = msg.angle_max - msg.angle_min
        angle_increment = angle_range / (num_readings - 1) if num_readings > 1 else msg.angle_increment
        
        # Check for obstacles in ANY direction (front, left, right)
        # This implements "stop at EVERY obstacle" behavior
        
        # Front sector (±8 degrees) - narrower for more precise front detection
        front_angle_rad = math.radians(8)
        front_center_idx = int((0.0 - msg.angle_min) / angle_increment)
        front_half_width = int(front_angle_rad / angle_increment)
        front_start = max(0, front_center_idx - front_half_width)
        front_end = min(num_readings - 1, front_center_idx + front_half_width)
        front_ranges = ranges[front_start:front_end + 1]
        min_front_distance = np.min(front_ranges) if len(front_ranges) > 0 else msg.range_max
        
        # Left sector (67.5° to 112.5° - 45° arc centered at 90°)
        left_angle_start = math.radians(67.5)
        left_angle_end = math.radians(112.5)
        left_start_idx = max(0, int((left_angle_start - msg.angle_min) / angle_increment))
        left_end_idx = min(num_readings - 1, int((left_angle_end - msg.angle_min) / angle_increment))
        left_ranges = ranges[left_start_idx:left_end_idx + 1] if left_end_idx >= left_start_idx else []
        min_left_distance = np.min(left_ranges) if len(left_ranges) > 0 else msg.range_max
        
        # Right sector (-112.5° to -67.5° - 45° arc centered at -90°)
        right_angle_start = math.radians(-112.5)
        right_angle_end = math.radians(-67.5)
        right_start_idx = max(0, int((right_angle_start - msg.angle_min) / angle_increment))
        right_end_idx = min(num_readings - 1, int((right_angle_end - msg.angle_min) / angle_increment))
        right_ranges = ranges[right_start_idx:right_end_idx + 1] if right_end_idx >= right_start_idx else []
        min_right_distance = np.min(right_ranges) if len(right_ranges) > 0 else msg.range_max
        
        # Only stop for FRONT obstacles during forward movement
        # Side obstacles are detected and reported but don't stop forward motion
        self.obstacle_detected = min_front_distance <= self.stop_distance
        
        # Path is clear only when front has enough clearance
        self.path_clear = min_front_distance > self.safe_distance
        
    def analyze_spatial_context(self):
        """Analyze laser scan and generate spatial description"""
        if self.laser_data is None:
            return None
            
        ranges = np.array(self.laser_data.ranges)
        ranges[np.isinf(ranges)] = self.laser_data.range_max
        ranges[np.isnan(ranges)] = self.laser_data.range_max
        ranges[ranges == 0.0] = self.laser_data.range_max
        
        num_readings = len(ranges)
        angle_range = self.laser_data.angle_max - self.laser_data.angle_min
        angle_increment = angle_range / (num_readings - 1) if num_readings > 1 else self.laser_data.angle_increment
        
        # Analyze each sector
        sectors = {}
        
        # Front sector (±8°) - narrower for more precise front detection
        sectors['front'] = self.analyze_sector(ranges, -8, 8, self.laser_data.angle_min, angle_increment)
        
        # Left sector (67.5° to 112.5° - 45° arc centered at 90°)
        sectors['left'] = self.analyze_sector(ranges, 67.5, 112.5, self.laser_data.angle_min, angle_increment)
        
        # Right sector (-112.5° to -67.5° - 45° arc centered at -90°)
        sectors['right'] = self.analyze_sector(ranges, -112.5, -67.5, self.laser_data.angle_min, angle_increment)
        
        # Behind sector (150° to -150°)
        sectors['behind'] = self.analyze_sector_behind(ranges, self.laser_data.angle_min, angle_increment)
        
        return sectors
    
    def analyze_sector(self, ranges, start_angle_deg, end_angle_deg, angle_min, angle_increment):
        """Analyze a sector of laser scan data"""
        start_angle_rad = math.radians(start_angle_deg)
        end_angle_rad = math.radians(end_angle_deg)
        
        start_idx = max(0, int((start_angle_rad - angle_min) / angle_increment))
        end_idx = min(len(ranges) - 1, int((end_angle_rad - angle_min) / angle_increment))
        
        if start_idx > end_idx:
            start_idx, end_idx = end_idx, start_idx
            
        sector_ranges = ranges[start_idx:end_idx + 1]
        
        if len(sector_ranges) == 0:
            return {'status': 'NO_DATA', 'min': None, 'avg': None, 'description': 'No data'}
            
        min_dist = np.min(sector_ranges)
        avg_dist = np.mean(sector_ranges)
        
        # Determine status
        if min_dist < 0.3:
            status = 'BLOCKED'
            symbol = '⚠️'
        elif min_dist < 0.6:
            status = 'NARROW'
            symbol = '⚠️'
        else:
            status = 'CLEAR'
            symbol = '✅'
            
        # Generate description
        if status == 'BLOCKED':
            desc = f"{symbol} BLOCKED - Wall at {min_dist:.2f}m ({min_dist*39.37:.0f} inches)"
        elif status == 'NARROW':
            desc = f"{symbol} NARROW  - Wall at {min_dist:.2f}m ({min_dist*39.37:.0f} inches)"
        else:
            desc = f"{symbol} CLEAR   - Open space, nearest obstacle at {min_dist:.2f}m"
            
        return {
            'status': status,
            'min': min_dist,
            'avg': avg_dist,
            'description': desc
        }
    
    def analyze_sector_behind(self, ranges, angle_min, angle_increment):
        """Analyze behind sector (wraps around ±180°)"""
        # Behind is approximately 150° to -150° (through ±180°)
        left_behind_start = math.radians(150)
        left_behind_end = math.radians(180)
        right_behind_start = math.radians(-180)
        right_behind_end = math.radians(-150)
        
        # Get indices for both parts
        left_start_idx = max(0, int((left_behind_start - angle_min) / angle_increment))
        left_end_idx = min(len(ranges) - 1, int((left_behind_end - angle_min) / angle_increment))
        right_start_idx = max(0, int((right_behind_start - angle_min) / angle_increment))
        right_end_idx = min(len(ranges) - 1, int((right_behind_end - angle_min) / angle_increment))
        
        # Combine both parts
        behind_ranges = []
        if left_end_idx >= left_start_idx:
            behind_ranges.extend(ranges[left_start_idx:left_end_idx + 1])
        if right_end_idx >= right_start_idx:
            behind_ranges.extend(ranges[right_start_idx:right_end_idx + 1])
            
        if len(behind_ranges) == 0:
            return {'status': 'NO_DATA', 'min': None, 'avg': None, 'description': 'No data'}
            
        min_dist = np.min(behind_ranges)
        avg_dist = np.mean(behind_ranges)
        
        if min_dist < 0.3:
            status = 'BLOCKED'
            symbol = '⚠️'
        elif min_dist < 0.6:
            status = 'NARROW'
            symbol = '⚠️'
        else:
            status = 'CLEAR'
            symbol = '✅'
            
        if status == 'CLEAR':
            desc = f"{symbol} CLEAR   - Open space for at least {min_dist:.1f}m"
        else:
            desc = f"{symbol} {'BLOCKED' if status == 'BLOCKED' else 'NARROW'} - Obstacle at {min_dist:.2f}m"
            
        return {
            'status': status,
            'min': min_dist,
            'avg': avg_dist,
            'description': desc
        }
    
    def display_spatial_description(self, spatial_context):
        """Display formatted spatial description in console"""
        print("\n" + "=" * 63)
        print("🤖 B4M SPATIAL INTERPRETER - OBSTACLE DETECTED")
        print("=" * 63)
        
        # Display movement since last stop
        if self.distance_traveled > 0.01:  # Only show if moved more than 1cm
            print("\n📏 Movement Since Last Stop:")
            print("-" * 63)
            elapsed_time = time.time() - self.movement_start_time
            avg_speed = self.distance_traveled / elapsed_time if elapsed_time > 0 else 0
            print(f"• Distance traveled: {self.distance_traveled:.2f}m forward")
            print(f"• Time elapsed: {elapsed_time:.1f} seconds")
            print(f"• Average speed: {avg_speed:.3f} m/s")
        print("\n📍 Current Situation:")
        print("-" * 63)
        print(f"FRONT:  {spatial_context['front']['description']}")
        print(f"LEFT:   {spatial_context['left']['description']}")
        print(f"RIGHT:  {spatial_context['right']['description']}")
        print(f"BEHIND: {spatial_context['behind']['description']}")
        
        print("\n📊 Detailed Scan Analysis:")
        print("-" * 63)
        print(f"• Front sector (±15°):  Min: {spatial_context['front']['min']:.2f}m, Avg: {spatial_context['front']['avg']:.2f}m")
        print(f"• Left sector (67.5°-112.5°):   Min: {spatial_context['left']['min']:.2f}m, Avg: {spatial_context['left']['avg']:.2f}m")
        print(f"• Right sector (-112.5°--67.5°):  Min: {spatial_context['right']['min']:.2f}m, Avg: {spatial_context['right']['avg']:.2f}m")
        print(f"• Laser points: {len(self.laser_data.ranges)} readings covering 360°")
        
        print("\n🎯 Navigation Options:")
        print("-" * 63)
        print("1) Turn LEFT 90°")
        print("2) Turn RIGHT 90°")
        print("3) Turn AROUND 180°")
        print("4) Move FORWARD - Continue straight for 5 feet or until obstacle")
        print()
        
    def get_navigation_decision(self, spatial_context):
        """Get navigation decision from Ollama or manual input"""
        if self.ollama_mode and self.ollama_navigator:
            # Get Ollama decision
            print("\n🦙 Consulting Ollama for navigation decision...")
            print("   (Robot stopped while waiting for response)")
            
            decision, prompt = self.ollama_navigator.get_navigation_decision(spatial_context)
            
            # Show prompt that was sent to Ollama
            if prompt:
                print("\n📝 OLLAMA PROMPT:")
                print("-" * 63)
                # Display the prompt
                for line in prompt.split('\n')[:15]:  # Show first 15 lines
                    if line.strip():  # Only show non-empty lines
                        print(line[:63])  # Truncate long lines
                if len(prompt.split('\n')) > 15:
                    print("... [truncated for display]")
                print("-" * 63)
            
            if decision:
                print("\n✅ OLLAMA RESPONSE:")
                print("-" * 63)
                print(f"   Action: {decision['action'].upper()}")
                print(f"   Reason: {decision.get('reason', 'No reason provided')}")
                print(f"   Confidence: {decision.get('confidence', 0.5):.2f}")
                print("-" * 63)
                return self.map_ollama_to_choice(decision['action'])
            else:
                print("\n🛑 OLLAMA UNAVAILABLE - STOPPING")
                print("   Ollama did not respond within timeout period")
                print("   Robot stopping for safety")
                return None  # Stop the robot
        else:
            return self.get_user_decision()
    
    def map_ollama_to_choice(self, action):
        """Map Ollama action strings to choice numbers"""
        mapping = {
            "turn_left": 1,
            "turn_right": 2,
            "turn_around": 3,
            "go_straight": 4
        }
        return mapping.get(action, None)  # Return None if unknown action
    
    def get_user_decision(self):
        """Get navigation decision from user via blocking console input"""
        while True:
            try:
                choice = input("Please select action (1-4): ")
                if choice in ['1', '2', '3', '4']:
                    return int(choice)
                print("Invalid input. Please enter 1, 2, 3, or 4.")
            except KeyboardInterrupt:
                print("\n\n🛑 User requested stop - shutting down...")
                return None
            except EOFError:
                # Handle case where input stream is closed
                print("\n\n⚠️ Input stream closed - shutting down...")
                return None
    
    def execute_turn(self, turn_choice):
        """Execute the selected turn maneuver or manual forward movement"""
        if turn_choice == 1:  # Left 90°
            self.turn_direction = 1
            self.turn_direction_text = "LEFT"
            print(f"\n🔄 Executing turn {self.turn_direction_text}...")
            print("   (Will continue turning until path ahead is clear)")
            
            # Reset tracking for new turn
            self.state = "turning"
            self.turn_start_time = time.time()
            self.clear_readings_count = 0
            self.total_rotation = 0.0
            
        elif turn_choice == 2:  # Right 90°
            self.turn_direction = -1
            self.turn_direction_text = "RIGHT"
            print(f"\n🔄 Executing turn {self.turn_direction_text}...")
            print("   (Will continue turning until path ahead is clear)")
            
            # Reset tracking for new turn
            self.state = "turning"
            self.turn_start_time = time.time()
            self.clear_readings_count = 0
            self.total_rotation = 0.0
            
        elif turn_choice == 3:  # Turn around 180°
            self.turn_direction = 1  # Default to left for 180°
            self.turn_direction_text = "AROUND"
            print(f"\n🔄 Executing turn {self.turn_direction_text}...")
            print("   (Will continue turning until path ahead is clear)")
            
            # Reset tracking for new turn
            self.state = "turning"
            self.turn_start_time = time.time()
            self.clear_readings_count = 0
            self.total_rotation = 0.0
            
        else:  # Move forward (choice 4)
            self.execute_manual_forward()
            return
        
        # Reset distance tracking for next segment (for turns only)
        self.distance_traveled = 0.0
        self.movement_start_time = time.time()
        self.last_movement_time = time.time()
        
    def execute_manual_forward(self):
        """Execute manual forward movement for 5 feet or until obstacle detected"""
        print(f"\n➡️ Moving forward for 5 feet (1.52m) or until obstacle...")
        self.state = "moving_forward_manual"
        self.forward_target_distance = self.manual_forward_distance
        self.forward_start_distance = self.distance_traveled
        self.manual_forward_start_time = time.time()
        
        # Reset obstacle detection for manual movement
        self.obstacle_detected = False
        self.user_decision_pending = False
        
    def validate_manual_forward_completion(self):
        """Check if manual forward movement should complete"""
        if self.laser_data is None:
            return False
            
        # Get front distance for emergency stop check
        ranges = np.array(self.laser_data.ranges)
        ranges[np.isinf(ranges)] = self.laser_data.range_max
        ranges[np.isnan(ranges)] = self.laser_data.range_max
        ranges[ranges == 0.0] = self.laser_data.range_max
        
        num_readings = len(ranges)
        angle_range = self.laser_data.angle_max - self.laser_data.angle_min
        angle_increment = angle_range / (num_readings - 1) if num_readings > 1 else self.laser_data.angle_increment
        
        # Front sector (±15 degrees) for emergency detection
        front_angle_rad = math.radians(15)
        front_center_idx = int((0.0 - self.laser_data.angle_min) / angle_increment)
        front_half_width = int(front_angle_rad / angle_increment)
        front_start = max(0, front_center_idx - front_half_width)
        front_end = min(num_readings - 1, front_center_idx + front_half_width)
        front_ranges = ranges[front_start:front_end + 1]
        min_front_distance = np.min(front_ranges) if len(front_ranges) > 0 else self.laser_data.range_max
        
        # Check emergency obstacle detection (10cm)
        if min_front_distance <= self.emergency_stop_distance:
            elapsed_time = time.time() - self.manual_forward_start_time
            distance_moved = self.distance_traveled - self.forward_start_distance
            print(f"\n📐 Forward Movement Stopped:")
            print("-" * 63)
            print(f"• Emergency stop - Obstacle at {min_front_distance:.2f}m")
            print(f"• Distance traveled: {distance_moved:.2f}m ({distance_moved*3.28:.1f} feet)")
            print(f"• Movement duration: {elapsed_time:.1f} seconds")
            print(f"• Stopped by: Front obstacle at {min_front_distance:.2f}m")
            return True
        
        # Check if target distance reached
        distance_moved = self.distance_traveled - self.forward_start_distance
        if distance_moved >= self.forward_target_distance:
            elapsed_time = time.time() - self.manual_forward_start_time
            print(f"\n📐 Forward Movement Complete:")
            print("-" * 63)
            print(f"• Target distance reached: {self.forward_target_distance:.2f}m (5 feet)")
            print(f"• Movement duration: {elapsed_time:.1f} seconds")
            print(f"• Average speed: {self.forward_target_distance/elapsed_time:.3f} m/s")
            return True
        
        # Show progress update every 0.5m
        if int(distance_moved * 2) > int((distance_moved - 0.1) * 2):
            remaining = self.forward_target_distance - distance_moved
            print(f"📏 Progress: {distance_moved:.2f}m moved, {remaining:.2f}m remaining")
        
        return False
    
    def control_loop(self):
        """Main control loop for spatial interpreter"""
        if self.laser_data is None:
            return
            
        cmd = Twist()
        
        if self.state == "moving_forward":
            # Reset just_turned flag after 2 seconds of forward movement
            if self.just_turned and time.time() - self.forward_start_time > 2.0:
                self.just_turned = False
                
            if self.obstacle_detected and not self.user_decision_pending:
                # Stop and wait for user input
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.cmd_pub.publish(cmd)
                
                # Update last movement time before stopping
                self.last_movement_time = time.time()
                
                # Analyze and display spatial context
                spatial_context = self.analyze_spatial_context()
                if spatial_context:
                    self.display_spatial_description(spatial_context)
                    
                    # Set flag to prevent re-displaying and reset just_turned
                    self.user_decision_pending = True
                    self.state = "stopped_waiting"
                    self.just_turned = False  # Reset flag when handling new obstacle
                    
                    # Get navigation decision (from Ollama or user)
                    decision = self.get_navigation_decision(spatial_context)
                    if decision is None:
                        # Ollama failed or user wants to exit - stop robot
                        if self.ollama_mode:
                            # In Ollama mode, stop and wait
                            print("\n⚠️ System halted - manual intervention required")
                            self.state = "stopped_waiting"
                            self.user_decision_pending = False
                            return
                        else:
                            # In manual mode, shutdown
                            rclpy.shutdown()
                            return
                    
                    # Execute the turn
                    self.execute_turn(decision)
                    self.user_decision_pending = False
            else:
                # No obstacle or already handling - continue forward
                if not self.obstacle_detected:
                    cmd.linear.x = self.linear_speed
                    cmd.angular.z = 0.0
                    
                    # Track distance traveled
                    current_time = time.time()
                    time_delta = current_time - self.last_movement_time
                    self.distance_traveled += self.linear_speed * time_delta
                    self.last_movement_time = current_time
                else:
                    # Obstacle detected but decision pending
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.0
                    
        elif self.state == "stopped_waiting":
            # Waiting for user input - stay stopped
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            
        elif self.state == "turning":
            # Execute turn until path is clear
            cmd.linear.x = 0.0
            cmd.angular.z = self.angular_speed * self.turn_direction
            
            # Track rotation
            self.total_rotation += abs(self.angular_speed * 0.1)  # 0.1 is the timer period
            
            # Check if minimum turn time has elapsed
            turn_duration = time.time() - self.turn_start_time
            min_turn_elapsed = turn_duration >= self.min_turn_time
            
            # Check if path ahead is clear using LaserScan
            if self.path_clear:
                self.clear_readings_count += 1
            else:
                self.clear_readings_count = 0
                
            # Need both minimum time and clear readings
            if min_turn_elapsed and self.clear_readings_count >= self.required_clear_readings:
                # Get front distance for confirmation message
                if self.laser_data:
                    ranges = np.array(self.laser_data.ranges)
                    ranges[np.isinf(ranges)] = self.laser_data.range_max
                    ranges[np.isnan(ranges)] = self.laser_data.range_max
                    ranges[ranges == 0.0] = self.laser_data.range_max
                    
                    num_readings = len(ranges)
                    angle_range = self.laser_data.angle_max - self.laser_data.angle_min
                    angle_increment = angle_range / (num_readings - 1) if num_readings > 1 else self.laser_data.angle_increment
                    
                    front_angle_rad = math.radians(15)
                    front_center_idx = int((0.0 - self.laser_data.angle_min) / angle_increment)
                    front_half_width = int(front_angle_rad / angle_increment)
                    front_start = max(0, front_center_idx - front_half_width)
                    front_end = min(num_readings - 1, front_center_idx + front_half_width)
                    front_ranges = ranges[front_start:front_end + 1]
                    front_distance = np.min(front_ranges) if len(front_ranges) > 0 else 0
                    
                    # Display turn summary
                    total_degrees = math.degrees(self.total_rotation)
                    turn_duration = time.time() - self.turn_start_time
                    print(f"\n📐 Turn Complete:")
                    print("-" * 63)
                    print(f"• Total rotation: {total_degrees:.0f}° {self.turn_direction_text}")
                    print(f"• Turn duration: {turn_duration:.1f} seconds")
                    print(f"• Clear path detected ahead: {front_distance:.2f}m\n")
                
                # Resume forward movement (stay quiet)
                self.state = "moving_forward"
                self.obstacle_detected = False  # Reset for next obstacle
                self.just_turned = True  # Ignore side obstacles briefly after turning
                self.forward_start_time = time.time()  # Start timer for forward movement
            elif not min_turn_elapsed:
                # Still in minimum turn time
                pass
            elif not self.path_clear and turn_duration > 1.5:
                # Show continuing turn message occasionally
                if int(turn_duration * 10) % 10 == 0:  # Every second
                    if self.laser_data:
                        ranges = np.array(self.laser_data.ranges)
                        ranges[np.isinf(ranges)] = self.laser_data.range_max
                        ranges[np.isnan(ranges)] = self.laser_data.range_max
                        ranges[ranges == 0.0] = self.laser_data.range_max
                        
                        num_readings = len(ranges)
                        angle_range = self.laser_data.angle_max - self.laser_data.angle_min
                        angle_increment = angle_range / (num_readings - 1) if num_readings > 1 else self.laser_data.angle_increment
                        
                        front_angle_rad = math.radians(15)
                        front_center_idx = int((0.0 - self.laser_data.angle_min) / angle_increment)
                        front_half_width = int(front_angle_rad / angle_increment)
                        front_start = max(0, front_center_idx - front_half_width)
                        front_end = min(num_readings - 1, front_center_idx + front_half_width)
                        front_ranges = ranges[front_start:front_end + 1]
                        front_distance = np.min(front_ranges) if len(front_ranges) > 0 else 0
                        
                        print(f"   Continuing turn... front still blocked at {front_distance:.2f}m")
        
        elif self.state == "moving_forward_manual":
            # Manual forward movement for 5 feet or until obstacle
            if self.validate_manual_forward_completion():
                # Movement completed - return to normal operation
                self.state = "moving_forward"
                self.obstacle_detected = False
                self.user_decision_pending = False
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
            else:
                # Continue moving forward
                cmd.linear.x = self.linear_speed
                cmd.angular.z = 0.0
                
                # Track distance traveled
                current_time = time.time()
                time_delta = current_time - self.last_movement_time
                self.distance_traveled += self.linear_speed * time_delta
                self.last_movement_time = current_time
        
        # Publish velocity command
        self.cmd_pub.publish(cmd)

def main():
    """Main entry point for B4M Spatial Interpreter"""
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description='B4M Spatial Interpreter for robot navigation')
    parser.add_argument('--ollama-mode', action='store_true',
                       help='Enable Ollama LLM navigation mode')
    args, unknown = parser.parse_known_args()
    
    # Initialize ROS2
    rclpy.init(args=unknown)  # Pass remaining args to ROS2
    
    try:
        interpreter = B4MSpatialInterpreter(ollama_mode=args.ollama_mode)
        rclpy.spin(interpreter)
        
    except KeyboardInterrupt:
        print("\n\n🛑 Spatial Interpreter stopped by user")
        
    except Exception as e:
        print(f"\n❌ Spatial Interpreter error: {e}")
        import traceback
        traceback.print_exc()
        return 1
        
    finally:
        # Send stop command before shutting down
        if 'interpreter' in locals():
            stop_cmd = Twist()
            interpreter.cmd_pub.publish(stop_cmd)
            time.sleep(0.1)
            
        try:
            rclpy.shutdown()
        except:
            pass
    
    print("🎯 B4M Spatial Interpreter completed")
    return 0

if __name__ == '__main__':
    exit(main())