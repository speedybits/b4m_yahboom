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
        
        prompt = f"""Robot navigation decision needed. BLOCKED means impassable obstacle.

SENSORS:
{spatial_desc}

ACTIONS: turn_left, turn_right, turn_around, go_straight

RULE: If FRONT is BLOCKED, NEVER choose go_straight. Turn toward CLEAR direction.

Reply JSON only: {{"action":"<choice>","reason":"<why>","confidence":<0-1>}}"""
        
        return prompt
    
    def get_navigation_decision(self, spatial_context):
        """Get navigation decision from Ollama"""
        # ULTRA DEBUG: Track API call entry
        call_id = int(time.time() * 1000) % 10000  # Unique ID for this call
        print(f"[{get_timestamp()}] 🔥 OLLAMA API CALL #{call_id} STARTING")
        
        try:
            print(f"[{get_timestamp()}] 📋 Generating prompt for call #{call_id}...")
            prompt = self.generate_prompt(spatial_context)
            print(f"[{get_timestamp()}] 📋 Prompt generated for call #{call_id}, length: {len(prompt)} chars")
            
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
            
            # Measure response time
            print(f"[{get_timestamp()}] 🌍 Making HTTP POST request #{call_id} to {self.api_url}...")
            start_time = time.time()
            response = requests.post(
                self.api_url,
                json=payload,
                timeout=self.timeout
            )
            response_time = time.time() - start_time
            print(f"[{get_timestamp()}] 🌍 HTTP response #{call_id} received in {response_time:.3f}s, status: {response.status_code}")
            
            if response.status_code == 200:
                result = response.json()
                decision = json.loads(result['response'])
                
                # Validate response
                if self.validate_response(decision):
                    return decision, prompt, response_time
                else:
                    return None, prompt, response_time
            else:
                return None, prompt, response_time
                
        except (requests.Timeout, requests.RequestException, json.JSONDecodeError) as e:
            print(f"[{get_timestamp()}] ⚠️ OLLAMA API CALL #{call_id} FAILED: {type(e).__name__}: {str(e)}")
            return None, None, None
    
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

import sys
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
from datetime import datetime

def get_timestamp():
    """Get formatted timestamp for debug output"""
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]  # HH:MM:SS.mmm

def print_flush(*args, **kwargs):
    """Print with immediate flush to ensure real-time output"""
    print(*args, **kwargs)
    sys.stdout.flush()

class B4MSpatialInterpreter(Node):
    def __init__(self, ollama_mode=False):
        super().__init__('b4m_spatial_interpreter')
        
        # Check if running in Ollama mode
        self.ollama_mode = ollama_mode
        self.ollama_navigator = None
        self.debug_verbose = False  # Disable verbose debug output by default
        
        if self.ollama_mode:
            # Load Ollama configuration
            self.config = self.load_ollama_config()
            if self.config:
                self.ollama_navigator = OllamaNavigator(self.config)
                self.get_logger().info("🦙 Ollama mode activated")
                print(f"\n[{get_timestamp()}] 🦙 OLLAMA MODE ACTIVATED")
                print("=" * 63)
                print(f"[{get_timestamp()}] Robot will use Ollama LLM for navigation decisions")
                print(f"[{get_timestamp()}] Model: {self.config['ollama']['model']}")
                print(f"[{get_timestamp()}] API: {self.config['ollama']['host']}:{self.config['ollama']['port']}")
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
        self.stop_distance = 0.28  # Stop at 28cm to avoid noise flicker
        self.clear_distance = 0.32  # Clear at 32cm to avoid noise flicker 
        self.safe_distance = 0.40  # Resume when clear at 40cm
        
        # State management
        self.state = "moving_forward"  # States: moving_forward, stopped_waiting, turning, moving_forward_manual
        self.turn_direction = 0  # 1 for left, -1 for right
        self.laser_data = None
        self.obstacle_detected = False
        self.path_clear = True
        self.user_decision_pending = False
        self.last_ollama_call_time = 0  # Rate limit Ollama calls
        self.min_ollama_interval = 2.0  # Minimum 2 seconds between Ollama calls
        self.last_progress_update_time = 0  # For 10-second progress updates
        self.progress_update_interval = 10.0  # Progress update every 10 seconds
        self.distance_at_last_progress = 0.0  # Distance when last progress was shown
        self.processing_obstacle = False  # Flag to prevent re-entry during obstacle processing
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
        self.get_logger().info("🔌 B4M Spatial Interpreter initialized")
        self.get_logger().info("   Robot will move forward until obstacle detected")
        self.get_logger().info("   Console will stay quiet during normal operation")
        if not self.ollama_mode:
            print("\n✅ B4M SPATIAL INTERPRETER ACTIVE")
            print("=" * 63)
            print("🔌 Robot starting with B4M Spatial Interpreter\n")
        else:
            print_flush("\n🦙 OLLAMA MODE ACTIVATED")
            print_flush("=" * 63)
            print_flush("Robot will use Ollama LLM for navigation decisions")
            print_flush(f"Model: {self.config['ollama']['model']}")
            print_flush(f"API: {self.config['ollama']['host']}:{self.config['ollama']['port']}")
            print_flush("=" * 63)
            print_flush(f"\n[{get_timestamp()}] 🤖 B4M Spatial Interpreter started")
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
        # ULTRA DEBUG: Track every laser callback
        if hasattr(self, '_laser_callback_count'):
            self._laser_callback_count += 1
        else:
            self._laser_callback_count = 1
            
        if self._laser_callback_count % 100 == 1:  # Log every 100th callback
            print(f"[{get_timestamp()}] 🔍 LASER CALLBACK #{self._laser_callback_count} - State: {self.state}, Obstacle: {self.obstacle_detected}, Processing: {getattr(self, 'processing_obstacle', 'UNDEFINED')}")
            
        self.laser_data = msg
        
        # Debug: Log first callback in Ollama mode
        if self.ollama_mode and not hasattr(self, '_first_scan_logged'):
            self._first_scan_logged = True
            self.get_logger().info(f"First laser scan received, {len(msg.ranges)} points")
            print_flush(f"[{get_timestamp()}] 📡 Laser scan data received - robot moving forward")
        
        # Analyze laser data for obstacles
        ranges = np.array(msg.ranges)
        
        # Handle infinite/invalid values
        ranges[np.isinf(ranges)] = msg.range_max
        ranges[np.isnan(ranges)] = msg.range_max
        # Filter out invalid readings: 0.0 and readings at/below range_min are sensor noise
        ranges[ranges == 0.0] = msg.range_max  # Invalid reading - treat as clear
        ranges[ranges <= msg.range_min] = msg.range_max  # Below sensor min range - treat as clear
        
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
        
        # Only stop for FRONT obstacles during forward movement with hysteresis
        # Side obstacles are detected and reported but don't stop forward motion
        prev_detected = self.obstacle_detected
        
        # Use hysteresis to prevent flicker: different thresholds for detection vs clearing
        if not self.obstacle_detected:
            # Currently clear - detect obstacle only if distance drops below stop_distance
            self.obstacle_detected = min_front_distance <= self.stop_distance
        else:
            # Currently detected - clear obstacle only if distance rises above clear_distance  
            self.obstacle_detected = min_front_distance <= self.clear_distance
        
        # ULTRA DEBUG: Track obstacle detection changes
        if self.obstacle_detected != prev_detected:
            print(f"[{get_timestamp()}] 🚨 OBSTACLE DETECTION CHANGED! Was: {prev_detected}, Now: {self.obstacle_detected}, Front: {min_front_distance:.3f}m, Processing: {getattr(self, 'processing_obstacle', 'UNDEFINED')}")
        
        # Simple obstacle detection - no debouncing to avoid infinite loops
        if self.debug_verbose and self.obstacle_detected != prev_detected:
            print(f"[{get_timestamp()}] 🚨 Obstacle detection changed! Now: {self.obstacle_detected}, Front dist: {min_front_distance:.2f}m")
        
        # Path is clear only when front has enough clearance
        self.path_clear = min_front_distance > self.safe_distance
        
    def analyze_spatial_context(self):
        """Analyze laser scan and generate spatial description"""
        if self.laser_data is None:
            return None
            
        ranges = np.array(self.laser_data.ranges)
        ranges[np.isinf(ranges)] = self.laser_data.range_max
        ranges[np.isnan(ranges)] = self.laser_data.range_max
        # Filter out invalid readings: 0.0 and readings at/below range_min are sensor noise
        ranges[ranges == 0.0] = self.laser_data.range_max  # Invalid reading - treat as clear
        ranges[ranges <= self.laser_data.range_min] = self.laser_data.range_max  # Below sensor min range - treat as clear
        
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
        print(f"\n[{get_timestamp()}] " + "=" * 63)
        print(f"[{get_timestamp()}] 🤖 B4M SPATIAL INTERPRETER - OBSTACLE DETECTED")
        print(f"[{get_timestamp()}] " + "=" * 63)
        
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
            print(f"\n[{get_timestamp()}] 🦙 Consulting Ollama for navigation decision...")
            print(f"[{get_timestamp()}]    (Robot stopped while waiting for response)")
            
            decision, prompt, response_time = self.ollama_navigator.get_navigation_decision(spatial_context)
            
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
                print(f"\n✅ OLLAMA RESPONSE: (received in {response_time:.1f}s)")
                print("-" * 63)
                print(f"   Action: {decision['action'].upper()}")
                print(f"   Reason: {decision.get('reason', 'No reason provided')}")
                print(f"   Confidence: {decision.get('confidence', 0.5):.2f}")
                print("-" * 63)
                return self.map_ollama_to_choice(decision['action'])
            else:
                print(f"\n🛑 OLLAMA UNAVAILABLE - STOPPING")
                if response_time:
                    print(f"   Ollama did not respond within timeout period ({response_time:.1f}s)")
                else:
                    print("   Ollama did not respond within timeout period")
                print("   Robot stopping for safety")
                print("   ")
                print("⚠️ MANUAL INTERVENTION REQUIRED")
                print("   Please check Ollama service status or restart in manual mode")
                print("   Use Ctrl+C to exit, then restart without --ollama flag")
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
        # Filter out invalid readings: 0.0 and readings at/below range_min are sensor noise
        ranges[ranges == 0.0] = self.laser_data.range_max  # Invalid reading - treat as clear
        ranges[ranges <= self.laser_data.range_min] = self.laser_data.range_max  # Below sensor min range - treat as clear
        
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
        # ULTRA DEBUG: Track control loop iterations
        if not hasattr(self, '_control_loop_count'):
            self._control_loop_count = 0
        self._control_loop_count += 1
        
        # Log every 50th iteration (every 5 seconds at 10Hz) OR when processing obstacle
        if (self._control_loop_count % 50 == 1) or getattr(self, 'processing_obstacle', False):
            print(f"[{get_timestamp()}] 🔄 CONTROL LOOP #{self._control_loop_count} - State: {self.state}, Obstacle: {self.obstacle_detected}, Processing: {getattr(self, 'processing_obstacle', False)}, Pending: {self.user_decision_pending}")
        
        if self.laser_data is None:
            if self._control_loop_count % 50 == 1:
                print(f"[{get_timestamp()}] ⚠️ No laser data available in control loop #{self._control_loop_count}")
            return
            
        # Progress updates during free movement (every 10 seconds)
        current_time = time.time()
        if (self.state == "moving_forward" and not self.obstacle_detected and 
            not self.user_decision_pending and 
            current_time - self.last_progress_update_time >= self.progress_update_interval):
            
            # Only show progress if we've actually moved
            distance_since_last = self.distance_traveled - self.distance_at_last_progress
            if distance_since_last > 0.1:  # At least 10cm of movement
                print_flush(f"[{get_timestamp()}] 📏 Progress: {self.distance_traveled:.1f}m traveled, continuing forward")
                self.last_progress_update_time = current_time
                self.distance_at_last_progress = self.distance_traveled
        
        # Debug: Log control loop state periodically (only if verbose debug enabled)
        if self.debug_verbose:
            if not hasattr(self, '_loop_count'):
                self._loop_count = 0
            self._loop_count += 1
            if self._loop_count % 100 == 0:  # Every 10 seconds at 10Hz
                print(f"[{get_timestamp()}] 🔄 DEBUG: Control loop active - State: {self.state}, Obstacle: {self.obstacle_detected}, Pending: {self.user_decision_pending}")
        
        cmd = Twist()
        
        if self.state == "moving_forward":
            # Reset just_turned flag after 2 seconds of forward movement
            if self.just_turned and time.time() - self.forward_start_time > 2.0:
                self.just_turned = False
                
            if self.obstacle_detected and not self.user_decision_pending and not self.processing_obstacle:
                # ULTRA DEBUG: Log entry into obstacle processing
                print(f"[{get_timestamp()}] 🔥 ENTERING OBSTACLE PROCESSING - State: {self.state}, Flags: obstacle={self.obstacle_detected}, pending={self.user_decision_pending}, processing={self.processing_obstacle}")
                
                # Set processing flag to prevent re-entry
                self.processing_obstacle = True
                print(f"[{get_timestamp()}] 🔒 PROCESSING FLAG SET TO TRUE")
                
                # Announce obstacle detection
                print(f"[{get_timestamp()}] 🚨 Obstacle detected - stopping for analysis")
                if self.debug_verbose:
                    print(f"\n[{get_timestamp()}] 🔍 DEBUG: Obstacle detected! State: {self.state}, Decision pending: {self.user_decision_pending}")
                # Stop and wait for user input
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.cmd_pub.publish(cmd)
                
                # Update last movement time before stopping
                self.last_movement_time = time.time()
                
                # Analyze and display spatial context
                print(f"[{get_timestamp()}] 🔍 DEBUG: Analyzing spatial context...")
                spatial_context = self.analyze_spatial_context()
                print(f"[{get_timestamp()}] 🔍 DEBUG: Spatial context result: {spatial_context is not None}")
                if spatial_context:
                    print(f"[{get_timestamp()}] 🔍 DEBUG: Displaying spatial description...")
                    self.display_spatial_description(spatial_context)
                    
                    # Set flag to prevent re-displaying and reset just_turned
                    self.user_decision_pending = True
                    self.state = "stopped_waiting"
                    self.just_turned = False  # Reset flag when handling new obstacle
                    
                    # Rate limit Ollama calls to prevent system overload
                    current_time = time.time()
                    if self.ollama_mode:
                        time_since_last_call = current_time - self.last_ollama_call_time
                        if time_since_last_call < self.min_ollama_interval:
                            print(f"[{get_timestamp()}] ⏰ Rate limiting: Waiting {self.min_ollama_interval - time_since_last_call:.1f}s before next Ollama call")
                            self.processing_obstacle = False  # Reset flag
                            return  # Skip this cycle to rate limit
                        self.last_ollama_call_time = current_time
                    
                    # Get navigation decision (from Ollama or user) with timeout protection
                    try:
                        print(f"[{get_timestamp()}] 🔥 ABOUT TO CALL OLLAMA - Current time: {time.time():.3f}")
                        print(f"[{get_timestamp()}] 🦙 Calling Ollama API... (timeout={self.config['ollama']['timeout']}s)")
                        start_time = time.time()
                        decision = self.get_navigation_decision(spatial_context)
                        end_time = time.time()
                        print(f"[{get_timestamp()}] 💬 Ollama call completed in {end_time - start_time:.3f}s, decision: {decision}")
                        
                        if decision is None:
                            # Ollama failed or user wants to exit - stop robot
                            if self.ollama_mode:
                                # In Ollama mode, stop and wait
                                print("\n⚠️ System halted - manual intervention required")
                                self.state = "stopped_waiting"
                                self.user_decision_pending = False
                                self.processing_obstacle = False
                                return
                            else:
                                # In manual mode, shutdown
                                self.processing_obstacle = False
                                rclpy.shutdown()
                                return
                        
                        # Execute the turn
                        print(f"[{get_timestamp()}] ⚙️ Executing turn decision: {decision}")
                        self.execute_turn(decision)
                        self.user_decision_pending = False
                        self.processing_obstacle = False
                        
                    except Exception as e:
                        print(f"[{get_timestamp()}] ⚠️ Critical error in navigation decision: {e}")
                        import traceback
                        traceback.print_exc()
                        self.processing_obstacle = False
                        self.user_decision_pending = False
                        return
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
                    ranges[ranges == 0.0] = self.laser_data.range_min  # 0.0 means too close!
                    
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
                    
                    # Display turn completion messages matching documentation format
                    print(f"\n[{get_timestamp()}] ✅ Turn complete - path ahead is clear")
                
                # Resume forward movement
                self.state = "moving_forward"
                self.obstacle_detected = False  # Reset for next obstacle
                self.just_turned = True  # Ignore side obstacles briefly after turning
                self.forward_start_time = time.time()  # Start timer for forward movement
                print(f"[{get_timestamp()}] ➡️ Resuming forward movement")
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
                        ranges[ranges == 0.0] = self.laser_data.range_min  # 0.0 means too close!
                        
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
        
        # Debug: Show when movement state changes (only if verbose debug enabled)
        if self.debug_verbose:
            if not hasattr(self, '_last_cmd'):
                self._last_cmd = {'linear': 0, 'angular': 0}
            if cmd.linear.x != self._last_cmd['linear'] or cmd.angular.z != self._last_cmd['angular']:
                if cmd.linear.x > 0:
                    print(f"[{get_timestamp()}] 🤖 DEBUG: Moving forward at {cmd.linear.x:.3f} m/s")
                elif cmd.angular.z != 0:
                    print(f"[{get_timestamp()}] 🤖 DEBUG: Turning at {cmd.angular.z:.3f} rad/s")
                elif cmd.linear.x == 0 and cmd.angular.z == 0:
                    print(f"[{get_timestamp()}] 🤖 DEBUG: Stopped - State: {self.state}")
                self._last_cmd = {'linear': cmd.linear.x, 'angular': cmd.angular.z}

def main():
    """Main entry point for B4M Spatial Interpreter"""
    # Force unbuffered output for real-time console display
    sys.stdout = sys.__stdout__
    sys.stderr = sys.__stderr__
    
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description='B4M Spatial Interpreter for robot navigation')
    parser.add_argument('--ollama-mode', action='store_true',
                       help='Enable Ollama LLM navigation mode')
    parser.add_argument('--debug-verbose', action='store_true',
                       help='Enable verbose debug output')
    args, unknown = parser.parse_known_args()
    
    # Initialize ROS2
    rclpy.init(args=unknown)  # Pass remaining args to ROS2
    
    try:
        interpreter = B4MSpatialInterpreter(ollama_mode=args.ollama_mode)
        if args.debug_verbose:
            interpreter.debug_verbose = True
            print(f"[{get_timestamp()}] 📢 Verbose debug output enabled")
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