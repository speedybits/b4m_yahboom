#!/usr/bin/env python3
"""
B4M LiDAR Navigator - Intelligent navigation using LiDAR and B4M API

This node provides intelligent autonomous navigation by analyzing LiDAR data
and requesting turn directions from the B4M API when obstacles are detected.

Author: B4M Robot System
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Float32
from std_srvs.srv import SetBool
import numpy as np
import math
import time
import requests
import json
import logging
import os
from datetime import datetime
from typing import Optional


class B4MLidarAPI:
    """API client for B4M navigation decisions"""
    
    def __init__(self, logger=None):
        self.chat_url = "https://app.bike4mind.com/api/chat"
        self.session_create_url = "https://app.bike4mind.com/api/sessions/create"
        
        # Get API key from environment variable
        self.api_key = os.environ.get('B4M_API_KEY')
        if not self.api_key:
            if logger:
                logger.error("B4M_API_KEY environment variable not set!")
            raise RuntimeError("B4M_API_KEY environment variable not set! Please set it: export B4M_API_KEY='your_key_here'")
            
        self.timeout = 30.0  # 30 second timeout for chat requests
        self.logger = logger
        self.api_call_count = 0
        self.session_id = None
        
        # Set up dedicated API log file
        log_dir = "/tmp/b4m_lidar_logs"
        os.makedirs(log_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.api_log_file = f"{log_dir}/b4m_api_requests_{timestamp}.log"
        
        # Create API logger
        self.api_logger = logging.getLogger('b4m_api')
        self.api_logger.setLevel(logging.INFO)
        
        # Remove existing handlers to avoid duplicates
        for handler in self.api_logger.handlers[:]:
            self.api_logger.removeHandler(handler)
        
        # File handler for API requests
        file_handler = logging.FileHandler(self.api_log_file)
        file_handler.setLevel(logging.INFO)
        formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
        file_handler.setFormatter(formatter)
        self.api_logger.addHandler(file_handler)
        
        if self.logger:
            self.logger.info(f"API logging enabled: {self.api_log_file}")
        
        # Create B4M Robot session
        self._create_session()
    
    def _create_session(self):
        """Create a B4M Robot session for navigation decisions"""
        self.api_logger.info("="*80)
        self.api_logger.info("SESSION_CREATION - Creating B4M Robot Session")
        
        headers = {
            'X-API-Key': self.api_key,
            'Content-Type': 'application/json'
        }
        data = {
            'name': 'B4M_Robot_Session'
        }
        
        try:
            self.api_logger.info(f"SESSION_CREATION - Sending POST to {self.session_create_url}")
            self.api_logger.info(f"SESSION_CREATION - Request Data: {json.dumps(data, indent=2)}")
            
            response = requests.post(self.session_create_url, headers=headers, json=data, timeout=10.0)
            self.api_logger.info(f"SESSION_CREATION - HTTP Status: {response.status_code}")
            
            if response.status_code == 200:
                result = response.json()
                self.session_id = result.get('id') or result.get('_id')
                self.api_logger.info(f"SESSION_CREATION - Session created successfully: {self.session_id}")
                if self.logger:
                    self.logger.info(f"B4M Robot session created: {self.session_id}")
            else:
                self.api_logger.error(f"SESSION_CREATION - Failed with status {response.status_code}: {response.text}")
                if self.logger:
                    self.logger.error(f"Failed to create B4M session: {response.status_code}")
                    
        except Exception as e:
            self.api_logger.error(f"SESSION_CREATION - Exception: {e}")
            if self.logger:
                self.logger.error(f"Session creation error: {e}")
        
        self.api_logger.info("="*80)
        
    def get_turn_direction(self, obstacle_description: str) -> Optional[int]:
        """
        Query B4M API for turn direction based on obstacle description.
        
        Returns:
            1 for left turn (positive angular velocity)
            -1 for right turn (negative angular velocity)
            0 for stop (unclear response)
            None for API failure
        """
        self.api_call_count += 1
        call_id = f"API_CALL_{self.api_call_count}"
        
        headers = {
            'X-API-Key': self.api_key,
            'Content-Type': 'application/json'
        }
        
        data = {
            'message': f'I am a mobile robot navigating with LiDAR. Current situation: {obstacle_description}. Should I turn left or right? Respond with only "left" or "right".',
            'model': 'gpt-4o-mini',
            'temperature': 0.1,
            'max_tokens': 20
        }
        
        # Add session ID if available
        if self.session_id:
            data['sessionId'] = self.session_id
        
        # Terminal output for user visibility
        print(f"📝 Notebook: B4M_Robot_Session")
        print(f"💬 Message: {data['message']}")
        print(f"🌐 Sending to Bike4Mind API...")
        
        # Log the request
        self.api_logger.info("="*80)
        self.api_logger.info(f"{call_id} - REQUEST START")
        self.api_logger.info(f"URL: {self.chat_url}")
        self.api_logger.info(f"Headers: {json.dumps(headers, indent=2)}")
        self.api_logger.info(f"Request Body: {json.dumps(data, indent=2)}")
        self.api_logger.info(f"Obstacle Description: {obstacle_description}")
        self.api_logger.info(f"Session ID: {self.session_id}")
        
        request_time = time.time()
        
        try:
            self.api_logger.info(f"{call_id} - Sending HTTP POST request...")
            response = requests.post(
                self.chat_url,
                headers=headers,
                json=data,
                timeout=self.timeout
            )
            
            response_time = time.time() - request_time
            self.api_logger.info(f"{call_id} - Response received in {response_time:.3f}s")
            self.api_logger.info(f"{call_id} - HTTP Status Code: {response.status_code}")
            
            # Log raw response
            try:
                response_text_raw = response.text
                self.api_logger.info(f"{call_id} - Raw Response Text: {response_text_raw}")
            except Exception as e:
                self.api_logger.error(f"{call_id} - Could not read response text: {e}")
            
            # Check for successful response
            if response.status_code == 200:
                try:
                    result = response.json()
                    self.api_logger.info(f"{call_id} - Parsed JSON Response: {json.dumps(result, indent=2)}")
                    
                    # Check if this is a queued response
                    if result.get('status') == 'queued':
                        quest_id = result.get('id') or result.get('tracking_info', {}).get('quest_id')
                        self.api_logger.info(f"{call_id} - Response queued with ID: {quest_id}")
                        self.api_logger.warning(f"{call_id} - Quest polling not available - treating as timeout")
                        
                        # Terminal output for queued response
                        print(f"⏳ API Response: Message queued (ID: {quest_id})")
                        print(f"⚠️ Quest polling unavailable - robot will stop for safety")
                        
                        if self.logger:
                            self.logger.warning(f"API response queued but polling unavailable - stopping for safety")
                        return None
                    
                    # Direct response (older API format)
                    response_text = result.get('response', '').lower()
                    self.api_logger.info(f"{call_id} - Extracted Response Text: '{response_text}'")
                    
                    if 'left' in response_text:
                        decision = 1  # Turn left
                        decision_text = "TURN LEFT"
                        self.api_logger.info(f"{call_id} - DECISION: {decision_text} (return value: {decision})")
                        if self.logger:
                            self.logger.info(f"API Response: '{response_text}' -> {decision_text}")
                        return decision
                    elif 'right' in response_text:
                        decision = -1  # Turn right
                        decision_text = "TURN RIGHT"
                        self.api_logger.info(f"{call_id} - DECISION: {decision_text} (return value: {decision})")
                        if self.logger:
                            self.logger.info(f"API Response: '{response_text}' -> {decision_text}")
                        return decision
                    else:
                        # Unclear response - stop
                        decision = 0
                        decision_text = "UNCLEAR - STOP"
                        self.api_logger.warning(f"{call_id} - DECISION: {decision_text} (return value: {decision})")
                        if self.logger:
                            self.logger.warning(f"Unclear API response: '{response_text}' -> STOP")
                        return decision
                        
                except json.JSONDecodeError as e:
                    self.api_logger.error(f"{call_id} - JSON Parse Error: {e}")
                    if self.logger:
                        self.logger.error(f"Failed to parse API response: {e}")
                    return None
                    
            elif response.status_code == 401:
                self.api_logger.error(f"{call_id} - AUTHENTICATION FAILED - Invalid API key")
                if self.logger:
                    self.logger.error("API authentication failed - invalid API key")
                return None
            elif response.status_code == 429:
                self.api_logger.warning(f"{call_id} - RATE LIMIT EXCEEDED")
                if self.logger:
                    self.logger.warning("API rate limit exceeded")
                return None
            else:
                self.api_logger.error(f"{call_id} - HTTP ERROR: Status {response.status_code}")
                self.api_logger.error(f"{call_id} - Error Response Body: {response.text}")
                if self.logger:
                    self.logger.error(f"API error: {response.status_code}")
                return None
                
        except requests.exceptions.Timeout as e:
            self.api_logger.error(f"{call_id} - REQUEST TIMEOUT: {e}")
            if self.logger:
                self.logger.warning(f"API request timed out after {self.timeout}s")
            return None
        except requests.exceptions.RequestException as e:
            self.api_logger.error(f"{call_id} - REQUEST EXCEPTION: {e}")
            if self.logger:
                self.logger.error(f"API request failed: {e}")
            return None
        finally:
            self.api_logger.info(f"{call_id} - REQUEST END")
            self.api_logger.info("="*80)


class B4MLidarNavigator(Node):
    """ROS2 node for B4M LiDAR-based intelligent navigation"""
    
    def __init__(self):
        super().__init__('b4m_lidar_navigator')
        
        # Declare parameters
        # Note: use_sim_time is a global parameter and should not be declared by the node
        self.declare_parameter('api_endpoint', 'https://app.bike4mind.com/api/chat')
        self.declare_parameter('api_key', 'b4m_live_c491719bd23cc716e2db2c5182f4f900')
        self.declare_parameter('api_timeout', 2.0)
        self.declare_parameter('api_cooldown', 20.0)
        self.declare_parameter('linear_speed', 0.08)
        self.declare_parameter('angular_speed', 0.3)
        self.declare_parameter('stop_distance', 0.3048)
        self.declare_parameter('safe_distance', 0.4)
        self.declare_parameter('min_turn_time', 1.0)
        self.declare_parameter('required_clear_readings', 3)
        self.declare_parameter('enable_api', True)
        self.declare_parameter('debug_mode', False)
        
        # Get parameters
        # Get use_sim_time from global parameters (it's automatically set by ROS2)
        try:
            self.use_sim_time = self.get_parameter('use_sim_time').value
        except:
            # If not set, default to False
            self.use_sim_time = False
        self.api_cooldown_seconds = self.get_parameter('api_cooldown').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.stop_distance = self.get_parameter('stop_distance').value
        self.safe_distance = self.get_parameter('safe_distance').value
        self.min_turn_time = self.get_parameter('min_turn_time').value
        self.required_clear_readings = self.get_parameter('required_clear_readings').value
        self.enable_api = self.get_parameter('enable_api').value
        self.debug_mode = self.get_parameter('debug_mode').value
        
        # Initialize API client
        self.api_client = B4MLidarAPI(logger=self.get_logger())
        
        # API cooldown management
        self.last_api_call_time = 0
        self.waiting_for_cooldown = False
        self.cooldown_stop_logged = False
        
        # Create QoS profile for reliable communication
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.RELIABLE
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', qos_profile)
        self.status_pub = self.create_publisher(String, '/b4m_lidar/status', qos_profile)
        self.obstacle_info_pub = self.create_publisher(String, '/b4m_lidar/obstacle_info', qos_profile)
        self.cooldown_pub = self.create_publisher(Float32, '/b4m_lidar/api_cooldown', qos_profile)
        
        # Subscribers
        self.laser_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            qos_profile
        )
        
        self.command_sub = self.create_subscription(
            String,
            '/b4m_lidar/command',
            self.command_callback,
            qos_profile
        )
        
        # Services
        self.enable_srv = self.create_service(
            SetBool,
            '/b4m_lidar/enable',
            self.enable_callback
        )
        
        self.api_mode_srv = self.create_service(
            SetBool,
            '/b4m_lidar/set_api_mode',
            self.api_mode_callback
        )
        
        # State variables
        self.laser_data = None
        self.obstacle_detected = False
        self.front_clear = False
        self.turn_direction = 0  # 0 = stop, 1 = left, -1 = right
        self.navigation_state = "waiting"  # waiting, moving, turning, stopped
        self.enabled = True
        self.obstacle_description = ""
        
        # Turn control
        self.turn_start_time = 0
        self.clear_readings_count = 0
        self.api_decision_pending = False
        
        # Control timer - runs at 10Hz for smooth control
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        # Status timer - every 2 seconds
        self.status_timer = self.create_timer(2.0, self.publish_status)
        
        # Cooldown timer - every second
        self.cooldown_timer = self.create_timer(1.0, self.publish_cooldown)
        
        self.get_logger().info("🤖 B4M LiDAR Navigator initialized")
        self.get_logger().info(f"   Mode: {'Simulation' if self.use_sim_time else 'Real Robot'}")
        self.get_logger().info(f"   API mode: {'Enabled' if self.enable_api else 'Disabled'}")
        self.get_logger().info(f"   API cooldown: {self.api_cooldown_seconds} seconds")
        self.get_logger().info(f"   Stop distance: {self.stop_distance} m")
        self.get_logger().info(f"   Safe distance: {self.safe_distance} m")
        self.get_logger().info("   Waiting for laser scan data...")
    
    def laser_callback(self, msg):
        """Process laser scan data for obstacle detection"""
        self.laser_data = msg
        
        # Analyze laser data
        ranges = np.array(msg.ranges)
        
        # Handle invalid values
        ranges[np.isinf(ranges)] = msg.range_max
        ranges[np.isnan(ranges)] = msg.range_max
        ranges[ranges == 0.0] = msg.range_max
        
        # Calculate laser parameters
        num_readings = len(ranges)
        angle_range = msg.angle_max - msg.angle_min
        angle_increment = angle_range / (num_readings - 1) if num_readings > 1 else msg.angle_increment
        
        # Analyze different regions
        front_center_index = int((0.0 - msg.angle_min) / angle_increment)
        
        # Front sector (±15 degrees)
        front_angle_rad = math.radians(15)
        front_half_width = int(front_angle_rad / angle_increment)
        front_start = max(0, front_center_index - front_half_width)
        front_end = min(num_readings - 1, front_center_index + front_half_width)
        front_ranges = ranges[front_start:front_end + 1]
        
        # Left sector (-90 to -30 degrees)
        left_angle_start = math.radians(-90)
        left_angle_end = math.radians(-30)
        left_start_idx = max(0, int((left_angle_start - msg.angle_min) / angle_increment))
        left_end_idx = min(num_readings - 1, int((left_angle_end - msg.angle_min) / angle_increment))
        left_ranges = ranges[left_start_idx:left_end_idx + 1] if left_end_idx >= left_start_idx else []
        
        # Right sector (+30 to +90 degrees)
        right_angle_start = math.radians(30)
        right_angle_end = math.radians(90)
        right_start_idx = max(0, int((right_angle_start - msg.angle_min) / angle_increment))
        right_end_idx = min(num_readings - 1, int((right_angle_end - msg.angle_min) / angle_increment))
        right_ranges = ranges[right_start_idx:right_end_idx + 1] if right_end_idx >= right_start_idx else []
        
        # Calculate minimum distances
        min_front = np.min(front_ranges) if len(front_ranges) > 0 else msg.range_max
        min_left = np.min(left_ranges) if len(left_ranges) > 0 else msg.range_max
        min_right = np.min(right_ranges) if len(right_ranges) > 0 else msg.range_max
        
        # Determine obstacle status
        self.obstacle_detected = min_front <= self.stop_distance
        self.front_clear = min_front > self.safe_distance
        
        # Generate obstacle description with distances
        if min_front > 5.0:
            front_desc = "front clear >5m"
        else:
            front_desc = f"front blocked at {min_front:.1f}m"
            
        if min_left > 5.0:
            left_desc = "left clear >5m"
        else:
            left_desc = f"left {min_left:.1f}m"
            
        if min_right > 5.0:
            right_desc = "right clear >5m"
        else:
            right_desc = f"right {min_right:.1f}m"
        
        self.obstacle_description = f"{front_desc}, {left_desc}, {right_desc}"
        
        # Publish obstacle info
        obstacle_msg = String()
        obstacle_msg.data = self.obstacle_description
        self.obstacle_info_pub.publish(obstacle_msg)
        
        if self.debug_mode:
            self.get_logger().info(f"🔍 LiDAR: {self.obstacle_description}")
            self.get_logger().info(f"   Obstacle detected: {self.obstacle_detected}, Front clear: {self.front_clear}")
            self.get_logger().info(f"   Min distances - Front: {min_front:.2f}m, Left: {min_left:.2f}m, Right: {min_right:.2f}m")
    
    def request_turn_direction(self, obstacle_desc: str) -> Optional[int]:
        """Request turn direction from API with cooldown management"""
        current_time = time.time()
        time_since_last_call = current_time - self.last_api_call_time
        
        if time_since_last_call < self.api_cooldown_seconds:
            remaining_time = self.api_cooldown_seconds - time_since_last_call
            if not self.cooldown_stop_logged:
                self.get_logger().info(f"⏳ API cooldown: {remaining_time:.1f}s remaining - robot stopped")
                self.cooldown_stop_logged = True
            self.waiting_for_cooldown = True
            return None  # Must wait
        
        # Make API call
        self.get_logger().info(f"🌐 Requesting turn direction from API...")
        self.get_logger().info(f"   Situation: {obstacle_desc}")
        
        # Terminal output for user visibility
        print(f"\n🛑 ROBOT STOPPED - Obstacle detected!")
        print(f"📍 Situation: {obstacle_desc}")
        
        self.last_api_call_time = current_time
        self.waiting_for_cooldown = False
        self.cooldown_stop_logged = False
        
        if self.enable_api:
            direction = self.api_client.get_turn_direction(obstacle_desc)
        else:
            # Random mode for testing
            import random
            direction = random.choice([1, -1])
            self.get_logger().info("📲 Using random direction (API disabled)")
            print("📲 Using random direction (API disabled)")
        
        # Terminal output for API response
        if direction is None:
            self.get_logger().error("❌ API call failed - stopping robot")
            print("❌ API call failed - robot will stay stopped for safety")
            self.navigation_state = "stopped"
        elif direction == 0:
            self.get_logger().warning("⚠️ Unclear API response - stopping robot")
            print("⚠️ API gave unclear response - robot will stay stopped for safety")
            self.navigation_state = "stopped"
        else:
            direction_str = "left" if direction == 1 else "right"
            self.get_logger().info(f"✅ API decision: turn {direction_str}")
            print(f"✅ Bike4Mind API decision: Turn {direction_str.upper()}!")
            print(f"🔄 Robot starting {direction_str} turn...")
        
        return direction
    
    def control_loop(self):
        """Main control loop for navigation"""
        if not self.enabled:
            if self.debug_mode:
                self.get_logger().info("🛑 Control loop: Navigation disabled")
            return
            
        if self.laser_data is None:
            if self.debug_mode:
                self.get_logger().info("⏸️ Control loop: No laser data available")
            return
        
        cmd = Twist()
        
        if self.debug_mode:
            self.get_logger().info(f"🎛️ Control loop: State={self.navigation_state}, Front_clear={self.front_clear}, Obstacle={self.obstacle_detected}, Cooldown={self.waiting_for_cooldown}")
        
        # Handle different navigation states
        if self.navigation_state == "waiting":
            # Initial state - start moving if path clear
            if self.front_clear:
                old_state = self.navigation_state
                self.navigation_state = "moving"
                self.get_logger().info(f"🚀 State transition: {old_state} -> {self.navigation_state} (path clear)")
            else:
                # Request initial turn direction
                if not self.api_decision_pending and not self.waiting_for_cooldown:
                    self.get_logger().info(f"🌐 Requesting API decision in {self.navigation_state} state")
                    direction = self.request_turn_direction(self.obstacle_description)
                    if direction is not None and direction != 0:
                        self.turn_direction = direction
                        old_state = self.navigation_state
                        self.navigation_state = "turning"
                        self.turn_start_time = time.time()
                        self.clear_readings_count = 0
                        self.api_decision_pending = False
                        direction_str = "left" if direction == 1 else "right"
                        self.get_logger().info(f"🔄 State transition: {old_state} -> {self.navigation_state} (turn {direction_str})")
                    else:
                        self.api_decision_pending = True
                        if self.debug_mode:
                            self.get_logger().info("⏳ API decision pending or failed")
        
        elif self.navigation_state == "moving":
            # Moving forward
            if self.obstacle_detected:
                # Stop and request turn direction
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                old_state = self.navigation_state
                self.navigation_state = "obstacle_detected"
                self.get_logger().info(f"🛑 State transition: {old_state} -> {self.navigation_state} (obstacle detected)")
                if self.debug_mode:
                    self.get_logger().info(f"   Obstacle details: {self.obstacle_description}")
            else:
                # Continue moving forward
                cmd.linear.x = self.linear_speed
                cmd.angular.z = 0.0
                # Show movement status periodically
                import time
                current_time = time.time()
                if not hasattr(self, '_last_movement_print') or (current_time - self._last_movement_print) > 3.0:
                    print(f"➡️ Robot moving forward (speed: {cmd.linear.x:.2f} m/s)")
                    self._last_movement_print = current_time
                if self.debug_mode:
                    self.get_logger().info(f"➡️ Moving forward: linear={cmd.linear.x:.3f} m/s")
        
        elif self.navigation_state == "obstacle_detected":
            # Obstacle detected - request API decision
            if not self.api_decision_pending and not self.waiting_for_cooldown:
                self.get_logger().info(f"🌐 Requesting API decision in {self.navigation_state} state")
                direction = self.request_turn_direction(self.obstacle_description)
                if direction is not None and direction != 0:
                    self.turn_direction = direction
                    old_state = self.navigation_state
                    self.navigation_state = "turning"
                    self.turn_start_time = time.time()
                    self.clear_readings_count = 0
                    self.api_decision_pending = False
                    direction_str = "left" if direction == 1 else "right"
                    self.get_logger().info(f"🔄 State transition: {old_state} -> {self.navigation_state} (turn {direction_str})")
                elif direction == 0 or direction is None:
                    old_state = self.navigation_state
                    self.navigation_state = "stopped"
                    self.api_decision_pending = False
                    reason = "unclear response" if direction == 0 else "API failure"
                    self.get_logger().info(f"🛑 State transition: {old_state} -> {self.navigation_state} ({reason})")
                else:
                    self.api_decision_pending = True
                    if self.debug_mode:
                        self.get_logger().info("⏳ API decision pending")
            else:
                # Waiting for API or cooldown
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                if self.debug_mode:
                    cooldown_status = "in cooldown" if self.waiting_for_cooldown else "pending decision"
                    self.get_logger().info(f"⏸️ Stopped waiting ({cooldown_status})")
        
        elif self.navigation_state == "turning":
            # Turning in place
            cmd.linear.x = 0.0
            cmd.angular.z = self.angular_speed * self.turn_direction
            
            if self.debug_mode:
                direction_str = "left" if self.turn_direction == 1 else "right"
                self.get_logger().info(f"🔄 Turning {direction_str}: angular={cmd.angular.z:.3f} rad/s")
            
            # Check turn completion
            current_time = time.time()
            turn_duration = current_time - self.turn_start_time
            
            # Count clear readings
            if self.front_clear:
                self.clear_readings_count += 1
            else:
                self.clear_readings_count = 0
            
            # Check if turn is complete
            if turn_duration >= self.min_turn_time and self.clear_readings_count >= self.required_clear_readings:
                old_state = self.navigation_state
                self.navigation_state = "moving"
                direction_str = "left" if self.turn_direction == 1 else "right"
                self.get_logger().info(f"✅ State transition: {old_state} -> {self.navigation_state} (turn {direction_str} complete after {turn_duration:.1f}s)")
            elif self.debug_mode:
                min_turn_ok = turn_duration >= self.min_turn_time
                clear_readings_ok = self.clear_readings_count >= self.required_clear_readings
                self.get_logger().info(f"   Turn progress: duration={turn_duration:.1f}s (need {self.min_turn_time}s) ✓={min_turn_ok}, clear_readings={self.clear_readings_count}/{self.required_clear_readings} ✓={clear_readings_ok}")
        
        elif self.navigation_state == "stopped":
            # Stopped due to API failure or unclear response
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            if self.debug_mode:
                self.get_logger().info("🛑 Stopped - no movement commands")
        
        # Publish velocity command
        if self.debug_mode:
            self.get_logger().info(f"📤 Publishing cmd_vel: linear={cmd.linear.x:.3f}, angular={cmd.angular.z:.3f}")
        self.cmd_pub.publish(cmd)
    
    def publish_status(self):
        """Publish navigation status"""
        status_msg = String()
        
        if self.waiting_for_cooldown:
            current_time = time.time()
            time_since_last_call = current_time - self.last_api_call_time
            remaining_time = self.api_cooldown_seconds - time_since_last_call
            status_msg.data = f"State: {self.navigation_state} | Cooldown: {remaining_time:.1f}s | {self.obstacle_description}"
        else:
            status_msg.data = f"State: {self.navigation_state} | API: {'Enabled' if self.enable_api else 'Disabled'} | {self.obstacle_description}"
        
        self.status_pub.publish(status_msg)
        
        if self.debug_mode:
            self.get_logger().info(f"📊 {status_msg.data}")
    
    def publish_cooldown(self):
        """Publish API cooldown status"""
        cooldown_msg = Float32()
        
        if self.waiting_for_cooldown:
            current_time = time.time()
            time_since_last_call = current_time - self.last_api_call_time
            remaining_time = max(0, self.api_cooldown_seconds - time_since_last_call)
            cooldown_msg.data = remaining_time
        else:
            cooldown_msg.data = 0.0
        
        self.cooldown_pub.publish(cooldown_msg)
    
    def command_callback(self, msg):
        """Handle manual commands"""
        command = msg.data.lower()
        
        if command == "stop":
            self.enabled = False
            self.navigation_state = "stopped"
            self.get_logger().info("🛑 Navigation stopped by command")
            
            # Send stop command
            cmd = Twist()
            self.cmd_pub.publish(cmd)
            
        elif command == "start":
            self.enabled = True
            self.navigation_state = "waiting"
            self.get_logger().info("▶️ Navigation started by command")
            
        elif command == "reset":
            self.navigation_state = "waiting"
            self.api_decision_pending = False
            self.waiting_for_cooldown = False
            self.get_logger().info("🔄 Navigation reset")
    
    def enable_callback(self, request, response):
        """Service callback to enable/disable navigation"""
        self.enabled = request.data
        
        if not self.enabled:
            self.navigation_state = "stopped"
            # Send stop command
            cmd = Twist()
            self.cmd_pub.publish(cmd)
        else:
            self.navigation_state = "waiting"
        
        response.success = True
        response.message = f"Navigation {'enabled' if self.enabled else 'disabled'}"
        self.get_logger().info(f"🔧 {response.message}")
        return response
    
    def api_mode_callback(self, request, response):
        """Service callback to enable/disable API mode"""
        self.enable_api = request.data
        
        response.success = True
        response.message = f"API mode {'enabled' if self.enable_api else 'disabled (using random)'}"
        self.get_logger().info(f"🔧 {response.message}")
        return response


def main():
    print("🤖 B4M LiDAR Navigator")
    print("======================")
    print()
    print("Intelligent navigation using LiDAR and B4M API")
    print("Robot will request turn directions when obstacles detected")
    print("API cooldown: 20 seconds between requests")
    print()
    
    # Check if we're in simulation mode based on ROS parameters
    import os
    use_sim_time = os.environ.get('ROS_USE_SIM_TIME', '').lower() == 'true'
    
    if use_sim_time:
        print("🎮 SIMULATION MODE: Using Gazebo Classic simulation")
        print("   Virtual robot will navigate in simulated environment")
    else:
        print("🤖 REAL ROBOT MODE: Operating with physical robot")
        print("   ⚠️  ENSURE PHYSICAL ROBOT IS READY AND AREA IS SAFE")
        
    print()
    print("Press Ctrl+C to stop")
    print()
    
    rclpy.init()
    
    try:
        navigator = B4MLidarNavigator()
        print("✅ Navigator initialized")
        print("🚀 Starting navigation loop...")
        print()
        
        rclpy.spin(navigator)
        
    except KeyboardInterrupt:
        print("\n🛑 Navigation stopped by user")
        
        # Send stop command before shutting down
        if 'navigator' in locals():
            stop_cmd = Twist()
            navigator.cmd_pub.publish(stop_cmd)
            time.sleep(0.1)
            
    except Exception as e:
        print(f"❌ Navigation failed: {e}")
        return 1
        
    finally:
        if 'navigator' in locals():
            # Send final stop command
            stop_cmd = Twist()
            navigator.cmd_pub.publish(stop_cmd)
        
        try:
            rclpy.shutdown()
        except:
            pass
    
    print("🎯 B4M LiDAR navigation completed")
    return 0


if __name__ == '__main__':
    exit(main())