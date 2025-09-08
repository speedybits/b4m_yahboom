#!/usr/bin/env python3
"""
Ollama Navigation Controller for B4M Yahboom Robot
Combines 360° spatial awareness with Navigation 2 goal selection
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String, Header
from tf2_ros import TransformListener, Buffer
from tf2_geometry_msgs import do_transform_pose

import math
import json
import yaml
import time
import requests
import numpy as np
import os
import sys
import argparse
from datetime import datetime
from typing import Dict, List, Tuple, Optional, Any
from dataclasses import dataclass
from enum import Enum


def quaternion_from_euler(roll, pitch, yaw):
    """
    Convert euler angles to quaternion (roll, pitch, yaw in radians)
    Returns [x, y, z, w]
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    w = cr * cp * cy + sr * sp * sy

    return [x, y, z, w]


def euler_from_quaternion(quaternion):
    """
    Convert quaternion to euler angles (roll, pitch, yaw)
    quaternion = [x, y, z, w]
    Returns (roll, pitch, yaw) in radians
    """
    x, y, z, w = quaternion
    
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def get_yaw_from_quaternion(orientation):
    """
    Extract yaw angle from geometry_msgs quaternion
    """
    quaternion = [
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w
    ]
    _, _, yaw = euler_from_quaternion(quaternion)
    return yaw


class NavigationState(Enum):
    """States for the navigation system"""
    IDLE = "idle"
    ANALYZING = "analyzing"
    REQUESTING_GOAL = "requesting_goal"
    NAVIGATING = "navigating"
    GOAL_REACHED = "goal_reached"
    FAILED = "failed"
    STOPPED = "stopped"


@dataclass
class SpatialContext:
    """Stores 360° spatial context from LIDAR"""
    sectors: Dict[int, float]  # Angle -> minimum distance
    front_clear: bool
    left_clear: bool
    right_clear: bool
    behind_clear: bool
    closest_obstacle_distance: float
    closest_obstacle_angle: float
    clearest_direction: float
    exploration_percentage: float
    
    
@dataclass
class NavigationGoal:
    """Represents a navigation goal from Ollama"""
    relative_distance: float
    relative_bearing: float  # In radians
    final_orientation: float  # In radians
    reasoning: str
    exploration_value: float
    navigation_value: float
    timestamp: datetime


class OllamaNavController(Node):
    """Main controller for Ollama-guided Navigation 2 integration"""
    
    def __init__(self):
        super().__init__('ollama_nav_controller')
        
        # Load configuration
        self.load_config()
        
        # Initialize state
        self.state = NavigationState.IDLE
        self.consecutive_failures = 0
        self.goals_completed = 0
        self.total_distance_traveled = 0.0
        self.session_start_time = time.time()
        self.current_pose = None
        self.map_data = None
        self.laser_data = None
        self.last_goal_time = None
        self.exploration_start_percentage = 0.0
        
        # Setup ROS2 components
        self.setup_ros2_components()
        
        # Print startup message
        self.print_startup_message()
        
        # Start main control loop (wait a bit for TF to stabilize)
        self.create_timer(5.0, self.check_system_ready)  # Check readiness first
        self.main_timer = None
        
    def check_system_ready(self):
        """Check if all required systems are ready before starting main loop"""
        try:
            # Check if essential topics are available
            topics = self.get_topic_names_and_types()
            topic_names = [name for name, _ in topics]
            
            required_topics = ['/scan', '/odom', '/map']
            missing_topics = [topic for topic in required_topics if topic not in topic_names]
            
            if missing_topics:
                self.get_logger().warn(f"Still waiting for topics: {missing_topics}")
                return
                
            # Check if TF transforms are available
            try:
                # Test a basic transform lookup
                transform = self.tf_buffer.lookup_transform(
                    'base_link', 'base_link', 
                    Time()
                )
                self.tf_ready = True
                self.get_logger().info("✅ System ready - TF and topics available")
                
                # Start main control loop
                if self.main_timer is None:
                    self.main_timer = self.create_timer(1.0, self.control_loop)
                    
            except Exception as e:
                self.get_logger().warn(f"TF not ready yet: {e}")
                
        except Exception as e:
            self.get_logger().warn(f"System readiness check failed: {e}")
        
    def load_config(self):
        """Load configuration from YAML file"""
        config_path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            'config',
            'ollama_nav_config.yaml'
        )
        
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)
            
        self.get_logger().info(f"Loaded configuration from {config_path}")
        
    def setup_ros2_components(self):
        """Initialize ROS2 publishers, subscribers, and action clients"""
        
        # QoS profiles
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribers
        self.laser_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            qos_best_effort
        )
        
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            qos_reliable
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_best_effort
        )
        
        self.amcl_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_pose_callback,
            qos_best_effort
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            qos_reliable
        )
        
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/ollama_nav_markers',
            qos_reliable
        )
        
        self.status_pub = self.create_publisher(
            String,
            '/ollama_nav_status',
            qos_reliable
        )
        
        # Action client for Navigation 2
        self.nav2_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )
        
        # TF2 for coordinate transformations  
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Wait for TF to be ready
        self.tf_ready = False
        
        self.get_logger().info("ROS2 components initialized")
        
    def print_startup_message(self):
        """Print the startup message to console"""
        print("\n" + "="*63)
        print("🧭🦙 OLLAMA NAVIGATION MODE")
        print("="*63)
        print("Intelligent LLM-guided navigation with Navigation 2 stack")
        print(f"Model: {self.config['ollama_nav']['model']} | API: {self.config['ollama_nav']['host']}:{self.config['ollama_nav']['port']}")
        print("="*63)
        print("")
        
        timestamp = datetime.now().strftime("[%H:%M:%S.%f")[:-3] + "]"
        print(f"{timestamp} 🚀 Starting Navigation 2 with Cartographer SLAM...")
        time.sleep(0.5)
        print(f"{timestamp} 📍 Robot localization initialized")
        print(f"{timestamp} 🗺️ SLAM mapping active - building environment map")
        print(f"{timestamp} 🦙 Ollama Navigation Controller ready")
        print(f"{timestamp} 🎯 Awaiting initial spatial analysis for first goal selection...")
        print("")
        
    def laser_callback(self, msg: LaserScan):
        """Process laser scan data"""
        self.laser_data = msg
        
    def map_callback(self, msg: OccupancyGrid):
        """Process map data for exploration tracking"""
        self.map_data = msg
        self.calculate_exploration_percentage()
        
    def odom_callback(self, msg: Odometry):
        """Track robot odometry for distance calculations"""
        if self.current_pose is None:
            self.current_pose = msg.pose.pose
            
    def amcl_pose_callback(self, msg: PoseWithCovarianceStamped):
        """Update robot pose from AMCL localization"""
        self.current_pose = msg.pose.pose
        
    def calculate_exploration_percentage(self):
        """Calculate percentage of map explored"""
        if self.map_data is None:
            return 0.0
            
        total_cells = len(self.map_data.data)
        known_cells = sum(1 for cell in self.map_data.data if cell >= 0)
        
        if total_cells > 0:
            return (known_cells / total_cells) * 100.0
        return 0.0
        
    def analyze_spatial_context(self) -> SpatialContext:
        """Analyze 360° LIDAR data to create spatial context"""
        if self.laser_data is None:
            return None
            
        ranges = np.array(self.laser_data.ranges)
        
        # Filter invalid readings
        ranges[ranges == 0.0] = self.laser_data.range_max
        ranges[ranges <= self.laser_data.range_min] = self.laser_data.range_max
        ranges[np.isinf(ranges)] = self.laser_data.range_max
        ranges[np.isnan(ranges)] = self.laser_data.range_max
        
        # Analyze sectors (24 sectors of 15° each)
        num_sectors = 24
        sector_size = len(ranges) // num_sectors
        sectors = {}
        
        for i in range(num_sectors):
            start_idx = i * sector_size
            end_idx = start_idx + sector_size
            sector_ranges = ranges[start_idx:end_idx]
            
            if len(sector_ranges) > 0:
                min_dist = np.min(sector_ranges)
                angle = i * 15  # Degrees
                sectors[angle] = min_dist
                
        # Determine clear directions
        obstacle_threshold = self.config['safety']['obstacle_clearance']
        
        # Check cardinal directions (using wider sectors for stability)
        front_ranges = ranges[0:30] if len(ranges) > 30 else ranges
        front_clear = np.min(front_ranges) > obstacle_threshold
        
        left_idx = len(ranges) // 4
        left_ranges = ranges[left_idx-15:left_idx+15] if len(ranges) > left_idx+15 else ranges
        left_clear = np.min(left_ranges) > obstacle_threshold
        
        right_idx = 3 * len(ranges) // 4
        right_ranges = ranges[right_idx-15:right_idx+15] if len(ranges) > right_idx+15 else ranges
        right_clear = np.min(right_ranges) > obstacle_threshold
        
        behind_idx = len(ranges) // 2
        behind_ranges = ranges[behind_idx-15:behind_idx+15] if len(ranges) > behind_idx+15 else ranges
        behind_clear = np.min(behind_ranges) > obstacle_threshold
        
        # Find closest obstacle
        min_distance = np.min(ranges)
        min_idx = np.argmin(ranges)
        closest_angle = (min_idx / len(ranges)) * 360.0
        
        # Find clearest direction
        max_distance = np.max(ranges)
        max_idx = np.argmax(ranges)
        clearest_angle = (max_idx / len(ranges)) * 360.0
        
        return SpatialContext(
            sectors=sectors,
            front_clear=front_clear,
            left_clear=left_clear,
            right_clear=right_clear,
            behind_clear=behind_clear,
            closest_obstacle_distance=float(min_distance),
            closest_obstacle_angle=closest_angle,
            clearest_direction=clearest_angle,
            exploration_percentage=self.calculate_exploration_percentage()
        )
        
    def generate_spatial_description(self, context: SpatialContext) -> str:
        """Generate natural language description of spatial context"""
        description = []
        
        description.append("ENVIRONMENTAL ANALYSIS - Navigation Context")
        description.append("="*43)
        description.append("")
        description.append("IMMEDIATE SURROUNDINGS (LIDAR):")
        
        # Describe cardinal directions
        def describe_direction(clear: bool, distance: float, name: str) -> str:
            if clear:
                return f"• {name}: CLEAR - Open space extending >{distance:.1f}m"
            else:
                return f"• {name}: BLOCKED - Obstacle at {distance:.1f}m"
                
        front_dist = context.sectors.get(0, 999)
        description.append(describe_direction(context.front_clear, front_dist, "FRONT (0°)"))
        
        right_dist = context.sectors.get(90, 999)
        description.append(describe_direction(context.right_clear, right_dist, "RIGHT (90°)"))
        
        behind_dist = context.sectors.get(180, 999)
        description.append(describe_direction(context.behind_clear, behind_dist, "BEHIND (180°)"))
        
        left_dist = context.sectors.get(270, 999)
        description.append(describe_direction(context.left_clear, left_dist, "LEFT (270°)"))
        
        description.append("")
        description.append("MAP CONTEXT:")
        
        if self.current_pose:
            # Get position and orientation
            x = self.current_pose.position.x
            y = self.current_pose.position.y
            
            yaw = get_yaw_from_quaternion(self.current_pose.orientation)
            yaw_degrees = math.degrees(yaw)
            
            description.append(f"• Current position: ({x:.1f}, {y:.1f}) facing {yaw_degrees:.0f}°")
            
        description.append(f"• Explored area: {context.exploration_percentage:.0f}% of visible map space")
        description.append(f"• Closest obstacle: {context.closest_obstacle_distance:.1f}m at {context.closest_obstacle_angle:.0f}°")
        description.append(f"• Clearest direction: {context.clearest_direction:.0f}° with maximum clearance")
        
        description.append("")
        description.append("NAVIGATION OPPORTUNITIES:")
        
        # Find exploration opportunities
        clear_sectors = [angle for angle, dist in context.sectors.items() if dist > 2.0]
        if clear_sectors:
            description.append(f"• Clear paths available at: {', '.join([f'{a}°' for a in clear_sectors[:5]])}")
            
        return "\n".join(description)
        
    def create_ollama_prompt(self, spatial_description: str) -> str:
        """Create the prompt for Ollama"""
        prompt = f"""You are a navigation AI for an autonomous robot. Based on the environmental analysis below, 
select the optimal navigation target using RELATIVE positioning from the robot's current location.

CURRENT SITUATION:
{spatial_description}

OBJECTIVES (Balance Both):
1. EXPLORATION: Discover unmapped areas to build complete environment knowledge
2. NAVIGATION: Position strategically for efficient future movements
3. Maintain safe distance from obstacles (>0.5m clearance)
4. Maximize sensor coverage of surroundings

RELATIVE POSITIONING:
- Specify movement as distance and bearing FROM current position
- Distance: How far to move (1.0 to 5.0 meters)
- Bearing: Direction relative to current heading (-180 to +180 degrees)
  • 0° = straight ahead
  • 90° = right turn
  • -90° = left turn
  • 180° = behind

RESPONSE FORMAT:
Respond with a JSON object containing:
{{
  "relative_distance": [METERS_TO_MOVE],
  "relative_bearing": [DEGREES_FROM_CURRENT_HEADING],
  "final_orientation": [DESIRED_HEADING_AT_DESTINATION], 
  "reasoning": "Brief explanation balancing exploration and navigation needs",
  "exploration_value": [0.0-1.0],
  "navigation_value": [0.0-1.0]
}}

Example response:
{{"relative_distance": 2.3, "relative_bearing": 45, "final_orientation": 90, "reasoning": "Moving northeast to explore unmapped corridor while maintaining strategic position", "exploration_value": 0.8, "navigation_value": 0.7}}"""
        
        return prompt
        
    def request_ollama_goal(self, spatial_context: SpatialContext) -> Optional[NavigationGoal]:
        """Request navigation goal from Ollama"""
        # Generate spatial description
        spatial_description = self.generate_spatial_description(spatial_context)
        
        # Display analysis
        print("\n" + "="*63)
        print("🤖 OLLAMA NAVIGATION - GOAL SELECTION")
        print("="*63)
        print("")
        print("📏 Environmental Analysis Complete:")
        print("-"*63)
        print(f"• LIDAR Coverage: 360° scan with {len(self.laser_data.ranges) if self.laser_data else 0} data points")
        print(f"• Immediate Clearance: Front {spatial_context.front_clear}, Left {spatial_context.left_clear}, Right {spatial_context.right_clear}, Rear {spatial_context.behind_clear}")
        print(f"• Map Status: {spatial_context.exploration_percentage:.0f}% explored")
        
        if self.current_pose:
            x = self.current_pose.position.x
            y = self.current_pose.position.y
            print(f"• Current Position: ({x:.2f}, {y:.2f})")
            
        print("")
        print("🦙 Consulting Ollama for optimal navigation goal...")
        print("   Generating spatial context with map integration...")
        
        # Create prompt
        prompt = self.create_ollama_prompt(spatial_description)
        
        # Prepare API request
        url = f"http://{self.config['ollama_nav']['host']}:{self.config['ollama_nav']['port']}/api/generate"
        
        payload = {
            "model": self.config['ollama_nav']['model'],
            "prompt": prompt,
            "format": "json",
            "stream": False,
            "options": {
                "temperature": self.config['generation']['temperature'],
                "top_p": self.config['generation']['top_p'],
                "max_new_tokens": self.config['generation']['max_tokens']
            }
        }
        
        try:
            # Make API request
            start_time = time.time()
            response = requests.post(url, json=payload, timeout=self.config['ollama_nav']['timeout'])
            response_time = time.time() - start_time
            
            if response.status_code == 200:
                result = response.json()
                response_text = result.get('response', '{}')
                
                # Parse JSON response
                goal_data = json.loads(response_text)
                
                # Validate and create goal
                if self.validate_goal_response(goal_data):
                    goal = NavigationGoal(
                        relative_distance=float(goal_data['relative_distance']),
                        relative_bearing=math.radians(float(goal_data['relative_bearing'])),
                        final_orientation=math.radians(float(goal_data.get('final_orientation', 0))),
                        reasoning=goal_data.get('reasoning', 'No reasoning provided'),
                        exploration_value=float(goal_data.get('exploration_value', 0.5)),
                        navigation_value=float(goal_data.get('navigation_value', 0.5)),
                        timestamp=datetime.now()
                    )
                    
                    # Display result
                    print("")
                    print(f"✅ OLLAMA GOAL SELECTED: (received in {response_time:.1f}s)")
                    print("-"*63)
                    print(f"   Relative Distance: {goal.relative_distance:.1f}m")
                    print(f"   Relative Bearing: {math.degrees(goal.relative_bearing):.0f}° ({'right' if goal.relative_bearing > 0 else 'left'} from current heading)")
                    print(f"   Final Orientation: {math.degrees(goal.final_orientation):.0f}°")
                    print(f"   Reasoning: {goal.reasoning}")
                    print(f"   Exploration Value: {goal.exploration_value:.2f}")
                    print(f"   Navigation Value: {goal.navigation_value:.2f}")
                    print("-"*63)
                    print("")
                    
                    return goal
                    
        except requests.Timeout:
            self.get_logger().error("Ollama request timed out")
            print("\n⏰ Ollama request timed out")
            
        except Exception as e:
            self.get_logger().error(f"Ollama request failed: {e}")
            print(f"\n❌ Ollama request failed: {e}")
            
        return None
        
    def validate_goal_response(self, response: Dict) -> bool:
        """Validate Ollama response format"""
        required_fields = ['relative_distance', 'relative_bearing']
        
        for field in required_fields:
            if field not in response:
                return False
                
        # Validate ranges
        try:
            distance = float(response['relative_distance'])
            if not (self.config['navigation']['min_goal_distance'] <= distance <= 
                    self.config['navigation']['max_goal_distance']):
                return False
                
            bearing = float(response['relative_bearing'])
            if not (-180 <= bearing <= 180):
                return False
                
            return True
            
        except (ValueError, TypeError):
            return False
            
    def convert_relative_to_absolute(self, goal: NavigationGoal) -> PoseStamped:
        """Convert relative goal to absolute map coordinates"""
        if self.current_pose is None:
            self.get_logger().warn("No current pose available for goal conversion")
            return None
        try:
            # Get current position and orientation
            current_x = self.current_pose.position.x
            current_y = self.current_pose.position.y
            
            current_yaw = get_yaw_from_quaternion(self.current_pose.orientation)
            
            # Calculate absolute bearing
            absolute_bearing = current_yaw + goal.relative_bearing
            
            # Calculate target position
            target_x = current_x + goal.relative_distance * math.cos(absolute_bearing)
            target_y = current_y + goal.relative_distance * math.sin(absolute_bearing)
            
            # Create pose message
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()
            
            pose.pose.position.x = target_x
            pose.pose.position.y = target_y
            pose.pose.position.z = 0.0
            
            # Set orientation
            target_quaternion = quaternion_from_euler(0, 0, goal.final_orientation)
            pose.pose.orientation.x = target_quaternion[0]
            pose.pose.orientation.y = target_quaternion[1]
            pose.pose.orientation.z = target_quaternion[2]
            pose.pose.orientation.w = target_quaternion[3]
            
            return pose
            
        except Exception as e:
            self.get_logger().error(f"Error converting relative goal to absolute: {e}")
            return None
        
    def send_navigation_goal(self, pose: PoseStamped) -> bool:
        """Send goal to Navigation 2"""
        if not self.nav2_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Navigation 2 action server not available")
            return False
            
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        goal_msg.behavior_tree = ""  # Use default behavior tree
        
        timestamp = datetime.now().strftime("[%H:%M:%S.%f")[:-3] + "]"
        print(f"{timestamp} 🎯 Publishing navigation goal to Nav2...")
        print(f"{timestamp} 📊 RViz: Goal marker and planned path now visible")
        print(f"{timestamp} ➡️ Robot beginning navigation to selected pose...")
        print("")
        
        # Send goal
        self.nav2_future = self.nav2_client.send_goal_async(goal_msg)
        self.nav2_future.add_done_callback(self.goal_response_callback)
        
        return True
        
    def goal_response_callback(self, future):
        """Handle Navigation 2 goal response"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error("Navigation goal rejected by Nav2")
            self.state = NavigationState.FAILED
            self.consecutive_failures += 1
            return
            
        self.get_logger().info("Navigation goal accepted by Nav2")
        self.state = NavigationState.NAVIGATING
        
        # Get result
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.get_result_callback)
        
    def get_result_callback(self, future):
        """Handle Navigation 2 result"""
        result = future.result().result
        
        timestamp = datetime.now().strftime("[%H:%M:%S.%f")[:-3] + "]"
        
        if result:
            print(f"{timestamp} 🎯 Navigation goal REACHED successfully!")
            self.state = NavigationState.GOAL_REACHED
            self.goals_completed += 1
            self.consecutive_failures = 0
            
            # Display results
            print("\n" + "="*63)
            print("🤖 OLLAMA NAVIGATION - GOAL ACHIEVED")
            print("="*63)
            print("")
            
            if self.current_pose:
                x = self.current_pose.position.x
                y = self.current_pose.position.y
                print(f"📍 Final Position: ({x:.2f}, {y:.2f})")
                
            print("📊 Navigation Results:")
            print("-"*63)
            print(f"• Goals Completed: {self.goals_completed}")
            print(f"• SLAM Mapping Progress: {self.calculate_exploration_percentage():.0f}% explored")
            print("")
            print("🔄 Preparing for next goal selection...")
            print("   Analyzing updated environment and map data...")
            print("")
            
        else:
            print(f"{timestamp} 🚨 Nav2: Path planning FAILED (Attempt {self.consecutive_failures + 1}/3)")
            print("   Unable to find valid path to goal")
            self.state = NavigationState.FAILED
            self.consecutive_failures += 1
            
            if self.consecutive_failures >= self.config['safety']['max_consecutive_failures']:
                self.handle_max_failures()
                
    def handle_max_failures(self):
        """Handle maximum consecutive failures"""
        print("\n" + "🛑 CRITICAL ERROR - NAVIGATION SYSTEM HALTED")
        print("="*63)
        print("Multiple navigation failures detected:")
        print(f"• Failed Attempts: {self.consecutive_failures} consecutive failures")
        print("• Last Error: Unable to find valid path to any selected goal")
        print("• System Status: Navigation disabled for safety")
        print("")
        print("MANUAL INTERVENTION REQUIRED:")
        print("• Check for undetected obstacles or sensor issues")
        print("• Verify SLAM map quality and localization accuracy")
        print("• Consider restarting with ./b4m_shutdown.sh --keep-agent")
        print("• Then relaunch without --ollama-nav for manual control")
        print("="*63)
        print("")
        
        timestamp = datetime.now().strftime("[%H:%M:%S.%f")[:-3] + "]"
        print(f"{timestamp} 🔴 System entering SAFE STOP mode")
        
        if self.current_pose:
            x = self.current_pose.position.x
            y = self.current_pose.position.y
            print(f"{timestamp} 📍 Final position locked at: ({x:.1f}, {y:.1f})")
            
        self.state = NavigationState.STOPPED
        
        # Stop robot
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)
        
    def control_loop(self):
        """Main control loop"""
        if self.state == NavigationState.STOPPED:
            return
            
        # Don't start until system is ready
        if not self.tf_ready:
            return
            
        if self.state == NavigationState.IDLE or self.state == NavigationState.GOAL_REACHED:
            # Check if we have necessary data
            if self.laser_data is None or self.current_pose is None:
                return
                
            # Analyze spatial context
            spatial_context = self.analyze_spatial_context()
            if spatial_context is None:
                return
                
            # Request goal from Ollama
            self.state = NavigationState.REQUESTING_GOAL
            goal = self.request_ollama_goal(spatial_context)
            
            if goal:
                # Convert to absolute coordinates
                absolute_pose = self.convert_relative_to_absolute(goal)
                
                if absolute_pose:
                    # Send to Navigation 2
                    if self.send_navigation_goal(absolute_pose):
                        self.last_goal_time = time.time()
                    else:
                        self.state = NavigationState.FAILED
                        self.consecutive_failures += 1
                else:
                    self.state = NavigationState.IDLE
            else:
                self.state = NavigationState.FAILED
                self.consecutive_failures += 1
                
                if self.consecutive_failures >= self.config['safety']['max_consecutive_failures']:
                    self.handle_max_failures()
                    
        elif self.state == NavigationState.NAVIGATING:
            # Check for timeout
            if self.last_goal_time:
                elapsed = time.time() - self.last_goal_time
                if elapsed > self.config['safety']['nav2_timeout']:
                    self.get_logger().warn("Navigation timeout exceeded")
                    self.state = NavigationState.FAILED
                    self.consecutive_failures += 1
                    
                    # Cancel current goal
                    if hasattr(self, 'nav2_future'):
                        self.nav2_future.cancel()
                        
    def publish_visualization_markers(self, goal_pose: PoseStamped, reasoning: str):
        """Publish visualization markers for RViz"""
        marker_array = MarkerArray()
        
        # Goal marker
        goal_marker = Marker()
        goal_marker.header = goal_pose.header
        goal_marker.ns = "ollama_nav_goals"
        goal_marker.id = self.goals_completed
        goal_marker.type = Marker.ARROW
        goal_marker.action = Marker.ADD
        goal_marker.pose = goal_pose.pose
        goal_marker.scale.x = self.config['visualization']['goal_marker_scale']
        goal_marker.scale.y = 0.1
        goal_marker.scale.z = 0.1
        goal_marker.color.r = 0.0
        goal_marker.color.g = 1.0
        goal_marker.color.b = 0.0
        goal_marker.color.a = 1.0
        marker_array.markers.append(goal_marker)
        
        # Reasoning text
        if self.config['visualization']['show_reasoning_text']:
            text_marker = Marker()
            text_marker.header = goal_pose.header
            text_marker.ns = "ollama_nav_reasoning"
            text_marker.id = self.goals_completed
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose = goal_pose.pose
            text_marker.pose.position.z += 0.5
            text_marker.text = reasoning
            text_marker.scale.z = 0.2
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            marker_array.markers.append(text_marker)
            
        self.marker_pub.publish(marker_array)
        
    def shutdown(self):
        """Clean shutdown"""
        print("\n^C")
        timestamp = datetime.now().strftime("[%H:%M:%S.%f")[:-3] + "]"
        print(f"{timestamp} 🛑 Shutdown requested - stopping navigation")
        
        if self.current_pose:
            x = self.current_pose.position.x
            y = self.current_pose.position.y
            yaw = get_yaw_from_quaternion(self.current_pose.orientation)
            yaw_degrees = math.degrees(yaw)
            print(f"{timestamp} 📍 Final robot position: ({x:.1f}, {y:.1f}) facing {yaw_degrees:.0f}°")
            
        print(f"{timestamp} 🗺️ Final map status: {self.calculate_exploration_percentage():.0f}% explored")
        
        # Session statistics
        session_duration = time.time() - self.session_start_time
        minutes = int(session_duration // 60)
        seconds = int(session_duration % 60)
        
        print("\n🧭🦙 OLLAMA NAVIGATION SESSION COMPLETE")
        print("="*63)
        print("Session Statistics:")
        print(f"• Total Goals Selected: {self.goals_completed}")
        print(f"• Total Distance Traveled: {self.total_distance_traveled:.1f}m")
        
        if self.goals_completed > 0:
            success_rate = ((self.goals_completed / (self.goals_completed + self.consecutive_failures)) * 100)
            print(f"• Navigation Success Rate: {success_rate:.1f}%")
            
        print(f"• Area Explored: {self.exploration_start_percentage:.0f}% → {self.calculate_exploration_percentage():.0f}%")
        print(f"• Session Duration: {minutes}m {seconds}s")
        print("="*63)
        
        # Stop robot
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        controller = OllamaNavController()
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        if 'controller' in locals():
            controller.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()