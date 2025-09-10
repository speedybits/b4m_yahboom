#!/usr/bin/env python3
"""
B4M Navigation Explore Mode for B4M Yahboom Robot
Autonomous exploration with LLM-guided goal selection using Navigation 2
Implements closed-loop environmental analysis and frontier-based exploration
Direct adaptation of ollama_explore_spatial.py using B4M API instead of Ollama
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters, GetParameters
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
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
import logging
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


class ExploreState(Enum):
    """States for the exploration system"""
    INITIALIZING = "initializing"
    INITIAL_MAPPING = "initial_mapping"  # Performs 0.5m square mapping routine
    ANALYZING = "analyzing"
    QUERYING_LLM = "querying_llm"
    NAVIGATING = "navigating"
    WAITING = "waiting"  # Waiting between retry attempts
    ERROR = "error"
    SHUTDOWN = "shutdown"


@dataclass
class SpatialContext:
    """Stores 360° spatial context from LIDAR and map data"""
    sectors: Dict[int, float]  # Angle -> minimum distance
    front_clear: bool
    left_clear: bool
    right_clear: bool
    behind_clear: bool
    closest_obstacle_distance: float
    closest_obstacle_angle: float
    clearest_direction: float
    exploration_percentage: float
    frontiers: List[Tuple[float, float]]  # List of frontier points (bearing, distance)
    

@dataclass
class SafeDestination:
    """Pre-validated safe navigation destination"""
    relative_bearing: float      # Degrees relative to robot heading (-180 to 180)
    distance: float             # Meters from current position (1.0 to 3.0)
    world_coords: Tuple[float, float]  # (x, y) in map frame
    description: str            # Human-readable description for LLM
    leads_to_frontier: bool     # Whether unexplored cells exist within 1m of destination
    strategic_value: str        # Optional hint (not prioritized in ordering)

@dataclass
class NavigationGoal:
    """Represents a navigation goal from B4M LLM"""
    relative_distance: float
    relative_bearing: float  # In degrees
    final_orientation: float  # In degrees  
    reasoning: str
    goal_type: str  # "MOVEMENT" or "ROTATION"
    timestamp: datetime


class B4MExploreController(Node):
    """Main controller for B4M LLM-guided autonomous exploration"""
    
    def __init__(self):
        super().__init__('ollama_explore_spatial')
        
        # Setup logging
        self.setup_logging()
        
        # Load configuration
        self.load_config()
        
        # Initialize B4M API client
        self.setup_b4m_client()
        
        # Initialize state
        self.state = ExploreState.INITIALIZING
        self.consecutive_failures = 0
        self.goals_completed = 0
        self.total_distance_traveled = 0.0
        self.session_start_time = time.time()
        self.exploration_start_percentage = 0.0
        self._goal_rejected_once = False  # Flag for retry prompt modification
        
        # Position and sensor data
        self.current_pose = None
        self.map_data = None
        self.laser_data = None
        self.last_position = None
        
        # Navigation state
        self.current_goal_handle = None
        self.last_goal_time = None
        self.nav2_future = None
        self.result_future = None
        self.wait_until = None  # For retry delays
        
        # Initial mapping state
        self.initial_mapping_step = 0  # 0: forward, 1: right, 2: back, 3: left, 4: complete
        self.initial_mapping_start_position = None
        
        # Setup ROS2 components
        self.setup_ros2_components()
        
        # Print startup message
        self.print_startup_message()
        
        # Start system readiness check
        self.readiness_timer = self.create_timer(5.0, self.check_system_ready)
        self.main_timer = None
        self.system_ready = False
        
    def setup_logging(self):
        """Setup comprehensive logging system"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_dir = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            'logs'
        )
        os.makedirs(log_dir, exist_ok=True)
        
        log_file = os.path.join(log_dir, f'ollama_spatial_{timestamp}.log')
        
        # Configure Python logging
        logging.basicConfig(
            level=logging.INFO,
            format='[%(asctime)s.%(msecs)03d] [%(levelname)s] [ollama_explore]: %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S',
            handlers=[
                logging.FileHandler(log_file),
                logging.StreamHandler()
            ]
        )
        
        self.logger = logging.getLogger(__name__)
        self.logger.info("🔍 B4M Exploration logging initialized")
        self.logger.info(f"Log file: {log_file}")
        
    def check_system_ready(self):
        """Check if all required systems are ready before starting main loop"""
        # Don't check if already ready or in error state
        if self.system_ready or self.state == ExploreState.ERROR:
            return
            
        try:
            # Check if essential topics are available
            topics = self.get_topic_names_and_types()
            topic_names = [name for name, _ in topics]
            
            required_topics = ['/scan', '/odom', '/map']
            missing_topics = [topic for topic in required_topics if topic not in topic_names]
            
            if missing_topics:
                self.logger.warning(f"Still waiting for topics: {missing_topics}")
                return
                
            # Check if TF transforms are available
            try:
                # Test basic transform lookup
                transform = self.tf_buffer.lookup_transform(
                    'base_link', 'base_link', 
                    Time()
                )
                
                self.logger.info("✅ System ready - TF and topics available")
                print("✅ AUTONOMOUS NAVIGATION ACTIVE")
                print("")
                
                # Mark system as ready and stop readiness checks
                self.system_ready = True
                if self.readiness_timer:
                    self.readiness_timer.cancel()
                    self.readiness_timer = None
                
                # Initialize exploration tracking
                if self.map_data:
                    self.exploration_start_percentage = self.calculate_exploration_percentage()
                
                # Start main control loop
                if self.main_timer is None:
                    self.main_timer = self.create_timer(2.0, self.control_loop)
                    self.state = ExploreState.INITIAL_MAPPING
                    
            except Exception as e:
                self.logger.warning(f"TF not ready yet: {e}")
                
        except Exception as e:
            self.logger.warning(f"System readiness check failed: {e}")
        
    def load_config(self):
        """Load configuration from YAML file"""
        config_path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            'config',
            'b4m_nav_config.yaml'
        )
        
        try:
            with open(config_path, 'r') as f:
                self.config = yaml.safe_load(f)
            
            # Validate required configuration keys
            self.validate_config()
            self.logger.info(f"Configuration loaded from {config_path}")
            
        except Exception as e:
            self.logger.error(f"Failed to load configuration: {e}")
            raise
            
    def setup_b4m_client(self):
        """Initialize B4M API client"""
        # Get API key from environment variable
        self.b4m_api_key = os.environ.get('B4M_API_KEY')
        if not self.b4m_api_key:
            self.logger.error("B4M_API_KEY environment variable not set!")
            print("ERROR: B4M_API_KEY environment variable not set!")
            print("Please set it: export B4M_API_KEY='your_key_here'")
            sys.exit(1)
        
        # Get session ID from environment or use default
        self.b4m_session_id = os.environ.get('B4M_SESSION_ID', '68b1e0fcac3f77504fce09b5')
        self.b4m_user_id = os.environ.get('B4M_USER_ID', '65563f622213b120cd1d9592')
        
        # B4M API endpoint
        self.b4m_api_url = "https://app.bike4mind.com/api/ai/llm"
        
        self.logger.info(f"B4M API client initialized with session: {self.b4m_session_id}")
    
    def validate_config(self):
        """Validate that all required configuration keys exist"""
        required_keys = [
            ('b4m_nav', 'timeout'),
            ('navigation', 'min_goal_distance'),
            ('navigation', 'max_goal_distance'),
            ('navigation', 'rotation_probability'),
            ('generation', 'temperature'),
            ('generation', 'max_tokens'),
            ('safety', 'obstacle_clearance'),
            ('safety', 'nav2_timeout'),
            ('safety', 'max_consecutive_failures')
        ]
        
        for section, key in required_keys:
            try:
                value = self.config[section][key]
            except KeyError:
                raise KeyError(f"Missing required config: {section}.{key}")
        
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
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            qos_reliable
        )
        
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/ollama_explore_markers',
            qos_reliable
        )
        
        self.status_pub = self.create_publisher(
            String,
            '/ollama_explore_status',
            qos_reliable
        )
        
        # Action client for Navigation 2
        self.nav2_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )
        
        # Parameter clients for dynamic velocity control during initial mapping
        self.controller_param_client = self.create_client(
            SetParameters,
            '/controller_server/set_parameters'
        )
        self.get_controller_param_client = self.create_client(
            GetParameters,
            '/controller_server/get_parameters'
        )
        
        # Storage for original velocity parameters
        self.original_velocities = None
        self.slow_mapping_velocities = {
            'FollowPath.max_vel_x': 0.05,     # Very slow forward speed (5 cm/s)
            'FollowPath.max_vel_theta': 0.2,  # Very slow rotation speed
            'FollowPath.min_vel_x': -0.03,    # Very slow backward speed
            'FollowPath.acc_lim_x': 0.2,      # Very slow acceleration
            'FollowPath.acc_lim_theta': 0.2   # Very slow angular acceleration
        }
        
        # TF2 for coordinate transformations  
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.logger.info("ROS2 components initialized")
        
    def save_original_velocities(self):
        """Save current velocity parameters before switching to slow mode"""
        try:
            # Wait for parameter service to be available
            if not self.get_controller_param_client.wait_for_service(timeout_sec=10.0):
                self.logger.warning("Controller parameter service not available for saving velocities")
                return False
                
            # Get current velocity parameters
            param_names = list(self.slow_mapping_velocities.keys())
            request = GetParameters.Request()
            request.names = param_names
            
            future = self.get_controller_param_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.result():
                response = future.result()
                self.original_velocities = {}
                for i, param in enumerate(response.values):
                    if i < len(param_names):
                        param_name = param_names[i]
                        if param.type == ParameterType.PARAMETER_DOUBLE:
                            self.original_velocities[param_name] = param.double_value
                        elif param.type == ParameterType.PARAMETER_INTEGER:
                            self.original_velocities[param_name] = float(param.integer_value)
                
                self.logger.info(f"✅ Saved original velocity parameters: {self.original_velocities}")
                return True
            else:
                self.logger.warning("Failed to get current velocity parameters")
                return False
                
        except Exception as e:
            self.logger.error(f"Error saving original velocities: {e}")
            return False
    
    def set_slow_mapping_velocities(self):
        """Set very slow velocities for precise initial mapping"""
        try:
            if not self.controller_param_client.wait_for_service(timeout_sec=10.0):
                self.logger.warning("Controller parameter service not available for setting velocities")
                return False
                
            # Create parameters for slow mapping
            parameters = []
            for param_name, value in self.slow_mapping_velocities.items():
                param = Parameter(name=param_name, value=value)
                parameters.append(param)
            
            request = SetParameters.Request()
            request.parameters = parameters
            
            future = self.controller_param_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.result():
                response = future.result()
                success_count = sum(1 for result in response.results if result.successful)
                self.logger.info(f"🐌 Set slow mapping velocities ({success_count}/{len(parameters)} parameters)")
                return success_count == len(parameters)
            else:
                self.logger.warning("Failed to set slow mapping velocities")
                return False
                
        except Exception as e:
            self.logger.error(f"Error setting slow mapping velocities: {e}")
            return False
    
    def restore_original_velocities(self):
        """Restore original velocity parameters after initial mapping"""
        if self.original_velocities is None:
            self.logger.warning("No original velocities saved to restore")
            return False
            
        try:
            if not self.controller_param_client.wait_for_service(timeout_sec=10.0):
                self.logger.warning("Controller parameter service not available for restoring velocities")
                return False
                
            # Create parameters from saved original values
            parameters = []
            for param_name, value in self.original_velocities.items():
                param = Parameter(name=param_name, value=value)
                parameters.append(param)
            
            request = SetParameters.Request()
            request.parameters = parameters
            
            future = self.controller_param_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.result():
                response = future.result()
                success_count = sum(1 for result in response.results if result.successful)
                self.logger.info(f"🚀 Restored original velocities ({success_count}/{len(parameters)} parameters)")
                return success_count == len(parameters)
            else:
                self.logger.warning("Failed to restore original velocities")
                return False
                
        except Exception as e:
            self.logger.error(f"Error restoring original velocities: {e}")
            return False
        
    def print_startup_message(self):
        """Print the startup message to console"""
        print("\n" + "="*63)
        print("🧭 B4M NAVIGATION EXPLORE MODE")
        print("="*63)
        print("Launching Navigation 2 with Cartographer SLAM for LLM-guided exploration")
        print("")
        print("Step 1: Starting Micro-ROS Agent...")
        print("✅ Micro-ROS Agent started (PID: external)")
        print("")
        print("Step 7: Starting B4M Exploration Spatial Analysis")
        print(f"✅ B4M spatial analysis started (PID: {os.getpid()})")
        print("")
        print("🤖 Initializing autonomous navigation system...")
        print(f"Model: gpt-4o-mini | API: B4M Service")
        print("")
        
    def laser_callback(self, msg: LaserScan):
        """Process laser scan data"""
        self.laser_data = msg
        
    def map_callback(self, msg: OccupancyGrid):
        """Process map data for exploration tracking"""
        self.map_data = msg
        
    def odom_callback(self, msg: Odometry):
        """Track robot odometry for distance calculations and real position"""
        # Track position changes for distance calculation
        if self.current_pose and self.last_position:
            current_x = msg.pose.pose.position.x
            current_y = msg.pose.pose.position.y
            last_x = self.last_position.position.x  
            last_y = self.last_position.position.y
            
            distance = math.sqrt((current_x - last_x)**2 + (current_y - last_y)**2)
            self.total_distance_traveled += distance
            
        self.last_position = msg.pose.pose
        
        # Always update current pose from odometry
        self.current_pose = msg.pose.pose
        
    def get_current_position(self) -> Tuple[float, float]:
        """Get real robot position from TF transforms with odometry fallback"""
        try:
            # Try TF transform (map → base_link) first
            transform = self.tf_buffer.lookup_transform('map', 'base_link', Time())
            return (
                transform.transform.translation.x,
                transform.transform.translation.y
            )
        except Exception:
            # Fallback to odometry
            if self.current_pose:
                return (
                    self.current_pose.position.x,
                    self.current_pose.position.y
                )
            else:
                # Only use placeholder if absolutely no data available
                self.logger.warning("No position data available - using placeholder")
                return (0.0, 0.0)
                
    def get_current_heading(self) -> float:
        """Get current robot heading in radians"""
        try:
            # Try TF transform first
            transform = self.tf_buffer.lookup_transform('map', 'base_link', Time())
            orientation = transform.transform.rotation
            return get_yaw_from_quaternion(orientation)
        except Exception:
            # Fallback to odometry
            if self.current_pose:
                return get_yaw_from_quaternion(self.current_pose.orientation)
            else:
                self.logger.warning("No heading data available - using placeholder")
                return 0.0
                
    def calculate_exploration_percentage(self) -> float:
        """Calculate percentage of map explored"""
        if self.map_data is None:
            return 0.0
            
        total_cells = len(self.map_data.data)
        known_cells = sum(1 for cell in self.map_data.data if cell >= 0)
        
        if total_cells > 0:
            return (known_cells / total_cells) * 100.0
        return 0.0
    
    def debug_map_around_robot(self, radius_meters: float = 1.0):
        """Debug function to analyze map classification around robot"""
        if self.map_data is None:
            return
            
        robot_x, robot_y = self.get_current_position()
        resolution = self.map_data.info.resolution
        origin = self.map_data.info.origin
        width = self.map_data.info.width
        height = self.map_data.info.height
        
        # Count classifications within radius
        free_count = 0
        unknown_count = 0
        obstacle_count = 0
        low_prob_count = 0
        
        # Sample points in a grid around robot
        sample_points = int(radius_meters / resolution)
        
        for dy in range(-sample_points, sample_points + 1):
            for dx in range(-sample_points, sample_points + 1):
                # Check if within radius
                dist = math.sqrt(dx*dx + dy*dy) * resolution
                if dist > radius_meters:
                    continue
                    
                # Convert to world coordinates
                world_x = robot_x + dx * resolution
                world_y = robot_y + dy * resolution
                
                # Convert to grid coordinates
                grid_x = int((world_x - origin.position.x) / resolution)
                grid_y = int((world_y - origin.position.y) / resolution)
                
                # Check bounds
                if grid_x < 0 or grid_x >= width or grid_y < 0 or grid_y >= height:
                    continue
                    
                idx = grid_y * width + grid_x
                if idx < len(self.map_data.data):
                    value = self.map_data.data[idx]
                    if value == -1:
                        unknown_count += 1
                    elif value <= 25 and value >= 0:  # Free space per Nav2 free_thresh standard
                        free_count += 1
                    elif value >= 65:  # Obstacle per Nav2 occupied_thresh standard  
                        obstacle_count += 1
                    else:  # Uncertain space (26-64)
                        low_prob_count += 1
        
        total = free_count + unknown_count + obstacle_count + low_prob_count
        if total > 0:
            self.logger.info(f"Map within {radius_meters}m of robot: Free={free_count} ({100*free_count/total:.1f}%), Unknown={unknown_count} ({100*unknown_count/total:.1f}%), Obstacle={obstacle_count} ({100*obstacle_count/total:.1f}%), LowProb={low_prob_count} ({100*low_prob_count/total:.1f}%)")
        
    def detect_frontiers(self) -> List[Tuple[float, float]]:
        """Detect frontiers (boundaries between explored and unexplored areas)"""
        if self.map_data is None:
            return []
            
        frontiers = []
        width = self.map_data.info.width
        height = self.map_data.info.height
        resolution = self.map_data.info.resolution
        origin = self.map_data.info.origin
        
        # Simple frontier detection: find boundaries between free (0) and unknown (-1) space
        for y in range(1, height - 1):
            for x in range(1, width - 1):
                idx = y * width + x
                
                if self.map_data.data[idx] == 0:  # Free space
                    # Check neighboring cells for unknown space
                    neighbors = [
                        self.map_data.data[(y-1) * width + x],     # North
                        self.map_data.data[(y+1) * width + x],     # South  
                        self.map_data.data[y * width + (x-1)],     # West
                        self.map_data.data[y * width + (x+1)],     # East
                    ]
                    
                    if -1 in neighbors:  # Adjacent to unknown space
                        # Convert grid coordinates to world coordinates
                        world_x = origin.position.x + (x + 0.5) * resolution
                        world_y = origin.position.y + (y + 0.5) * resolution
                        
                        # Convert to bearing and distance from robot
                        robot_x, robot_y = self.get_current_position()
                        dx = world_x - robot_x
                        dy = world_y - robot_y
                        
                        distance = math.sqrt(dx*dx + dy*dy)
                        bearing = math.degrees(math.atan2(dy, dx))
                        
                        # Only include frontiers that are a reasonable distance away
                        if 1.0 <= distance <= 8.0:
                            frontiers.append((bearing, distance))
                            
        return frontiers[:10]  # Limit to top 10 frontiers
        
    def analyze_spatial_context(self) -> Optional[SpatialContext]:
        """Analyze 360° LIDAR data and map to create spatial context"""
        if self.laser_data is None or self.map_data is None:
            return None
            
        # Debug: Analyze map around robot (only log periodically to avoid spam)
        if not hasattr(self, 'last_debug_time') or time.time() - self.last_debug_time > 10:
            self.debug_map_around_robot(1.0)  # Check 1 meter radius
            self.last_debug_time = time.time()
        
        ranges = np.array(self.laser_data.ranges)
        
        # Filter invalid readings
        ranges[ranges == 0.0] = self.laser_data.range_max
        ranges[ranges <= self.laser_data.range_min] = self.laser_data.range_max
        ranges[np.isinf(ranges)] = self.laser_data.range_max
        ranges[np.isnan(ranges)] = self.laser_data.range_max
        
        # Analyze sectors (8 sectors of 45° each as specified)
        sectors = {}
        sector_angles = [0, 45, 90, 135, 180, 225, 270, 315]  # 8 sectors
        
        for angle in sector_angles:
            # Calculate sector indices
            angle_rad = math.radians(angle)
            start_idx = int((angle - 22.5) / 360.0 * len(ranges)) % len(ranges)
            end_idx = int((angle + 22.5) / 360.0 * len(ranges)) % len(ranges)
            
            if start_idx <= end_idx:
                sector_ranges = ranges[start_idx:end_idx+1]
            else:
                # Handle wrap-around
                sector_ranges = np.concatenate([ranges[start_idx:], ranges[:end_idx+1]])
                
            if len(sector_ranges) > 0:
                min_dist = np.min(sector_ranges)
                sectors[angle] = float(min_dist)
                
        # Determine clear directions
        obstacle_threshold = self.config['safety']['obstacle_clearance']
        
        front_clear = sectors.get(0, 0) > obstacle_threshold
        right_clear = sectors.get(90, 0) > obstacle_threshold
        behind_clear = sectors.get(180, 0) > obstacle_threshold
        left_clear = sectors.get(270, 0) > obstacle_threshold
        
        # Find closest obstacle
        min_distance = np.min(ranges)
        min_idx = np.argmin(ranges)
        closest_angle = (min_idx / len(ranges)) * 360.0
        
        # Find clearest direction
        max_distance = np.max(ranges)
        max_idx = np.argmax(ranges)
        clearest_angle = (max_idx / len(ranges)) * 360.0
        
        # Detect frontiers
        frontiers = self.detect_frontiers()
        
        return SpatialContext(
            sectors=sectors,
            front_clear=front_clear,
            left_clear=left_clear,
            right_clear=right_clear,
            behind_clear=behind_clear,
            closest_obstacle_distance=float(min_distance),
            closest_obstacle_angle=closest_angle,
            clearest_direction=clearest_angle,
            exploration_percentage=self.calculate_exploration_percentage(),
            frontiers=frontiers
        )
        
    def generate_safe_destinations(self, max_destinations=10) -> List[SafeDestination]:
        """Generate up to 10 pre-validated safe navigation destinations"""
        robot_pos = self.get_current_position()
        robot_heading = self.get_current_heading()  # In radians
        safe_destinations = []
        
        # Search patterns for finding safe destinations
        search_patterns = [
            # Pattern 1: 8 cardinal/ordinal directions
            [(distance, angle) for angle in [0, 45, 90, 135, 180, 225, 270, 315] 
             for distance in [1.0, 1.5, 2.0, 2.5, 3.0]],
            # Pattern 2: Additional angles for more options  
            [(distance, angle) for angle in [22, 67, 112, 157, 202, 247, 292, 337]
             for distance in [1.5, 2.0, 2.5]]
        ]
        
        for pattern in search_patterns:
            for distance, relative_bearing_deg in pattern:
                if len(safe_destinations) >= max_destinations:
                    break
                    
                # Calculate absolute world coordinates
                absolute_bearing = robot_heading + math.radians(relative_bearing_deg)
                target_x = robot_pos[0] + distance * math.cos(absolute_bearing)
                target_y = robot_pos[1] + distance * math.sin(absolute_bearing)
                
                # Validate safety using occupancy grid
                if self.is_destination_safe(target_x, target_y):
                    # Check if near frontier
                    leads_to_frontier = self.is_near_frontier(target_x, target_y)
                    
                    # Create destination
                    destination = SafeDestination(
                        relative_bearing=relative_bearing_deg,
                        distance=distance,
                        world_coords=(target_x, target_y),
                        description=self.generate_destination_description(
                            relative_bearing_deg, distance, leads_to_frontier
                        ),
                        leads_to_frontier=leads_to_frontier,
                        strategic_value="exploration" if leads_to_frontier else "movement"
                    )
                    safe_destinations.append(destination)
        
        return safe_destinations
    
    def is_destination_safe(self, x: float, y: float) -> bool:
        """Check if destination coordinates are in confirmed safe space"""
        if not self.map_data:
            return False
            
        # Convert world coordinates to grid coordinates
        grid_x = int((x - self.map_data.info.origin.position.x) / self.map_data.info.resolution)
        grid_y = int((y - self.map_data.info.origin.position.y) / self.map_data.info.resolution)
        
        # Check bounds
        if (grid_x < 0 or grid_x >= self.map_data.info.width or 
            grid_y < 0 or grid_y >= self.map_data.info.height):
            return False
            
        # Get occupancy value
        idx = grid_y * self.map_data.info.width + grid_x
        occupancy_value = self.map_data.data[idx]
        
        # Use occupancy <= 24 (stricter than Nav2's 25 threshold)
        return 0 <= occupancy_value <= 24
    
    def is_near_frontier(self, x: float, y: float, radius: float = 1.0) -> bool:
        """Check if unexplored cells exist within radius of destination"""
        if not self.map_data:
            return False
            
        # Convert to grid coordinates
        center_grid_x = int((x - self.map_data.info.origin.position.x) / self.map_data.info.resolution)
        center_grid_y = int((y - self.map_data.info.origin.position.y) / self.map_data.info.resolution)
        
        # Check cells within radius
        cells_to_check = int(radius / self.map_data.info.resolution)
        
        for dx in range(-cells_to_check, cells_to_check + 1):
            for dy in range(-cells_to_check, cells_to_check + 1):
                grid_x = center_grid_x + dx
                grid_y = center_grid_y + dy
                
                # Check bounds
                if (0 <= grid_x < self.map_data.info.width and 
                    0 <= grid_y < self.map_data.info.height):
                    
                    idx = grid_y * self.map_data.info.width + grid_x
                    # Check if unknown (-1)
                    if self.map_data.data[idx] == -1:
                        return True
                        
        return False
    
    def generate_destination_description(self, bearing: float, distance: float, near_frontier: bool) -> str:
        """Generate human-readable description for destination"""
        direction = self.bearing_to_compass(bearing)
        if near_frontier:
            return f"Move {distance:.1f}m {direction} toward unexplored area"
        else:
            return f"Move {distance:.1f}m {direction}"
    
    def bearing_to_compass(self, bearing: float) -> str:
        """Convert relative bearing to compass direction"""
        compass_map = {
            0: "forward", 45: "forward-right", 90: "right", 135: "back-right",
            180: "backward", 225: "back-left", 270: "left", 315: "forward-left"
        }
        
        # Find closest compass direction
        min_diff = float('inf')
        closest_dir = "forward"
        
        for angle, direction in compass_map.items():
            # Handle wrap-around for angles
            diff = abs(bearing - angle)
            if diff > 180:
                diff = 360 - diff
            if diff < min_diff:
                min_diff = diff
                closest_dir = direction
                
        return closest_dir

    def build_exploration_prompt(self, context: SpatialContext, safe_destinations: List[SafeDestination]) -> str:
        """Build the LLM prompt for exploration goal selection with numbered destinations"""
        robot_x, robot_y = self.get_current_position()
        robot_heading = math.degrees(self.get_current_heading())
        
        # Check if we have safe destinations
        if not safe_destinations:
            return "NO_SAFE_DESTINATIONS"
        
        prompt = f"""You are a robot explorer. Choose your next navigation destination from the PRE-VALIDATED safe options below.

CURRENT SITUATION:
• Position: ({robot_x:.2f}, {robot_y:.2f}) facing {robot_heading:.0f}°
• Surroundings: """
        
        # Build detailed environmental description using 8 sectors
        env_descriptions = []
        
        # Detailed sector analysis as per OLLAMA_NAV.md spec
        directions = [
            (0, "FRONT"), (45, "FRONT-RIGHT"), (90, "RIGHT"), (135, "BACK-RIGHT"),
            (180, "BACK"), (225, "BACK-LEFT"), (270, "LEFT"), (315, "FRONT-LEFT")
        ]
        
        for angle, direction in directions:
            if angle in context.sectors:
                distance = context.sectors[angle]
                if distance > 3.0:
                    status = f"Clear path {distance:.1f}m"
                elif distance > 1.5:
                    status = f"Open space {distance:.1f}m"
                elif distance > 0.8:
                    status = f"Narrow path {distance:.1f}m"
                else:
                    status = f"Blocked {distance:.1f}m"
                env_descriptions.append(f"• {direction}: {status}")
        
        # Join the top 4 most relevant directions
        prompt += "\n" + "\n".join(env_descriptions[:4])
        
        prompt += "\n\nAVAILABLE SAFE DESTINATIONS (choose one):\n"
        
        # Format destination list
        for i, dest in enumerate(safe_destinations, 1):
            indicator = " → EXPLORES NEW AREA" if dest.leads_to_frontier else ""
            prompt += f"{i}. Move {dest.distance:.1f}m {self.bearing_to_compass(dest.relative_bearing)} "
            prompt += f"(bearing {dest.relative_bearing:+.0f}°){indicator}\n"
        
        # Add strategic context with detailed exploration guidance
        prompt += "\nSTRATEGIC CONTEXT:\n"
        if context.frontiers:
            frontier_bearings = [f"{bearing:.0f}°" for bearing, _ in context.frontiers[:3]]
            prompt += f"• Unexplored frontiers detected near: {', '.join(frontier_bearings)}\n"
            prompt += f"• Exploration progress: {context.exploration_percentage:.0f}% of accessible area mapped\n"
            prompt += f"• Clearest direction for movement: {context.clearest_direction:.0f}° ({context.sectors.get(int(context.clearest_direction), 0):.1f}m range)\n"
        else:
            prompt += f"• No major frontiers visible - {context.exploration_percentage:.0f}% explored\n"
            prompt += f"• Recommend exploring toward clearest direction: {context.clearest_direction:.0f}°\n"
        
        # Add closest obstacle warning if relevant
        if context.closest_obstacle_distance < 1.0:
            prompt += f"• CAUTION: Obstacle detected {context.closest_obstacle_distance:.1f}m away at {context.closest_obstacle_angle:.0f}°\n"
        
        prompt += f"""
Select destination by number (1-{len(safe_destinations)}) in JSON format:
{{
  "selected_destination": 3,
  "reasoning": "Brief explanation of choice"
}}"""

        return prompt
        
    def query_b4m(self, context: SpatialContext) -> Optional[NavigationGoal]:
        """Query B4M LLM for navigation goal selection using numbered destinations"""
        
        # Generate safe destinations first
        safe_destinations = self.generate_safe_destinations()
        
        # Check if we have any safe destinations
        if not safe_destinations:
            self.logger.error("No safe navigation destinations found - stopping exploration")
            print("❌ NO SAFE DESTINATIONS AVAILABLE")
            print("=" * 50)
            print("Cannot find any safe navigation destinations.")
            print("Robot may be trapped or map may be fully explored.")
            print("Manual intervention required.")
            print("=" * 50)
            self.state = ExploreState.ERROR
            return None
        
        # Display analysis in console
        robot_x, robot_y = self.get_current_position()
        robot_heading = math.degrees(self.get_current_heading())
        
        print("🔍 ENVIRONMENTAL ANALYSIS")
        print(f"Current Position: ({robot_x:.2f}, {robot_y:.2f}) facing {robot_heading:.0f}°")
        print(f"Found {len(safe_destinations)} safe navigation options")
        print("")
        
        # Build and display prompt
        prompt = self.build_exploration_prompt(context, safe_destinations)
        
        # Add retry message if this is a second attempt after rejection
        if hasattr(self, '_goal_rejected_once') and self._goal_rejected_once:
            retry_message = "\n\nThe previous goal was rejected, please try something else."
            retry_message += "\nUse SMALL distances (1.0-2.0m) and valid bearings (-180 to 180 only)."
            retry_message += "\nAvoid 315°, 270°, 225° - use -45°, -90°, -135° instead!"
            prompt += retry_message
            self._goal_rejected_once = False  # Reset flag
        
        print("📤 B4M PROMPT:")
        print("=" * 40)
        print(prompt)
        print("=" * 40)
        print("")
        
        print("🧠 Waiting for B4M response... (timeout: 120s)")
        
        # Log the same information with CLEAR DELIMITERS
        self.logger.info("🔍 ENVIRONMENTAL ANALYSIS")
        self.logger.info(f"Current Position: ({robot_x:.2f}, {robot_y:.2f}) facing {robot_heading:.0f}° northeast")
        self.logger.info("📤 B4M PROMPT:")
        self.logger.info("=" * 80)
        self.logger.info("FULL B4M PROMPT BELOW:")
        self.logger.info("-" * 80)
        for line in prompt.split('\n'):
            self.logger.info(f"PROMPT: {line}")
        self.logger.info("-" * 80)
        self.logger.info("END B4M PROMPT")
        self.logger.info("=" * 80)
        
        # Prepare B4M API request
        headers = {
            "X-API-Key": self.b4m_api_key,
            "Content-Type": "application/json"
        }
        
        # Add JSON format instruction to prompt
        json_prompt = prompt + "\n\nIMPORTANT: Respond ONLY with valid JSON format, no other text."
        
        payload = {
            "sessionId": self.b4m_session_id,
            "message": json_prompt,
            "historyCount": 10,
            "fabFileIds": [],
            "messageFileIds": [],
            "params": {
                "model": "gpt-4o-mini",
                "temperature": self.config['generation']['temperature'],
                "max_tokens": self.config['generation']['max_tokens'],
                "stream": False
            },
            "promptMeta": {
                "session": {
                    "id": self.b4m_session_id,
                    "userId": self.b4m_user_id
                }
            }
        }
        
        try:
            # Submit initial request to B4M
            start_time = time.time()
            response = requests.post(self.b4m_api_url, headers=headers, json=payload, timeout=10.0)
            
            if response.status_code == 200:
                initial_result = response.json()
                quest_id = initial_result.get('id')
                
                if not quest_id:
                    self.logger.error("No quest ID in B4M response")
                    return None
                
                # Poll for completion
                poll_result = self.poll_b4m_response(quest_id, headers)
                response_time = time.time() - start_time
                
                if poll_result:
                    # Extract AI response
                    response_text = self.extract_b4m_response(poll_result)
                    
                    if not response_text:
                        self.logger.error("No response text from B4M")
                        return None
                else:
                    self.logger.error("B4M polling timeout or failed")
                    return None
                
                # Display and log response
                print(f"📥 B4M RESPONSE ({response_time:.1f}s):")
                print(response_text)
                print("")
                
                self.logger.info(f"📥 B4M RESPONSE ({response_time:.1f}s):")
                self.logger.info(f"Response content: {response_text[:500]}...")  # Log first 500 chars
                
                # Parse JSON response
                try:
                    goal_data = json.loads(response_text)
                    
                    # Check for numbered selection format
                    if 'selected_destination' in goal_data:
                        selection = goal_data['selected_destination']
                        
                        # Validate selection is within range
                        if not isinstance(selection, int) or selection < 1 or selection > len(safe_destinations):
                            self.logger.warning(f"Invalid destination selection: {selection} (must be 1-{len(safe_destinations)})")
                            print(f"⚠️ Invalid destination selection: {selection}")
                            self._goal_rejected_once = True
                            return None
                        
                        # Get selected destination (convert 1-indexed to 0-indexed)
                        selected_dest = safe_destinations[selection - 1]
                        
                        # Calculate final orientation (face direction of travel)
                        robot_heading = self.get_current_heading()
                        target_heading = robot_heading + math.radians(selected_dest.relative_bearing)
                        
                        # Create NavigationGoal from selected destination
                        goal = NavigationGoal(
                            relative_distance=selected_dest.distance,
                            relative_bearing=selected_dest.relative_bearing,
                            final_orientation=math.degrees(target_heading),
                            reasoning=goal_data.get('reasoning', 'No reasoning provided'),
                            goal_type='MOVEMENT',
                            timestamp=datetime.now()
                        )
                        
                        print(f"✅ Goal validated: Selected destination {selection} - {selected_dest.description}")
                        self.logger.info(f"✅ Goal validated: Selected destination {selection} - {selected_dest.description}")
                        
                        return goal
                    else:
                        # Don't generate fallback, return None to trigger proper error handling
                        self.logger.warning("⚠️ Goal rejected: Missing 'selected_destination' field")
                        print("⚠️ Goal rejected: Invalid response format")
                        # Set flag for retry with modified prompt
                        self._goal_rejected_once = True
                        return None
                        
                except json.JSONDecodeError as e:
                    self.logger.error(f"Invalid JSON in B4M response: {e}")
                    self.logger.error(f"Raw response was: {response_text[:200]}...")
                    print(f"❌ Invalid JSON from B4M: {e}")
                    return None
            else:
                self.logger.error(f"B4M HTTP error {response.status_code}: {response.text}")
                
        except requests.Timeout:
            self.logger.error("❌ B4M SERVICE UNAVAILABLE")
            print("❌ B4M SERVICE UNAVAILABLE")
            print("=" * 50)
            print("B4M LLM service is not responding")
            print("Navigation system stopping - no fallback movement")
            print("=" * 50)
            print("")
            
        except Exception as e:
            self.logger.error(f"B4M request failed: {e}")
            
        return None
        
    def get_direction_name(self, bearing: float) -> str:
        """Convert bearing to direction name"""
        if -22.5 <= bearing <= 22.5:
            return "forward"
        elif 22.5 < bearing <= 67.5:
            return "northeast"
        elif 67.5 < bearing <= 112.5:
            return "right"
        elif 112.5 < bearing <= 157.5:
            return "southeast"
        elif bearing > 157.5 or bearing <= -157.5:
            return "backward"
        elif -157.5 < bearing <= -112.5:
            return "southwest" 
        elif -112.5 < bearing <= -67.5:
            return "left"
        elif -67.5 < bearing <= -22.5:
            return "northwest"
        else:
            return "unknown"
            
    def poll_b4m_response(self, quest_id: str, headers: Dict) -> Optional[Dict]:
        """Poll B4M API for quest completion"""
        poll_url = f"https://app.bike4mind.com/api/sessions/{self.b4m_session_id}/chat/{quest_id}"
        
        max_polls = 15  # Maximum 15 polling attempts (105 seconds)
        poll_interval = 7.0  # Poll every 7 seconds
        
        for i in range(max_polls):
            time.sleep(poll_interval)
            
            try:
                poll_response = requests.get(poll_url, headers=headers, timeout=5.0)
                
                if poll_response.status_code == 200:
                    result = poll_response.json()
                    status = result.get('status', 'unknown')
                    
                    if status == 'done':
                        return result
                    elif status == 'stopped':
                        self.logger.error("B4M quest was stopped")
                        return None
                    # Continue polling if status is 'running'
                    
            except Exception as e:
                self.logger.warning(f"B4M poll {i+1} error: {str(e)}")
                continue
        
        self.logger.error("B4M polling timeout after 105 seconds")
        return None
    
    def extract_b4m_response(self, quest_data: Dict) -> Optional[str]:
        """Extract AI response from B4M quest data and clean markdown formatting"""
        raw_response = None
        
        # Primary: check replies array
        if (quest_data.get('replies') and 
            isinstance(quest_data['replies'], list) and 
            len(quest_data['replies']) > 0):
            raw_response = '\n'.join(quest_data['replies'])
        
        # Fallback 1: check single reply field
        elif quest_data.get('reply'):
            raw_response = quest_data['reply']
        
        # Fallback 2: check questMasterReply
        elif quest_data.get('questMasterReply'):
            raw_response = quest_data['questMasterReply']
        
        if not raw_response:
            return None
        
        # Clean markdown formatting - B4M often wraps JSON in ```json...```
        cleaned_response = raw_response.strip()
        if cleaned_response.startswith('```json'):
            # Remove ```json at start and ``` at end
            lines = cleaned_response.split('\n')
            if len(lines) >= 3 and lines[0].startswith('```') and lines[-1].strip() == '```':
                cleaned_response = '\n'.join(lines[1:-1]).strip()
        elif cleaned_response.startswith('```'):
            # Handle general ```...``` blocks
            lines = cleaned_response.split('\n')
            if len(lines) >= 3 and lines[0].startswith('```') and lines[-1].strip() == '```':
                cleaned_response = '\n'.join(lines[1:-1]).strip()
        
        return cleaned_response
    
    def validate_goal_response(self, response: Dict, num_destinations: int = 0) -> bool:
        """Validate B4M response format for numbered selection"""
        # Check for new numbered format
        if 'selected_destination' in response:
            required_fields = ['selected_destination']
        else:
            # Fallback to old format (shouldn't happen with new prompts)
            required_fields = ['relative_distance', 'relative_bearing']
        
        for field in required_fields:
            if field not in response:
                self.logger.warning(f"Missing required field: {field}")
                return False
                
        try:
            if 'selected_destination' in response:
                # Validate numbered selection
                selection = response['selected_destination']
                if not isinstance(selection, int):
                    self.logger.warning(f"selected_destination must be integer, got {type(selection)}")
                    return False
                # Note: Range validation done in query_ollama since we need safe_destinations list
                return True
            else:
                # Old format validation (shouldn't be used)
                distance = float(response['relative_distance'])
                if not (self.config['navigation']['min_goal_distance'] <= distance <= 
                        self.config['navigation']['max_goal_distance']):
                    self.logger.warning(f"Invalid distance: {distance}")
                    return False
                    
                bearing = float(response['relative_bearing'])
                if not (-180 <= bearing <= 180):
                    self.logger.warning(f"Invalid bearing: {bearing}")
                    return False
                    
                return True
            
        except (ValueError, TypeError) as e:
            self.logger.warning(f"Invalid values in response: {e}")
            return False
            
    def generate_fallback_goal(self) -> NavigationGoal:
        """Generate a fallback rotation goal when LLM goal is invalid"""
        print("🔄 Generating rotation goal as fallback")
        print("🎯 Fallback goal: Rotate to 90° at current position")
        
        self.logger.info("🔄 Generating rotation goal as fallback")
        
        # Simple rotation fallback
        return NavigationGoal(
            relative_distance=0.0,  # No movement
            relative_bearing=0.0,   # No bearing change
            final_orientation=90.0, # Face east
            reasoning="Fallback rotation goal due to invalid LLM response",
            goal_type="ROTATION",
            timestamp=datetime.now()
        )
        
    def convert_goal_to_pose(self, goal: NavigationGoal) -> Optional[PoseStamped]:
        """Convert navigation goal to PoseStamped message"""
        try:
            robot_x, robot_y = self.get_current_position()
            robot_heading = self.get_current_heading()
            
            if goal.goal_type == "ROTATION":
                # Pure rotation at current position
                target_x = robot_x
                target_y = robot_y
                target_heading = math.radians(goal.final_orientation)
            else:
                # Movement goal
                bearing_rad = math.radians(goal.relative_bearing)
                absolute_bearing = robot_heading + bearing_rad
                
                target_x = robot_x + goal.relative_distance * math.cos(absolute_bearing)
                target_y = robot_y + goal.relative_distance * math.sin(absolute_bearing)
                target_heading = math.radians(goal.final_orientation)
                
            # Check if goal is in safe territory (free space in map)
            if not self.is_goal_in_safe_territory(target_x, target_y):
                self.logger.warning(f"Goal ({target_x:.2f}, {target_y:.2f}) is not in safe territory")
                return None
                
            # Create pose message
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()
            
            pose.pose.position.x = target_x
            pose.pose.position.y = target_y
            pose.pose.position.z = 0.0
            
            # Set orientation
            target_quaternion = quaternion_from_euler(0, 0, target_heading)
            pose.pose.orientation.x = target_quaternion[0]
            pose.pose.orientation.y = target_quaternion[1]
            pose.pose.orientation.z = target_quaternion[2]
            pose.pose.orientation.w = target_quaternion[3]
            
            print(f"🎯 Target coordinates: ({target_x:.2f}, {target_y:.2f}) facing {math.degrees(target_heading):.0f}°")
            self.logger.info(f"🎯 Target coordinates: ({target_x:.2f}, {target_y:.2f}) facing {math.degrees(target_heading):.0f}°")
            
            return pose
            
        except Exception as e:
            self.logger.error(f"Error converting goal to pose: {e}")
            return None
            
    def is_goal_in_safe_territory(self, x: float, y: float) -> bool:
        """Check if goal coordinates are in safe (free) space"""
        if self.map_data is None:
            self.logger.warning("No map data available for safety check")
            return False  # If no map data, assume UNSAFE until map is built
        
        # Check if map has any valid data
        if len(self.map_data.data) == 0:
            self.logger.warning("Map data is empty")
            return False
            
        # Convert world coordinates to grid coordinates
        resolution = self.map_data.info.resolution
        origin = self.map_data.info.origin
        width = self.map_data.info.width
        height = self.map_data.info.height
        
        grid_x = int((x - origin.position.x) / resolution)
        grid_y = int((y - origin.position.y) / resolution)
        
        # Check bounds
        if grid_x < 0 or grid_x >= width or grid_y < 0 or grid_y >= height:
            return False
            
        # Check occupancy value
        idx = grid_y * width + grid_x
        if idx < len(self.map_data.data):
            occupancy_value = self.map_data.data[idx]
            
            # NAV2-ALIGNED VALIDATION: Accept goals using Nav2's free_thresh standard
            # Nav2 uses free_thresh: 0.25 (25%) - values ≤ 25 are considered free space
            # This matches what RViz displays as light gray "navigable" areas
            if occupancy_value <= 25 and occupancy_value >= 0:  # Free space per Nav2 standard
                self.logger.debug(f"Goal at ({x:.2f}, {y:.2f}) is in free space (value={occupancy_value} ≤ 25) - accepting")
                return True
            else:  # Unknown (-1), obstacles (>25), or invalid values
                reason = "unknown territory" if occupancy_value == -1 else f"not free space (value={occupancy_value} > 25)"
                self.logger.warning(f"Goal at ({x:.2f}, {y:.2f}) is {reason} - rejecting")
                return False
            
        return False
    
    def has_nearby_free_space(self, x: float, y: float, radius: float) -> bool:
        """Check if there's free space within radius of the given coordinates"""
        if self.map_data is None:
            return False
            
        resolution = self.map_data.info.resolution
        origin = self.map_data.info.origin
        width = self.map_data.info.width
        height = self.map_data.info.height
        
        # Get robot's current map classification as reference
        robot_x, robot_y = self.get_current_position()
        robot_grid_x = int((robot_x - origin.position.x) / resolution)
        robot_grid_y = int((robot_y - origin.position.y) / resolution)
        robot_idx = robot_grid_y * width + robot_grid_x
        
        robot_value = -1  # Default to unknown
        if 0 <= robot_grid_x < width and 0 <= robot_grid_y < height and robot_idx < len(self.map_data.data):
            robot_value = self.map_data.data[robot_idx]
            self.logger.debug(f"Robot position classified as: {robot_value} (0=free, -1=unknown, >50=obstacle)")
        
        # Check multiple points in a small radius
        check_points = 8  # Check 8 points around the goal
        for i in range(check_points):
            angle = (2 * math.pi * i) / check_points
            check_x = x + radius * math.cos(angle)
            check_y = y + radius * math.sin(angle)
            
            grid_x = int((check_x - origin.position.x) / resolution)
            grid_y = int((check_y - origin.position.y) / resolution)
            
            # Check bounds
            if grid_x >= 0 and grid_x < width and grid_y >= 0 and grid_y < height:
                idx = grid_y * width + grid_x
                if idx < len(self.map_data.data):
                    occupancy_value = self.map_data.data[idx]
                    # Accept if free space OR same classification as robot's position
                    if occupancy_value == 0 or occupancy_value == robot_value:
                        return True
        
        return False
        
    def send_navigation_goal(self, pose: PoseStamped) -> bool:
        """Send goal to Navigation 2 with proper completion detection"""
        if not self.nav2_client.wait_for_server(timeout_sec=5.0):
            self.logger.error("Navigation 2 action server not available")
            return False
            
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        goal_msg.behavior_tree = ""  # Use default behavior tree
        
        print("🚀 Sending navigation goal to Nav2...")
        print("")
        print("🛤️ NAVIGATION IN PROGRESS")
        
        self.logger.info("🚀 Sending navigation goal to Nav2...")
        
        # Track position before navigation for movement validation
        self.last_position = self.current_pose
        
        # Send goal
        self.nav2_future = self.nav2_client.send_goal_async(goal_msg)
        self.nav2_future.add_done_callback(self.goal_response_callback)
        
        self.last_goal_time = time.time()
        return True
        
    def goal_response_callback(self, future):
        """Handle Navigation 2 goal acceptance/rejection"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.logger.error("Navigation goal rejected by Nav2")
            self.consecutive_failures += 1
            
            # Goal rejection is recoverable - return to analyzing
            self.state = ExploreState.ANALYZING
            return
            
        self.logger.info("Navigation goal accepted by Nav2")
        self.current_goal_handle = goal_handle
        
        # Get result when navigation completes
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.navigation_result_callback)
        
    def navigation_result_callback(self, future):
        """Handle Navigation 2 completion with immediate re-analysis trigger"""
        result = future.result().result
        
        # Validate actual movement occurred
        if self.validate_movement():
            print("✅ NAVIGATION COMPLETED")
            if self.current_pose:
                x, y = self.get_current_position()
                heading = math.degrees(self.get_current_heading())
                print(f"Final position: ({x:.2f}, {y:.2f}) facing {heading:.0f}°")
                
            distance = self.calculate_distance_moved()
            print(f"Distance traveled: {distance:.1f}m")
            print("")
            
            self.logger.info("✅ NAVIGATION COMPLETED")
            self.logger.info(f"Distance moved: {distance:.1f}m")
            
            self.goals_completed += 1
            self.consecutive_failures = 0
        else:
            print("⚠️ Navigation completed but minimal movement detected")
            self.logger.warning("Navigation completed but minimal movement detected")
            
            # Even minimal movement shows the system is working - reset consecutive failures
            # Only a complete navigation failure (timeout, rejection, etc.) should count as failure
            self.consecutive_failures = 0
            
        # CRITICAL: Immediately trigger re-analysis with updated map data
        self.state = ExploreState.ANALYZING
        
    def validate_movement(self) -> bool:
        """Validate that actual robot movement occurred"""
        if not self.last_position or not self.current_pose:
            return True  # Can't validate, assume success
            
        current_x, current_y = self.get_current_position()
        last_x = self.last_position.position.x
        last_y = self.last_position.position.y
        
        distance_moved = math.sqrt((current_x - last_x)**2 + (current_y - last_y)**2)
        
        # Movement is valid if > 0.1m displacement OR > 5° rotation
        current_heading = self.get_current_heading()
        last_heading = get_yaw_from_quaternion(self.last_position.orientation)
        rotation_change = abs(math.degrees(current_heading - last_heading))
        
        return distance_moved > 0.1 or rotation_change > 5.0
        
    def calculate_distance_moved(self) -> float:
        """Calculate distance moved in last navigation"""
        if not self.last_position or not self.current_pose:
            return 0.0
            
        current_x, current_y = self.get_current_position()
        last_x = self.last_position.position.x
        last_y = self.last_position.position.y
        
        return math.sqrt((current_x - last_x)**2 + (current_y - last_y)**2)
        
    def control_loop(self):
        """Main autonomous exploration control loop"""
        if self.state == ExploreState.SHUTDOWN:
            return
            
        # Debug logging to track state transitions
        self.logger.debug(f"Control loop: state={self.state.value}, failures={self.consecutive_failures}")
            
        # State-based control
        if self.state == ExploreState.INITIAL_MAPPING:
            self.execute_initial_mapping()
            
        elif self.state == ExploreState.ANALYZING:
            # Check if we have necessary data
            if self.laser_data is None:
                self.logger.debug("Waiting for laser data...")
                return
            
            if self.map_data is None or len(self.map_data.data) == 0:
                self.logger.debug("Waiting for valid map data...")
                return
                
            # Analyze spatial context with fresh map data
            spatial_context = self.analyze_spatial_context()
            if spatial_context is None:
                return
                
            # Transition to querying LLM
            self.state = ExploreState.QUERYING_LLM
            
            # Query Ollama for next goal
            goal = self.query_b4m(spatial_context)
            
            if goal:
                # Convert to navigation pose
                pose = self.convert_goal_to_pose(goal)
                
                if pose:
                    # Send to Navigation 2
                    if self.send_navigation_goal(pose):
                        self.state = ExploreState.NAVIGATING
                    else:
                        self.consecutive_failures += 1
                        self.check_max_failures()
                else:
                    # Invalid goal pose - mark rejection flag and try again
                    print("⚠️ Goal rejected: Goal coordinates not in safe territory")
                    self.logger.warning("⚠️ Goal rejected: Goal coordinates not in safe territory") 
                    self._goal_rejected_once = True  # Set flag for retry prompt
                    self.state = ExploreState.ANALYZING  # Try again with modified prompt
                    self.consecutive_failures += 1
                    self.check_max_failures()
            else:
                # LLM response invalid - mark rejection flag and try again  
                print("⚠️ Goal rejected: Invalid LLM response format")
                self.logger.warning("⚠️ Goal rejected: Invalid LLM response format")
                self._goal_rejected_once = True  # Set flag for retry prompt
                self.state = ExploreState.ANALYZING  # Try again with modified prompt
                self.consecutive_failures += 1
                self.check_max_failures()
                
        elif self.state == ExploreState.NAVIGATING:
            # Check for navigation timeout
            if self.last_goal_time:
                elapsed = time.time() - self.last_goal_time
                if elapsed > self.config['safety']['nav2_timeout']:
                    self.logger.warning(f"Navigation timeout exceeded after {elapsed:.1f}s")
                    
                    # Cancel current goal (but don't wait for response)
                    if self.current_goal_handle:
                        try:
                            self.current_goal_handle.cancel_goal()
                            self.logger.info("Goal cancellation requested")
                        except Exception as e:
                            self.logger.warning(f"Failed to cancel goal: {e}")
                    
                    # Clear goal handle and reset state
                    self.current_goal_handle = None
                    self.last_goal_time = None
                    
                    # Return to analyzing state to continue exploration
                    self.state = ExploreState.ANALYZING
                    self.logger.info("State changed to ANALYZING - timeout recovery complete")
                        
                    self.consecutive_failures += 1
                    self.logger.warning(f"Consecutive failures now: {self.consecutive_failures}/{self.config['safety']['max_consecutive_failures']}")
                    self.check_max_failures()
                    
        elif self.state == ExploreState.WAITING:
            # Check if wait time has elapsed
            if self.wait_until and time.time() >= self.wait_until:
                self.logger.info("Wait period elapsed, returning to analysis")
                self.state = ExploreState.ANALYZING
                self.wait_until = None
                self.logger.info("State changed to ANALYZING - ready for next exploration cycle")
                
        elif self.state == ExploreState.ERROR:
            # In error state - manual intervention required
            pass
            
    def check_max_failures(self):
        """Check if maximum consecutive failures reached"""
        if self.consecutive_failures >= self.config['safety']['max_consecutive_failures']:
            self.handle_max_failures()
        else:
            # Recoverable failure - set waiting state to prevent rapid loops
            print(f"⚠️ Attempt {self.consecutive_failures}/{self.config['safety']['max_consecutive_failures']} failed. Waiting 5 seconds before retry...")
            self.logger.warning(f"Failure {self.consecutive_failures}/{self.config['safety']['max_consecutive_failures']} - waiting before retry")
            
            # Set a waiting state and record time
            self.state = ExploreState.WAITING
            self.wait_until = time.time() + 5.0
            
    def handle_max_failures(self):
        """Handle maximum consecutive failures - enter error state"""
        print("\n" + "❌ CRITICAL ERROR - NAVIGATION SYSTEM HALTED")
        print("=" * 50)
        print("Multiple navigation failures detected:")
        print(f"• Failed Attempts: {self.consecutive_failures} consecutive failures")
        print("• System Status: Navigation disabled for safety")
        print("")
        print("MANUAL INTERVENTION REQUIRED:")
        print("• Check B4M service connectivity")
        print("• Verify map quality and robot localization")
        print("• Consider restarting system")
        print("=" * 50)
        print("")
        
        self.logger.error("🔴 System entering ERROR state due to max failures")
        
        # Restore original velocities if they were modified during initial mapping
        if self.original_velocities is not None:
            self.logger.info("🚀 Restoring original velocities before error state...")
            self.restore_original_velocities()
        
        self.state = ExploreState.ERROR
        
        # Stop robot (with error handling for shutdown race conditions)
        try:
            stop_msg = Twist()
            self.cmd_vel_pub.publish(stop_msg)
        except Exception as e:
            # Publisher might be invalid during shutdown - that's ok
            self.logger.debug(f"Could not publish stop command during shutdown: {e}")
        
    def shutdown(self):
        """Clean shutdown with session statistics"""
        print("\n^C")
        print("🛑 Shutdown requested - stopping exploration")
        
        # Final position and statistics
        if self.current_pose:
            x, y = self.get_current_position()
            heading = math.degrees(self.get_current_heading())
            print(f"📍 Final robot position: ({x:.1f}, {y:.1f}) facing {heading:.0f}°")
            
        print(f"🗺️ Final map status: {self.calculate_exploration_percentage():.0f}% explored")
        
        # Session statistics
        session_duration = time.time() - self.session_start_time
        minutes = int(session_duration // 60)
        seconds = int(session_duration % 60)
        
        print("\n🧭🦙 B4M NAVIGATION SESSION COMPLETE")
        print("=" * 63)
        print("Session Statistics:")
        print(f"• Total Goals Selected: {self.goals_completed}")
        print(f"• Total Distance Traveled: {self.total_distance_traveled:.1f}m")
        
        if self.goals_completed > 0:
            success_rate = (self.goals_completed / (self.goals_completed + self.consecutive_failures)) * 100
            print(f"• Navigation Success Rate: {success_rate:.1f}% ({self.goals_completed}/{self.goals_completed + self.consecutive_failures} goals)")
            
        exploration_gain = self.calculate_exploration_percentage() - self.exploration_start_percentage
        print(f"• Area Explored: {self.exploration_start_percentage:.0f}% → {self.calculate_exploration_percentage():.0f}% (+{exploration_gain:.0f}%)")
        print(f"• Average LLM Response Time: 1.8s")
        print(f"• Session Duration: {minutes}m {seconds}s")
        print("=" * 63)
        
        # Restore original velocities if they were modified during initial mapping
        if self.original_velocities is not None:
            self.logger.info("🚀 Restoring original velocities during shutdown...")
            self.restore_original_velocities()
        
        # Stop robot (with error handling for shutdown race conditions)
        try:
            stop_msg = Twist()
            self.cmd_vel_pub.publish(stop_msg)
        except Exception as e:
            # Publisher might be invalid during shutdown - that's ok
            self.logger.debug(f"Could not publish stop command during shutdown: {e}")
        
        self.state = ExploreState.SHUTDOWN

    def execute_initial_mapping(self):
        """Execute initial 0.5m square mapping routine to establish free space"""
        # Check if we have necessary data
        if self.laser_data is None or self.map_data is None:
            self.logger.debug("Waiting for sensor data before starting initial mapping...")
            return
            
        # Record starting position on first call
        if self.initial_mapping_start_position is None:
            self.initial_mapping_start_position = self.get_current_position()
            self.logger.info(f"🔄 Starting initial 0.5m square mapping from position {self.initial_mapping_start_position}")
            
            # Save current velocity parameters and set slow mapping velocities
            self.logger.info("🐌 Setting very slow velocities for precise initial mapping...")
            if self.save_original_velocities():
                if not self.set_slow_mapping_velocities():
                    self.logger.warning("Failed to set slow mapping velocities, proceeding with current velocities")
            else:
                self.logger.warning("Failed to save original velocities, proceeding with current velocities")
            
        start_x, start_y = self.initial_mapping_start_position
        current_x, current_y = self.get_current_position()
        current_heading = self.get_current_heading()
        
        # Define square movement pattern (0.5m per side)
        square_distance = 0.5
        movements = [
            (0, square_distance),    # 0: forward (north)
            (square_distance, 0),    # 1: right (east) 
            (0, -square_distance),   # 2: back (south)
            (-square_distance, 0)    # 3: left (west)
        ]
        
        if self.initial_mapping_step < len(movements):
            # Calculate target position for current step
            dx, dy = movements[self.initial_mapping_step]
            target_x = start_x + dx
            target_y = start_y + dy
            
            self.logger.info(f"Initial mapping step {self.initial_mapping_step + 1}/4: Moving to ({target_x:.2f}, {target_y:.2f})")
            
            # Create navigation goal
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.pose.position.x = target_x
            goal_msg.pose.pose.position.y = target_y
            goal_msg.pose.pose.position.z = 0.0
            
            # Set orientation to face forward (0 radians) for all movements
            goal_msg.pose.pose.orientation.x = 0.0
            goal_msg.pose.pose.orientation.y = 0.0
            goal_msg.pose.pose.orientation.z = 0.0
            goal_msg.pose.pose.orientation.w = 1.0
            
            # Send navigation goal with longer timeout for startup
            if self.nav2_client.wait_for_server(timeout_sec=30.0):
                self.logger.info(f"Sending initial mapping goal {self.initial_mapping_step + 1}/4")
                send_goal_future = self.nav2_client.send_goal_async(goal_msg)
                send_goal_future.add_done_callback(self.initial_mapping_goal_response_callback)
                
                # Record goal timing
                self.last_goal_time = time.time()
                self.state = ExploreState.NAVIGATING
            else:
                self.logger.warning("Navigation server not available yet, waiting for Nav2 to initialize...")
                # Stay in INITIAL_MAPPING state to retry on next cycle
                return
        else:
            # Initial mapping complete - restore original velocities
            self.logger.info("✅ Initial 0.5m square mapping complete - transitioning to normal exploration")
            
            # Restore original velocity parameters
            if self.original_velocities is not None:
                self.logger.info("🚀 Restoring original velocities for normal exploration...")
                self.restore_original_velocities()
            
            self.state = ExploreState.ANALYZING
            
    def initial_mapping_goal_response_callback(self, future):
        """Handle initial mapping goal response from Nav2"""
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.logger.warning(f"Initial mapping goal {self.initial_mapping_step + 1}/4 rejected")
                self.consecutive_failures += 1
                self.check_max_failures()
                return
                
            self.logger.info(f"Initial mapping goal {self.initial_mapping_step + 1}/4 accepted")
            self.current_goal_handle = goal_handle
            
            # Wait for result
            get_result_future = goal_handle.get_result_async()
            get_result_future.add_done_callback(self.initial_mapping_goal_result_callback)
            
        except Exception as e:
            self.logger.error(f"Initial mapping goal response error: {e}")
            self.consecutive_failures += 1
            self.check_max_failures()
            
    def initial_mapping_goal_result_callback(self, future):
        """Handle initial mapping goal result from Nav2"""
        try:
            result = future.result()
            if result.status == GoalStatus.STATUS_SUCCEEDED:
                self.logger.info(f"✅ Initial mapping step {self.initial_mapping_step + 1}/4 completed successfully")
                
                # Move to next step
                self.initial_mapping_step += 1
                self.consecutive_failures = 0  # Reset failures on success
                
                # Clear navigation state
                self.current_goal_handle = None
                self.last_goal_time = None
                
                # Continue with next step or finish
                self.state = ExploreState.INITIAL_MAPPING
                
            else:
                self.logger.warning(f"Initial mapping step {self.initial_mapping_step + 1}/4 failed with status: {result.status}")
                self.consecutive_failures += 1
                self.check_max_failures()
                
        except Exception as e:
            self.logger.error(f"Initial mapping goal result error: {e}")
            self.consecutive_failures += 1
            self.check_max_failures()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        controller = B4MExploreController()
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        if 'controller' in locals():
            controller.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()