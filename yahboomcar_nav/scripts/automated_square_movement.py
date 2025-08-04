#!/usr/bin/env python3

"""
Automated Square Movement Script for SLAM Testing
Moves robot in 1-meter square perimeter pattern for automated SLAM validation
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
import time
import json
import os

class AutomatedSquareMovement(Node):
    def __init__(self):
        super().__init__('automated_square_movement')
        
        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # Movement parameters
        self.SQUARE_SIZE = 1.0  # meters
        self.LINEAR_SPEED = 0.2  # m/s
        self.ANGULAR_SPEED = 0.5  # rad/s
        self.POSITION_TOLERANCE = 0.05  # meters
        self.ANGLE_TOLERANCE = 0.1  # radians
        
        # State tracking
        self.current_odom = None
        self.start_pose = None
        self.current_side = 0  # 0-3 for four sides of square
        self.movement_state = 'starting'  # starting, moving_forward, turning
        self.obstacles_detected = []
        self.last_scan = None
        
        # Results tracking
        self.results = {
            'start_time': time.time(),
            'start_position': None,
            'final_position': None,
            'obstacles_detected': 0,
            'obstacle_positions': [],
            'sides_completed': 0,
            'loop_closed': False,
            'total_distance': 0.0,
            'return_accuracy': None,
            'success': False
        }
        
        # Timer for movement control
        self.timer = self.create_timer(0.1, self.movement_control)
        
        self.get_logger().info("Automated Square Movement initialized")
        self.get_logger().info(f"Square size: {self.SQUARE_SIZE}m, Linear speed: {self.LINEAR_SPEED}m/s")
        
        # Wait for topics to be available
        self.get_logger().info("Waiting for odometry and scan topics...")
        time.sleep(3)  # Give time for all systems to initialize
        
    def odom_callback(self, msg):
        self.current_odom = msg
        if self.start_pose is None and self.movement_state == 'starting':
            self.start_pose = msg.pose.pose
            self.results['start_position'] = {
                'x': msg.pose.pose.position.x,
                'y': msg.pose.pose.position.y,
                'z': msg.pose.pose.orientation.z,
                'w': msg.pose.pose.orientation.w
            }
            self.get_logger().info(f"Start position recorded: x={msg.pose.pose.position.x:.3f}, y={msg.pose.pose.position.y:.3f}")
            self.movement_state = 'moving_forward'
        
        # Debug: log occasional position updates
        if hasattr(self, '_last_debug_time'):
            if time.time() - self._last_debug_time > 5.0:  # Log every 5 seconds
                self.get_logger().info(f"Current position: x={msg.pose.pose.position.x:.3f}, y={msg.pose.pose.position.y:.3f}, state={self.movement_state}")
                self._last_debug_time = time.time()
        else:
            self._last_debug_time = time.time()
            
    def scan_callback(self, msg):
        self.last_scan = msg
        # Detect obstacles (objects closer than 0.8m in front sectors)
        if msg.ranges:
            front_ranges = []
            # Check front 60 degrees (±30 degrees from center)
            angle_min = -30 * math.pi / 180
            angle_max = 30 * math.pi / 180
            
            total_angles = len(msg.ranges)
            center_index = total_angles // 2
            angle_range = int(30 * total_angles / 360)  # ±30 degrees
            
            for i in range(center_index - angle_range, center_index + angle_range):
                if 0 <= i < len(msg.ranges) and msg.ranges[i] != float('inf'):
                    front_ranges.append(msg.ranges[i])
            
            if front_ranges:
                min_distance = min(front_ranges)
                if min_distance < 0.8 and self.current_odom:  # Obstacle detected
                    obstacle_pos = {
                        'x': self.current_odom.pose.pose.position.x,
                        'y': self.current_odom.pose.pose.position.y,
                        'distance': min_distance,
                        'side': self.current_side
                    }
                    
                    # Check if this is a new obstacle (not too close to previously detected ones)
                    is_new_obstacle = True
                    for existing in self.results['obstacle_positions']:
                        dx = obstacle_pos['x'] - existing['x']
                        dy = obstacle_pos['y'] - existing['y']
                        if math.sqrt(dx*dx + dy*dy) < 0.3:  # Within 30cm of existing obstacle
                            is_new_obstacle = False
                            break
                    
                    if is_new_obstacle:
                        self.results['obstacle_positions'].append(obstacle_pos)
                        self.results['obstacles_detected'] = len(self.results['obstacle_positions'])
                        self.get_logger().info(f"Obstacle {self.results['obstacles_detected']} detected at distance {min_distance:.2f}m")
    
    def get_current_yaw(self):
        if not self.current_odom:
            return 0.0
        
        orientation = self.current_odom.pose.pose.orientation
        # Convert quaternion to yaw
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def distance_to_start(self):
        if not self.current_odom or not self.start_pose:
            return float('inf')
        
        dx = self.current_odom.pose.pose.position.x - self.start_pose.position.x
        dy = self.current_odom.pose.pose.position.y - self.start_pose.position.y
        return math.sqrt(dx*dx + dy*dy)
    
    def movement_control(self):
        if not self.current_odom:
            return
            
        twist = Twist()
        
        if self.movement_state == 'starting':
            # Wait for start pose to be recorded
            return
            
        elif self.movement_state == 'moving_forward':
            # Move forward for square side
            twist.linear.x = self.LINEAR_SPEED
            
            # Check if we've moved far enough for this side
            if self.current_side == 0:  # First side
                target_x = self.start_pose.position.x + self.SQUARE_SIZE
                current_x = self.current_odom.pose.pose.position.x
                if current_x >= target_x - self.POSITION_TOLERANCE:
                    self.movement_state = 'turning'
                    self.get_logger().info(f"Side {self.current_side + 1} completed, starting turn")
                    
            elif self.current_side == 1:  # Second side  
                target_y = self.start_pose.position.y + self.SQUARE_SIZE
                current_y = self.current_odom.pose.pose.position.y
                if current_y >= target_y - self.POSITION_TOLERANCE:
                    self.movement_state = 'turning'
                    self.get_logger().info(f"Side {self.current_side + 1} completed, starting turn")
                    
            elif self.current_side == 2:  # Third side
                target_x = self.start_pose.position.x
                current_x = self.current_odom.pose.pose.position.x
                if current_x <= target_x + self.POSITION_TOLERANCE:
                    self.movement_state = 'turning'
                    self.get_logger().info(f"Side {self.current_side + 1} completed, starting turn")
                    
            elif self.current_side == 3:  # Fourth side (return to start)
                distance_to_start = self.distance_to_start()
                if distance_to_start <= self.POSITION_TOLERANCE * 2:
                    self.movement_state = 'completed'
                    self.get_logger().info("Square movement completed!")
                    
        elif self.movement_state == 'turning':
            # Turn 90 degrees left
            twist.angular.z = self.ANGULAR_SPEED
            
            # Check if we've turned enough (simplified - turn for fixed time)
            if not hasattr(self, 'turn_start_time'):
                self.turn_start_time = time.time()
            elif time.time() - self.turn_start_time > (math.pi/2) / self.ANGULAR_SPEED:
                # Turn completed
                self.current_side += 1
                self.results['sides_completed'] = self.current_side
                delattr(self, 'turn_start_time')
                
                if self.current_side < 4:
                    self.movement_state = 'moving_forward'
                    self.get_logger().info(f"Turn completed, starting side {self.current_side + 1}")
                else:
                    self.movement_state = 'completed'
                    
        elif self.movement_state == 'completed':
            # Stop the robot
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            
            # Calculate final results
            self.calculate_final_results()
            self.save_results()
            
            # Shutdown after brief pause
            if not hasattr(self, 'completion_time'):
                self.completion_time = time.time()
            elif time.time() - self.completion_time > 2.0:
                self.get_logger().info("Automated square movement test completed. Shutting down.")
                rclpy.shutdown()
                return
        
        # Publish movement command
        self.cmd_vel_pub.publish(twist)
    
    def calculate_final_results(self):
        if self.current_odom and self.start_pose:
            self.results['final_position'] = {
                'x': self.current_odom.pose.pose.position.x,
                'y': self.current_odom.pose.pose.position.y,
                'z': self.current_odom.pose.pose.orientation.z,
                'w': self.current_odom.pose.pose.orientation.w
            }
            
            # Calculate return accuracy
            final_distance = self.distance_to_start()
            self.results['return_accuracy'] = final_distance
            
            # Determine if loop closure was successful
            self.results['loop_closed'] = final_distance <= self.POSITION_TOLERANCE * 2
            
            # Calculate total distance (approximate)
            self.results['total_distance'] = self.SQUARE_SIZE * 4
            
            # Overall success criteria
            self.results['success'] = (
                self.results['sides_completed'] >= 4 and
                self.results['loop_closed'] and
                self.results['obstacles_detected'] >= 2  # Expect at least 2 obstacles in Gazebo
            )
            
            self.results['end_time'] = time.time()
            self.results['duration'] = self.results['end_time'] - self.results['start_time']
            
    def save_results(self):
        # Save results to file for validation
        results_file = '/tmp/automated_square_movement_results.json'
        
        try:
            with open(results_file, 'w') as f:
                json.dump(self.results, f, indent=2)
            
            self.get_logger().info(f"Results saved to {results_file}")
            self.get_logger().info(f"Square completed: {self.results['sides_completed']}/4 sides")
            self.get_logger().info(f"Loop closed: {self.results['loop_closed']} (accuracy: {self.results['return_accuracy']:.3f}m)")
            self.get_logger().info(f"Obstacles detected: {self.results['obstacles_detected']}")
            self.get_logger().info(f"Overall success: {self.results['success']}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to save results: {e}")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = AutomatedSquareMovement()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()