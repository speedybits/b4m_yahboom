#!/usr/bin/env python3
"""
Autonomous Exploration Script for B4M Robot

This script enables the robot to autonomously explore its environment while avoiding obstacles.
It works in both Gazebo Classic simulation and with the real robot.

Features:
- Slow movement for safe exploration
- Laser-based obstacle avoidance
- Integration with SLAM for real-time mapping
- Works in both simulation and real robot modes
- Continuous exploration until manually stopped

Author: Generated for B4M Robot System
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import math
import time
import random

class AutonomousExplorer(Node):
    def __init__(self):
        super().__init__('autonomous_explorer')
        
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
        self.linear_speed = 0.08  # Very slow for safe exploration (0.08 m/s)
        self.angular_speed = 0.3  # Moderate turning speed
        self.stop_distance = 0.3048  # Stop distance: 1 foot (30.48cm)
        self.safe_distance = 0.4  # Resume forward movement when path is clear (40cm)
        
        # State variables
        self.laser_data = None
        self.obstacle_close = False  # Obstacle within stop distance
        self.path_clear = True      # Path clear enough to move forward
        self.turn_direction = 1     # 1 for left, -1 for right
        self.exploration_state = "forward"  # "forward", "turning_in_place"
        
        # Turn timing variables
        self.turn_start_time = 0
        self.min_turn_time = 1.0    # Minimum time to turn (1 second)
        self.clear_readings_count = 0
        self.required_clear_readings = 3  # Require 3 consecutive clear readings
        
        # Exploration parameters
        self.forward_time = 0
        self.max_forward_time = random.uniform(8, 15)  # Random exploration duration
        self.turn_time = 0
        self.max_turn_time = 0
        
        # Control timer - runs at 10Hz for smooth control
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        # Status logging timer - every 10 seconds
        self.status_timer = self.create_timer(10.0, self.log_status)
        
        self.get_logger().info("🗺️  Autonomous Explorer initialized")
        self.get_logger().info(f"   Linear speed: {self.linear_speed} m/s")
        self.get_logger().info(f"   Angular speed: {self.angular_speed} rad/s") 
        self.get_logger().info(f"   Stop distance: {self.stop_distance} m (1 foot)")
        self.get_logger().info(f"   Safe distance: {self.safe_distance} m")
        self.get_logger().info("   Waiting for laser scan data...")
    
    def laser_callback(self, msg):
        """Process laser scan data for obstacle detection"""
        self.laser_data = msg
        
        # Analyze laser data for obstacles
        ranges = np.array(msg.ranges)
        
        # Handle infinite values
        ranges[np.isinf(ranges)] = msg.range_max
        ranges[np.isnan(ranges)] = msg.range_max
        
        # Calculate actual laser parameters dynamically
        num_readings = len(ranges)
        angle_range = msg.angle_max - msg.angle_min  # Total angle coverage
        angle_increment_actual = angle_range / (num_readings - 1) if num_readings > 1 else msg.angle_increment
        
        # Find the index corresponding to 0 degrees (front of robot)
        front_center_index = int((0.0 - msg.angle_min) / angle_increment_actual)
        
        # Check forward sector (±5 degrees from center)
        front_angle_rad = math.radians(5)  # 5 degrees on each side
        front_half_width = int(front_angle_rad / angle_increment_actual)
        
        # Ensure indices are within bounds
        front_start = max(0, front_center_index - front_half_width)
        front_end = min(num_readings - 1, front_center_index + front_half_width)
        
        front_ranges = ranges[front_start:front_end + 1]
        
        # Check for obstacles in front
        min_front_distance = np.min(front_ranges)
        
        # Check side obstacles (±30 to ±60 degrees) to prevent clipping walls
        left_angle_start = math.radians(-60)   # -60°
        left_angle_end = math.radians(-30)     # -30°
        right_angle_start = math.radians(30)   # +30°  
        right_angle_end = math.radians(60)     # +60°
        
        left_start_idx = max(0, int((left_angle_start - msg.angle_min) / angle_increment_actual))
        left_end_idx = min(num_readings - 1, int((left_angle_end - msg.angle_min) / angle_increment_actual))
        right_start_idx = max(0, int((right_angle_start - msg.angle_min) / angle_increment_actual))  
        right_end_idx = min(num_readings - 1, int((right_angle_end - msg.angle_min) / angle_increment_actual))
        
        left_side_ranges = ranges[left_start_idx:left_end_idx + 1] if left_end_idx >= left_start_idx else []
        right_side_ranges = ranges[right_start_idx:right_end_idx + 1] if right_end_idx >= right_start_idx else []
        
        min_side_distance = msg.range_max  # Default to max range
        if len(left_side_ranges) > 0:
            min_side_distance = min(min_side_distance, np.min(left_side_ranges))
        if len(right_side_ranges) > 0:
            min_side_distance = min(min_side_distance, np.min(right_side_ranges))
        
        # Enhanced obstacle detection: stop if EITHER front or sides are too close
        close_side_threshold = 0.25  # Stop if sides are within 25cm (10 inches)
        side_obstacle_close = min_side_distance <= close_side_threshold
        
        self.obstacle_close = (min_front_distance <= self.stop_distance) or side_obstacle_close
        self.path_clear = (min_front_distance > self.safe_distance) and (min_side_distance > 0.35)
        
        # Log detailed debug info more frequently when near obstacles
        if hasattr(self, '_last_debug_time'):
            debug_interval = 2.0 if (min_front_distance < 1.0 or min_side_distance < 0.5) else 5.0  # More frequent when close
            if time.time() - self._last_debug_time > debug_interval:
                self._log_debug_info(min_front_distance, front_ranges, min_side_distance, side_obstacle_close)
        else:
            self._last_debug_time = time.time()
    
    def _log_debug_info(self, min_front_distance, front_ranges, min_side_distance, side_obstacle_close):
        """Log debug information about sensor readings"""
        self._last_debug_time = time.time()
        self.get_logger().info(f"🔍 LASER DEBUG: Front: {min_front_distance:.2f}m, Sides: {min_side_distance:.2f}m")
        self.get_logger().info(f"   Front sector (10°): {[f'{r:.2f}' for r in front_ranges]}")
        self.get_logger().info(f"   State: {self.exploration_state}, Stop: {self.obstacle_close}, Clear: {self.path_clear}")
        if min_front_distance <= self.stop_distance:
            self.get_logger().warn(f"   ⚠️  FRONT STOP: {min_front_distance:.2f}m ≤ {self.stop_distance:.2f}m")
        if side_obstacle_close:
            self.get_logger().warn(f"   ⚠️  SIDE STOP: {min_side_distance:.2f}m ≤ 0.25m")
    
    def control_loop(self):
        """Main control loop for autonomous exploration"""
        if self.laser_data is None:
            return
        
        cmd = Twist()
        
        # State machine for exploration behavior
        if self.exploration_state == "forward":
            self._handle_forward_state(cmd)
        elif self.exploration_state == "turning_in_place":
            self._handle_turning_in_place_state(cmd)
        
        # Publish velocity command
        self.cmd_pub.publish(cmd)
    
    def _handle_forward_state(self, cmd):
        """Handle forward movement state"""
        # Check if obstacle is too close (front or sides) - STOP immediately
        if self.obstacle_close:
            # Stop all movement and start turning in place
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.exploration_state = "turning_in_place"
            self.turn_direction = random.choice([-1, 1])  # Random turn direction
            self.turn_start_time = time.time()  # Record when turning started
            self.clear_readings_count = 0  # Reset clear readings counter
            self.get_logger().info(f"🛑 Obstacle too close - stopped! Turning in place {'left' if self.turn_direction > 0 else 'right'}")
            return
        
        # Path is clear - move forward
        cmd.linear.x = self.linear_speed
        cmd.angular.z = 0.0
        
        self.forward_time += 0.1
        
        # Occasionally change direction for better exploration coverage
        if self.forward_time > self.max_forward_time:
            self.exploration_state = "turning_in_place"
            self.turn_direction = random.choice([-1, 1])  # Random direction
            self.turn_start_time = time.time()  # Record when turning started
            self.clear_readings_count = 0  # Reset clear readings counter
            self.forward_time = 0
            self.max_forward_time = random.uniform(8, 15)
            self.get_logger().info(f"🔄 Random direction change - turning {'left' if self.turn_direction > 0 else 'right'}")
    
    def _handle_turning_in_place_state(self, cmd):
        """Handle turning in place until path is clear with minimum turn duration"""
        # Always stop forward movement while turning
        cmd.linear.x = 0.0
        cmd.angular.z = self.angular_speed * self.turn_direction
        
        # Calculate how long we've been turning
        current_time = time.time()
        turn_duration = current_time - self.turn_start_time
        
        # Check if minimum turn time has elapsed
        min_turn_elapsed = turn_duration >= self.min_turn_time
        
        # Calculate front-only clearance for stuck situations
        ranges = np.array(self.laser_data.ranges)
        ranges[np.isinf(ranges)] = self.laser_data.range_max
        ranges[np.isnan(ranges)] = self.laser_data.range_max
        
        num_readings = len(ranges)
        angle_range = self.laser_data.angle_max - self.laser_data.angle_min
        angle_increment_actual = angle_range / (num_readings - 1) if num_readings > 1 else self.laser_data.angle_increment
        front_center_index = int((0.0 - self.laser_data.angle_min) / angle_increment_actual)
        front_angle_rad = math.radians(5)  # ±5 degrees
        front_half_width = int(front_angle_rad / angle_increment_actual)
        front_start = max(0, front_center_index - front_half_width)
        front_end = min(num_readings - 1, front_center_index + front_half_width)
        front_ranges = ranges[front_start:front_end + 1]
        min_front_distance = np.min(front_ranges)
        
        # Front-only clearance check (ignore sides when stuck)
        front_only_clear = min_front_distance > self.safe_distance
        
        # If turning for too long (>15 seconds), use front-only clearance
        if turn_duration > 15.0:
            # Emergency escape: only require front clearance if turning too long
            if front_only_clear:
                self.clear_readings_count += 1
            else:
                self.clear_readings_count = 0
            
            path_check = front_only_clear
            escape_mode = True
        else:
            # Normal operation: require both front and side clearance
            if self.path_clear:
                self.clear_readings_count += 1
            else:
                self.clear_readings_count = 0  # Reset if path is not clear
            
            path_check = self.path_clear
            escape_mode = False
        
        # Check if we have enough consecutive clear readings
        enough_clear_readings = self.clear_readings_count >= self.required_clear_readings
        
        # Only resume forward movement if BOTH conditions are met
        if min_turn_elapsed and enough_clear_readings:
            # Both minimum turn time elapsed AND sufficient clear readings
            self.exploration_state = "forward"
            if escape_mode:
                self.get_logger().info(f"🚨 ESCAPE MODE: Front clear after {turn_duration:.1f}s turn (front: {min_front_distance:.2f}m) - resuming forward")
            else:
                self.get_logger().info(f"✅ Path clear after {turn_duration:.1f}s turn with {self.clear_readings_count} clear readings - resuming forward")
            return
        
        # Log status while turning (every 2 seconds)
        if hasattr(self, '_last_turn_log_time'):
            if current_time - self._last_turn_log_time > 2.0:  # Every 2 seconds
                self._last_turn_log_time = current_time
                ranges = np.array(self.laser_data.ranges)
                ranges[np.isinf(ranges)] = self.laser_data.range_max
                ranges[np.isnan(ranges)] = self.laser_data.range_max
                # Calculate front sector for debug (same method as main callback)
                num_readings = len(ranges)
                angle_range = self.laser_data.angle_max - self.laser_data.angle_min
                angle_increment_actual = angle_range / (num_readings - 1) if num_readings > 1 else self.laser_data.angle_increment
                
                front_center_index = int((0.0 - self.laser_data.angle_min) / angle_increment_actual)
                front_angle_rad = math.radians(5)
                front_half_width = int(front_angle_rad / angle_increment_actual)
                
                front_start = max(0, front_center_index - front_half_width)  
                front_end = min(num_readings - 1, front_center_index + front_half_width)
                front_ranges = ranges[front_start:front_end + 1]
                min_distance = np.min(front_ranges)
                
                # Show detailed turn status
                status_msg = f"🔄 Turning {turn_duration:.1f}s"
                if not min_turn_elapsed:
                    status_msg += f" (need {self.min_turn_time:.1f}s min)"
                if not enough_clear_readings:
                    status_msg += f" (clear: {self.clear_readings_count}/{self.required_clear_readings})"
                
                # Add escape mode indicator
                if turn_duration > 15.0:
                    status_msg += " [ESCAPE MODE: front-only]"
                
                status_msg += f" - front: {min_distance:.2f}m"
                
                self.get_logger().info(status_msg)
        else:
            self._last_turn_log_time = current_time
    
    
    def log_status(self):
        """Log exploration status periodically"""
        if self.laser_data is not None:
            ranges = np.array(self.laser_data.ranges)
            ranges[np.isinf(ranges)] = self.laser_data.range_max
            ranges[np.isnan(ranges)] = self.laser_data.range_max
            
            # Calculate front sector distance (same as used for decisions)
            num_readings = len(ranges)
            angle_range = self.laser_data.angle_max - self.laser_data.angle_min
            angle_increment_actual = angle_range / (num_readings - 1) if num_readings > 1 else self.laser_data.angle_increment
            
            front_center_index = int((0.0 - self.laser_data.angle_min) / angle_increment_actual)
            front_angle_rad = math.radians(5)  # ±5 degrees
            front_half_width = int(front_angle_rad / angle_increment_actual)
            
            front_start = max(0, front_center_index - front_half_width)
            front_end = min(num_readings - 1, front_center_index + front_half_width)
            front_ranges = ranges[front_start:front_end + 1]
            min_front_distance = np.min(front_ranges)
            
            # Also get overall closest for context
            min_overall_distance = np.min(ranges)
            
            self.get_logger().info("🗺️  Exploration Status:")
            self.get_logger().info(f"   State: {self.exploration_state}")
            self.get_logger().info(f"   Front path distance: {min_front_distance:.2f}m")
            self.get_logger().info(f"   Closest obstacle (any direction): {min_overall_distance:.2f}m") 
            self.get_logger().info(f"   Obstacle close (≤1ft): {self.obstacle_close}")
            self.get_logger().info(f"   Path clear (>40cm): {self.path_clear}")
            self.get_logger().info(f"   Laser readings: {len(ranges)} points")
        else:
            self.get_logger().warn("   No laser data received")

def main():
    print("🗺️  B4M Autonomous Exploration")
    print("=============================")
    
    # Check if we're in simulation or real robot mode based on ROS parameters
    import os
    use_sim_time = os.environ.get('ROS_USE_SIM_TIME', '').lower() == 'true'
    
    if use_sim_time:
        print("🎮 SIMULATION MODE: Robot will explore in Gazebo Classic")
        print("       Virtual robot will move around the simulated environment")
    else:
        print("🤖 REAL ROBOT MODE: Physical robot will explore real environment")
        print("       ⚠️  ENSURE PHYSICAL SPACE IS CLEAR AND SAFE FOR ROBOT MOVEMENT")
        print("       ⚠️  Robot will move slowly but ALWAYS supervise operation")
        
    print()
    print("Starting autonomous exploration with 1-foot stop distance")
    print("Robot will stop when obstacles are ≤1 foot away, turn randomly until clear")
    print("Press Ctrl+C to stop exploration")
    print()
    
    rclpy.init()
    
    try:
        explorer = AutonomousExplorer()
        
        print("✅ Autonomous explorer initialized")
        print("🚀 Starting exploration loop...")
        print("   Use RViz to monitor the robot and SLAM mapping")
        print("   Robot stops at 1 foot, turns in place until path is clear")
        print()
        
        rclpy.spin(explorer)
        
    except KeyboardInterrupt:
        print("\n🛑 Exploration stopped by user")
        
        # Send stop command before shutting down
        if 'explorer' in locals():
            stop_cmd = Twist()  # All zeros
            explorer.cmd_pub.publish(stop_cmd)
            time.sleep(0.1)  # Give time for command to be sent
        
    except Exception as e:
        print(f"❌ Exploration failed: {e}")
        return 1
        
    finally:
        if 'explorer' in locals():
            # Send final stop command
            stop_cmd = Twist()
            explorer.cmd_pub.publish(stop_cmd)
        
        try:
            rclpy.shutdown()
        except:
            pass
    
    print("🎯 Autonomous exploration completed")
    return 0

if __name__ == '__main__':
    exit(main())