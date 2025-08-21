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

class B4MSpatialInterpreter(Node):
    def __init__(self):
        super().__init__('b4m_spatial_interpreter')
        
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
        self.state = "moving_forward"  # States: moving_forward, stopped_waiting, turning
        self.turn_direction = 0  # 1 for left, -1 for right
        self.laser_data = None
        self.obstacle_detected = False
        self.path_clear = True
        self.user_decision_pending = False
        
        # Turn tracking
        self.turn_start_time = 0
        self.min_turn_time = 1.0  # Minimum turn duration
        self.clear_readings_count = 0
        self.required_clear_readings = 3  # Need 3 consecutive clear readings
        
        # Control timer - runs at 10Hz for smooth control
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        # Initial startup message (only output during startup)
        self.get_logger().info("🔌 B4M Spatial Interpreter initialized")
        self.get_logger().info("   Robot will move forward until obstacle detected")
        self.get_logger().info("   Console will stay quiet during normal operation")
        print("\n✅ B4M SPATIAL INTERPRETER ACTIVE")
        print("=" * 63)
        print("🔌 Robot starting with B4M Spatial Interpreter\n")
        # After this, stay quiet until obstacle detected
        
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
        
        # Front sector (±15 degrees)
        front_angle_rad = math.radians(15)
        front_center_idx = int((0.0 - msg.angle_min) / angle_increment)
        front_half_width = int(front_angle_rad / angle_increment)
        front_start = max(0, front_center_idx - front_half_width)
        front_end = min(num_readings - 1, front_center_idx + front_half_width)
        front_ranges = ranges[front_start:front_end + 1]
        min_front_distance = np.min(front_ranges) if len(front_ranges) > 0 else msg.range_max
        
        # Left sector (45° to 90°)
        left_angle_start = math.radians(45)
        left_angle_end = math.radians(90)
        left_start_idx = max(0, int((left_angle_start - msg.angle_min) / angle_increment))
        left_end_idx = min(num_readings - 1, int((left_angle_end - msg.angle_min) / angle_increment))
        left_ranges = ranges[left_start_idx:left_end_idx + 1] if left_end_idx >= left_start_idx else []
        min_left_distance = np.min(left_ranges) if len(left_ranges) > 0 else msg.range_max
        
        # Right sector (-90° to -45°)
        right_angle_start = math.radians(-90)
        right_angle_end = math.radians(-45)
        right_start_idx = max(0, int((right_angle_start - msg.angle_min) / angle_increment))
        right_end_idx = min(num_readings - 1, int((right_angle_end - msg.angle_min) / angle_increment))
        right_ranges = ranges[right_start_idx:right_end_idx + 1] if right_end_idx >= right_start_idx else []
        min_right_distance = np.min(right_ranges) if len(right_ranges) > 0 else msg.range_max
        
        # Stop at ANY obstacle within stop distance (30cm)
        self.obstacle_detected = (
            min_front_distance <= self.stop_distance or
            min_left_distance <= self.stop_distance or
            min_right_distance <= self.stop_distance
        )
        
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
        
        # Front sector (±15°)
        sectors['front'] = self.analyze_sector(ranges, -15, 15, self.laser_data.angle_min, angle_increment)
        
        # Left sector (45° to 135°)
        sectors['left'] = self.analyze_sector(ranges, 45, 135, self.laser_data.angle_min, angle_increment)
        
        # Right sector (-135° to -45°)
        sectors['right'] = self.analyze_sector(ranges, -135, -45, self.laser_data.angle_min, angle_increment)
        
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
        print("\n📍 Current Situation:")
        print("-" * 63)
        print(f"FRONT:  {spatial_context['front']['description']}")
        print(f"LEFT:   {spatial_context['left']['description']}")
        print(f"RIGHT:  {spatial_context['right']['description']}")
        print(f"BEHIND: {spatial_context['behind']['description']}")
        
        print("\n📊 Detailed Scan Analysis:")
        print("-" * 63)
        print(f"• Front sector (±15°):  Min: {spatial_context['front']['min']:.2f}m, Avg: {spatial_context['front']['avg']:.2f}m")
        print(f"• Left sector (±45°):   Min: {spatial_context['left']['min']:.2f}m, Avg: {spatial_context['left']['avg']:.2f}m")
        print(f"• Right sector (±45°):  Min: {spatial_context['right']['min']:.2f}m, Avg: {spatial_context['right']['avg']:.2f}m")
        print(f"• Laser points: {len(self.laser_data.ranges)} readings covering 360°")
        
        print("\n🎯 Navigation Options:")
        print("-" * 63)
        print("1) Turn LEFT 90°")
        print("2) Turn RIGHT 90°")
        print("3) Turn AROUND 180°")
        print()
        
    def get_user_decision(self):
        """Get navigation decision from user via blocking console input"""
        while True:
            try:
                choice = input("Please select action (1-3): ")
                if choice in ['1', '2', '3']:
                    return int(choice)
                print("Invalid input. Please enter 1, 2, or 3.")
            except KeyboardInterrupt:
                print("\n\n🛑 User requested stop - shutting down...")
                return None
            except EOFError:
                # Handle case where input stream is closed
                print("\n\n⚠️ Input stream closed - shutting down...")
                return None
    
    def execute_turn(self, turn_choice):
        """Execute the selected turn maneuver"""
        if turn_choice == 1:  # Left 90°
            self.turn_direction = 1
            turn_desc = "LEFT"
        elif turn_choice == 2:  # Right 90°
            self.turn_direction = -1
            turn_desc = "RIGHT"
        else:  # Turn around 180°
            self.turn_direction = 1  # Default to left for 180°
            turn_desc = "AROUND"
            
        print(f"\n🔄 Executing turn {turn_desc}...")
        print("   (Will continue turning until path ahead is clear)")
        
        self.state = "turning"
        self.turn_start_time = time.time()
        self.clear_readings_count = 0
    
    def control_loop(self):
        """Main control loop for spatial interpreter"""
        if self.laser_data is None:
            return
            
        cmd = Twist()
        
        if self.state == "moving_forward":
            if self.obstacle_detected and not self.user_decision_pending:
                # Stop and wait for user input
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.cmd_pub.publish(cmd)
                
                # Analyze and display spatial context
                spatial_context = self.analyze_spatial_context()
                if spatial_context:
                    self.display_spatial_description(spatial_context)
                    
                    # Set flag to prevent re-displaying
                    self.user_decision_pending = True
                    self.state = "stopped_waiting"
                    
                    # Get user decision (blocking)
                    decision = self.get_user_decision()
                    if decision is None:
                        # User wants to exit
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
                    
                    print(f"\n✅ Clear path detected ahead ({front_distance:.2f}m)\n")
                
                # Resume forward movement (stay quiet)
                self.state = "moving_forward"
                self.obstacle_detected = False  # Reset for next obstacle
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
        
        # Publish velocity command
        self.cmd_pub.publish(cmd)

def main():
    """Main entry point for B4M Spatial Interpreter"""
    rclpy.init()
    
    try:
        interpreter = B4MSpatialInterpreter()
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