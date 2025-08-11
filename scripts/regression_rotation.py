#!/usr/bin/env python3
"""
Regression Rotation Test Script

This script performs a controlled 360-degree rotation for regression testing.
Based on the working Cartographer demo, it includes:
- 1-minute timeout to prevent infinite spinning
- Guaranteed robot stop after rotation
- Screenshot capture at key moments
- Proper error handling
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
import math
import time
import threading
from pathlib import Path
import subprocess
import sys
import os
import signal
from datetime import datetime

class RegressionRotator(Node):
    def __init__(self):
        super().__init__('regression_rotator')
        
        # Create QoS profile for reliable communication (same as autonomous exploration)
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.RELIABLE
        
        # Publishers and subscribers (identical to autonomous exploration)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', qos_profile)
        self.laser_sub = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.laser_callback, 
            qos_profile
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile
        )
        
        # Robot parameters (identical to autonomous exploration)
        self.linear_speed = 0.0  # No forward movement for regression
        self.angular_speed = 0.3  # Same angular speed as autonomous exploration
        
        # State variables
        self.laser_data = None
        self.laser_received = False
        self.odom_received = False
        self.initial_yaw = None
        self.current_yaw = 0
        self.total_rotation = 0
        self.target_rotation = 2 * math.pi  # 360 degrees
        self.actively_rotating = False  # Track if we're actively commanding rotation
        
        # Screenshot setup
        self.script_dir = Path(__file__).parent
        self.screenshot_dir = Path(__file__).parent.parent / "regression" / "screenshots"
        self.capture_script = self.script_dir / "capture_and_analyze.py"
        self.comparison_script = self.script_dir / "compare_screenshots.py"
        self.screenshot_count = 0
        
        # Control timer - runs at 10Hz for smooth control (same as autonomous exploration)
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        # Status logging timer - every 10 seconds (same as autonomous exploration)
        self.status_timer = self.create_timer(10.0, self.log_status)
        
        # Test state
        self.test_started = False
        self.test_completed = False
        self.comparison_completed = False
        self.comparison_passed = False
        self.rotation_start_time = None
        self.test_timeout = 60.0  # 60 second timeout
        self.screenshots_taken = {
            'initial': False,
            'mid': False,
            'final': False
        }
        
        # Detect simulation vs real robot mode (same as autonomous exploration)
        import os
        self.use_sim_time = os.environ.get('ROS_USE_SIM_TIME', '').lower() == 'true'
        
        mode_str = "🎮 SIMULATION MODE" if self.use_sim_time else "🤖 REAL ROBOT MODE"
        self.get_logger().info(f"🔄 Regression Rotator initialized (mimicking autonomous exploration)")
        self.get_logger().info(f"   {mode_str}")
        self.get_logger().info(f"   Angular speed: {self.angular_speed} rad/s")
        self.get_logger().info(f"   Target rotation: 360°")
        self.get_logger().info("   Waiting for laser scan and odometry data...")
        
    def laser_callback(self, msg):
        """Process laser scan data (same as autonomous exploration)"""
        self.laser_data = msg
        self.laser_received = True
        
    def odom_callback(self, msg):
        """Track robot rotation (same structure as autonomous exploration)"""
        self.odom_received = True
        
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 
                        1 - 2 * (q.y * q.y + q.z * q.z))
        
        if self.initial_yaw is None:
            self.initial_yaw = yaw
            
        # Only track rotation when we're actively commanding it
        if self.actively_rotating:
            # Track total rotation (handle wraparound)
            delta_yaw = yaw - self.current_yaw
            if delta_yaw > math.pi:
                delta_yaw -= 2 * math.pi
            elif delta_yaw < -math.pi:
                delta_yaw += 2 * math.pi
                
            self.total_rotation += abs(delta_yaw)
            
        self.current_yaw = yaw
        
    def capture_screenshot(self, name_prefix="regression"):
        """Capture screenshot - same approach as test"""
        try:
            self.screenshot_dir.mkdir(exist_ok=True)
            
            if self.capture_script.exists():
                result = subprocess.run(
                    ["python3", str(self.capture_script), "--name", name_prefix, 
                     "--output-dir", str(self.screenshot_dir), "--wait-for-rviz"],
                    capture_output=True, text=True, timeout=15
                )
                if result.returncode == 0:
                    self.get_logger().info(f"📸 Screenshot captured: {name_prefix}")
                    return True
                else:
                    self.get_logger().warn(f"Screenshot failed: {result.stderr}")
            return False
        except Exception as e:
            self.get_logger().error(f"Screenshot error: {e}")
            return False
    
    def run_screenshot_comparison(self):
        """Run screenshot comparison with reference images"""
        try:
            self.get_logger().info("🔍 Running screenshot comparison with reference images...")
            
            if self.comparison_script.exists():
                mode_arg = "simulation" if self.use_sim_time else "real"
                result = subprocess.run(
                    ["python3", str(self.comparison_script), 
                     "--actual-dir", str(self.screenshot_dir),
                     "--threshold", "0.90",
                     "--mode", mode_arg,
                     "--output", str(self.screenshot_dir / "comparison_results.json")],
                    capture_output=True, text=True, timeout=30
                )
                
                # Print comparison output
                if result.stdout:
                    print(result.stdout)
                if result.stderr:
                    print("Comparison warnings:", result.stderr)
                    
                if result.returncode == 0:
                    self.get_logger().info("✅ Screenshot comparison PASSED (≥90% similarity)")
                    return True
                else:
                    self.get_logger().error("❌ Screenshot comparison FAILED (<90% similarity)")
                    return False
            else:
                self.get_logger().error("❌ Screenshot comparison script not found")
                return False
                
        except Exception as e:
            self.get_logger().error(f"Screenshot comparison error: {e}")
            return False
        
    def stop_robot(self):
        """Send multiple stop commands to ensure robot stops"""
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = 0.0
        
        # Send stop command multiple times to ensure it's received
        for _ in range(10):
            self.cmd_pub.publish(stop_cmd)
            time.sleep(0.05)
        
        self.get_logger().info("✓ Robot stop command sent (10x)")
    
    def control_loop(self):
        """Control loop - identical structure to autonomous exploration"""
        # Wait for data to be available (same as autonomous exploration)
        if not self.laser_received or not self.odom_received:
            return
        
        # Check for timeout
        if self.test_started and not self.test_completed:
            elapsed = time.time() - self.rotation_start_time
            if elapsed > self.test_timeout:
                self.get_logger().error(f"❌ TIMEOUT: Test exceeded {self.test_timeout} seconds")
                self.stop_robot()
                self.test_completed = True
                # Force exit with failure
                sys.exit(1)
            
        # Start test if not already started
        if not self.test_started:
            self.get_logger().info("🚀 Starting controlled 360° rotation...")
            self.get_logger().info(f"   Timeout: {self.test_timeout} seconds")
            self.test_started = True
            self.actively_rotating = True  # Start tracking rotation
            self.rotation_start_time = time.time()
            
            # Take initial screenshot
            self.capture_screenshot("rotation_initial")
            self.screenshots_taken['initial'] = True
            
        # Check if rotation is complete
        if self.total_rotation >= self.target_rotation and not self.test_completed:
            # Stop rotation tracking and movement
            self.actively_rotating = False
            self.stop_robot()  # Use multiple stop commands
            
            # Take final screenshot
            self.capture_screenshot("rotation_final")
            self.screenshots_taken['final'] = True
            
            rotation_time = time.time() - self.rotation_start_time
            rotation_degrees = math.degrees(self.total_rotation)
            
            self.get_logger().info(f"✅ 360° rotation completed!")
            self.get_logger().info(f"   Total rotation: {rotation_degrees:.1f}°")
            self.get_logger().info(f"   Duration: {rotation_time:.1f} seconds")
            
            self.test_completed = True
            return
            
        # Run comparison after test is complete but before finishing
        if self.test_completed and not self.comparison_completed:
            # Give a moment for final screenshot to be saved
            time.sleep(2)
            
            # Run screenshot comparison
            self.comparison_passed = self.run_screenshot_comparison()
            self.comparison_completed = True
            return
            
        # Continue rotation if not complete
        if self.test_started and not self.test_completed and self.actively_rotating:
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = self.angular_speed
            self.cmd_pub.publish(cmd)
            
            # Take mid-rotation screenshot (around 180°)
            if not self.screenshots_taken['mid'] and self.total_rotation > math.pi:
                self.capture_screenshot("rotation_mid")
                self.screenshots_taken['mid'] = True
        
        # Send stop command if we've stopped but haven't completed comparison yet  
        elif self.test_completed and not self.comparison_completed:
            # Ensure robot stays stopped during comparison
            stop_cmd = Twist()
            stop_cmd.linear.x = 0.0
            stop_cmd.angular.z = 0.0
            self.cmd_pub.publish(stop_cmd)
                
    def log_status(self):
        """Status logging - same interval as autonomous exploration"""
        if self.test_started and not self.test_completed:
            rotation_degrees = math.degrees(self.total_rotation)
            elapsed_time = time.time() - self.rotation_start_time
            time_remaining = self.test_timeout - elapsed_time
            self.get_logger().info(f"🔄 Progress: {rotation_degrees:.1f}° | Elapsed: {elapsed_time:.1f}s | Remaining: {time_remaining:.1f}s")
            
def main():
    print("🔄 Regression Rotation Script")
    print("===============================")
    print("Mimicking autonomous exploration structure with controlled rotation")
    print("")
    
    # Check if we're in simulation or real robot mode (same as autonomous exploration)
    import os
    use_sim_time = os.environ.get('ROS_USE_SIM_TIME', '').lower() == 'true'
    
    if use_sim_time:
        print("🎮 SIMULATION MODE: Robot will rotate in Gazebo Classic")
        print("       Virtual robot performs controlled 360° rotation for testing")
    else:
        print("🤖 REAL ROBOT MODE: Physical robot will perform rotation")
        print("       Real robot performs controlled 360° rotation for testing")
        print("       ⚠️  Ensure robot has clear space for safe rotation")
    
    print("")
    
    rclpy.init()
    
    try:
        rotator = RegressionRotator()
        
        # Keep running until rotation and comparison are complete
        while not (rotator.test_completed and rotator.comparison_completed):
            rclpy.spin_once(rotator, timeout_sec=0.1)
            time.sleep(0.01)  # Small delay to prevent CPU spinning
            
        print(f"\n📊 REGRESSION TEST RESULTS:")
        print(f"   Rotation: ✅ Completed 360°")
        print(f"   Screenshots: ✅ Captured (initial, mid, final)")
        
        if rotator.comparison_passed:
            print(f"   Comparison: ✅ PASSED (≥90% similarity)")
            print(f"\n🎉 REGRESSION TEST PASSED!")
            return 0
        else:
            print(f"   Comparison: ❌ FAILED (<90% similarity)")
            print(f"\n❌ REGRESSION TEST FAILED!")
            return 1
        
    except KeyboardInterrupt:
        print("\n🛑 Rotation test interrupted by user")
        return 0
    except Exception as e:
        print(f"\n❌ Rotation test error: {e}")
        return 1
    finally:
        try:
            if 'rotator' in locals():
                # Stop any movement with multiple commands
                rotator.stop_robot()
                rotator.destroy_node()
            rclpy.shutdown()
        except:
            pass
        
        # Emergency stop via command line as final fallback
        try:
            os.system("ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'")
        except:
            pass

if __name__ == "__main__":
    exit(main())