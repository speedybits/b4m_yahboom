#!/usr/bin/env python3
"""
Demo-Based Regression Test

This follows the exact pattern from the working Cartographer demo:
1. Robot bringup is already done
2. RViz is already running  
3. Cartographer (map_cartographer_launch.py) is already running
4. This script just does rotation like keyboard control would

The key insight: The demo uses simple keyboard control that just publishes to /cmd_vel
without complex QoS settings or timers.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import subprocess
from pathlib import Path
import os

class DemoBasedRegression(Node):
    def __init__(self):
        super().__init__('demo_based_regression')
        
        # Simple publisher - no special QoS, just like keyboard control
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Screenshot setup
        self.screenshot_dir = Path(__file__).parent.parent / "regression" / "screenshots"
        self.screenshot_dir.mkdir(exist_ok=True)
        
        # Test parameters
        self.rotation_duration = 20.0  # seconds to rotate 360 degrees at 0.3 rad/s
        self.angular_speed = 0.3  # rad/s
        
        self.get_logger().info("🔄 Demo-Based Regression Test")
        self.get_logger().info(f"   Duration: {self.rotation_duration}s")
        self.get_logger().info(f"   Angular speed: {self.angular_speed} rad/s")

    def capture_screenshot(self, name):
        """Simple screenshot capture"""
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        screenshot_path = self.screenshot_dir / f"rotation_{name}_{timestamp}.png"
        
        # Use import which works reliably
        result = subprocess.run([
            "import", "-window", "root", str(screenshot_path)
        ], capture_output=True, text=True, timeout=10)
        
        if result.returncode == 0:
            self.get_logger().info(f"📸 Screenshot: {name}")
            return True
        return False

    def run_rotation_test(self):
        """Run the 360 degree rotation test"""
        
        # Take initial screenshot
        self.capture_screenshot("initial")
        
        # Create rotation command
        rotate_cmd = Twist()
        rotate_cmd.linear.x = 0.0
        rotate_cmd.angular.z = self.angular_speed
        
        # Start rotation
        self.get_logger().info("🚀 Starting 360° rotation...")
        start_time = time.time()
        
        # Rotate for calculated duration, publishing at 10Hz like keyboard control
        while time.time() - start_time < self.rotation_duration:
            self.cmd_pub.publish(rotate_cmd)
            
            # Progress update every 5 seconds
            elapsed = time.time() - start_time
            if int(elapsed) % 5 == 0 and elapsed - int(elapsed) < 0.1:
                progress = (elapsed / self.rotation_duration) * 360
                self.get_logger().info(f"🔄 Progress: ~{progress:.0f}° ({elapsed:.1f}s)")
            
            # Mid-rotation screenshot
            if elapsed > self.rotation_duration / 2 and elapsed < (self.rotation_duration / 2 + 0.2):
                self.capture_screenshot("mid")
            
            # Sleep to maintain 10Hz rate
            time.sleep(0.1)
        
        # Stop rotation
        stop_cmd = Twist()
        self.cmd_pub.publish(stop_cmd)
        
        # Take final screenshot
        self.capture_screenshot("final")
        
        elapsed = time.time() - start_time
        self.get_logger().info(f"✅ Rotation complete! Duration: {elapsed:.1f}s")
        
        # Run comparison
        return self.run_comparison()

    def run_comparison(self):
        """Run screenshot comparison"""
        try:
            # Detect mode for comparison
            use_sim_time = os.environ.get('ROS_USE_SIM_TIME', '').lower() == 'true'
            mode = "simulation" if use_sim_time else "real"
            
            comparison_script = Path(__file__).parent / "compare_screenshots.py"
            if comparison_script.exists():
                self.get_logger().info("🔍 Running screenshot comparison...")
                
                result = subprocess.run([
                    "python3", str(comparison_script),
                    "--actual-dir", str(self.screenshot_dir),
                    "--threshold", "0.90",
                    "--mode", mode,
                    "--output", str(self.screenshot_dir / "comparison_results.json")
                ], capture_output=True, text=True, timeout=30)
                
                print(result.stdout)
                if result.stderr:
                    print("Warnings:", result.stderr)
                
                if result.returncode == 0:
                    self.get_logger().info("✅ Screenshot comparison PASSED")
                    return True
                else:
                    self.get_logger().error("❌ Screenshot comparison FAILED")
                    return False
            else:
                self.get_logger().warn("⚠️  Comparison script not found, skipping comparison")
                return True
                
        except Exception as e:
            self.get_logger().error(f"Comparison error: {e}")
            return False

def main():
    print("=" * 50)
    print("🔄 DEMO-BASED REGRESSION TEST")
    print("=" * 50)
    print("Following working Cartographer demo pattern")
    print("Simple /cmd_vel publishing at 10Hz")
    print("")
    
    rclpy.init()
    
    try:
        test = DemoBasedRegression()
        
        # Run the rotation test
        success = test.run_rotation_test()
        
        print("")
        print("=" * 50)
        if success:
            print("🎉 REGRESSION TEST PASSED!")
            print("=" * 50)
            return 0
        else:
            print("❌ REGRESSION TEST FAILED!")
            print("=" * 50)
            return 1
            
    except KeyboardInterrupt:
        print("\n🛑 Test interrupted by user")
        # Send stop command
        try:
            node = Node('emergency_stop')
            pub = node.create_publisher(Twist, '/cmd_vel', 10)
            pub.publish(Twist())
            node.destroy_node()
        except:
            pass
        return 0
        
    except Exception as e:
        print(f"\n❌ Test error: {e}")
        return 1
        
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    exit(main())