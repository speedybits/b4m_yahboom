#!/usr/bin/env python3
"""
Enhanced Regression Test with Odometry Quality Monitoring

This script runs the standard regression rotation test while simultaneously
monitoring odometry quality. It provides comprehensive testing of both
visual regression and navigation system health.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import subprocess
from pathlib import Path
import os
import threading
import json

# Import our custom modules
import sys
sys.path.append(str(Path(__file__).parent))
from odometry_test import OdometryQualityTest

class RegressionWithOdometry(Node):
    def __init__(self):
        super().__init__('regression_with_odometry')
        
        # Publisher for robot movement
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Test parameters
        self.rotation_duration = 20.0
        self.angular_speed = 0.3
        
        # Screenshot and logging setup
        self.script_dir = Path(__file__).parent
        self.screenshot_dir = Path(__file__).parent.parent / "regression" / "screenshots"
        self.capture_script = self.script_dir / "capture_and_analyze.py"
        self.comparison_script = self.script_dir / "compare_screenshots.py"
        
        # Test state
        self.odometry_test = None
        self.test_results = {}
        
        # Detect simulation vs real robot
        self.use_sim_time = os.environ.get('ROS_USE_SIM_TIME', '').lower() == 'true'
        mode_str = "🎮 SIMULATION MODE" if self.use_sim_time else "🤖 REAL ROBOT MODE"
        
        self.get_logger().info("🔄 Enhanced Regression Test with Odometry Monitoring")
        self.get_logger().info(f"   {mode_str}")
        self.get_logger().info(f"   Duration: {self.rotation_duration}s")
        self.get_logger().info(f"   Angular speed: {self.angular_speed} rad/s")

    def capture_screenshot(self, name):
        """Capture screenshot using the proven method"""
        try:
            self.screenshot_dir.mkdir(exist_ok=True)
            
            if self.capture_script.exists():
                result = subprocess.run(
                    ["python3", str(self.capture_script), "--name", f"rotation_{name}", 
                     "--output-dir", str(self.screenshot_dir), "--wait-for-rviz"],
                    capture_output=True, text=True, timeout=15
                )
                if result.returncode == 0:
                    self.get_logger().info(f"📸 Screenshot captured: {name}")
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
            self.get_logger().info("🔍 Running screenshot comparison...")
            
            if self.comparison_script.exists():
                mode_arg = "simulation" if self.use_sim_time else "real"
                result = subprocess.run(
                    ["python3", str(self.comparison_script), 
                     "--actual-dir", str(self.screenshot_dir),
                     "--mode", mode_arg,
                     "--output", str(self.screenshot_dir / "comparison_results.json")],
                    capture_output=True, text=True, timeout=30
                )
                
                print(result.stdout)
                if result.stderr:
                    print("Comparison warnings:", result.stderr)
                    
                return result.returncode == 0
            else:
                self.get_logger().error("❌ Screenshot comparison script not found")
                return False
                
        except Exception as e:
            self.get_logger().error(f"Screenshot comparison error: {e}")
            return False

    def run_comprehensive_test(self):
        """Run both rotation and odometry tests simultaneously"""
        self.get_logger().info("🚀 Starting comprehensive regression test...")
        
        # Initialize odometry test
        self.odometry_test = OdometryQualityTest()
        
        # Start odometry monitoring
        self.odometry_test.start_test()
        
        # Phase 1: Stationary baseline (5 seconds)
        self.get_logger().info("📊 Phase 1: Stationary baseline measurement")
        self.odometry_test.start_test_phase("stationary_baseline", 5)
        time.sleep(5)
        self.odometry_test.end_test_phase("stationary_baseline")
        
        # Take initial screenshot
        screenshot_success = self.capture_screenshot("initial")
        
        # Phase 2: Rotation test (20 seconds) with odometry monitoring
        self.get_logger().info("📊 Phase 2: Rotation test with odometry monitoring")
        self.odometry_test.start_test_phase("rotation_phase", self.rotation_duration)
        
        # Start rotation
        self.get_logger().info("🔄 Starting 360° rotation...")
        start_time = time.time()
        
        # Create rotation command
        rotate_cmd = Twist()
        rotate_cmd.linear.x = 0.0
        rotate_cmd.angular.z = self.angular_speed
        
        # Rotate for specified duration
        while time.time() - start_time < self.rotation_duration:
            self.cmd_pub.publish(rotate_cmd)
            
            # Progress updates
            elapsed = time.time() - start_time
            if int(elapsed) % 5 == 0 and elapsed - int(elapsed) < 0.1:
                progress = (elapsed / self.rotation_duration) * 360
                self.get_logger().info(f"🔄 Progress: ~{progress:.0f}° ({elapsed:.1f}s)")
            
            # Mid-rotation screenshot
            if elapsed > self.rotation_duration / 2 and elapsed < (self.rotation_duration / 2 + 0.2):
                self.capture_screenshot("mid")
            
            # Process odometry callbacks
            rclpy.spin_once(self.odometry_test, timeout_sec=0.05)
            time.sleep(0.05)
        
        # Stop rotation
        stop_cmd = Twist()
        for _ in range(10):
            self.cmd_pub.publish(stop_cmd)
            time.sleep(0.05)
        
        self.odometry_test.end_test_phase("rotation_phase")
        self.get_logger().info("✅ Rotation completed")
        
        # Phase 3: Post-movement settling (5 seconds)
        self.get_logger().info("📊 Phase 3: Post-movement settling analysis")
        self.odometry_test.start_test_phase("post_movement", 5)
        
        # Take final screenshot
        self.capture_screenshot("final")
        
        # Continue odometry monitoring during settling
        for _ in range(50):  # 5 seconds at 10Hz
            rclpy.spin_once(self.odometry_test, timeout_sec=0.1)
            time.sleep(0.1)
            
        self.odometry_test.end_test_phase("post_movement")
        self.odometry_test.stop_test()
        
        # Generate results
        return self.generate_final_results()

    def generate_final_results(self):
        """Generate comprehensive test results"""
        self.get_logger().info("📊 Generating comprehensive test results...")
        
        # Get odometry test results
        odometry_report = self.odometry_test.generate_quality_report()
        self.odometry_test.print_quality_report(odometry_report)
        
        # Save odometry logs
        timestamp = int(time.time())
        odom_log_path = self.screenshot_dir / f"odometry_test_{timestamp}.json"
        self.odometry_test.save_detailed_logs(odom_log_path)
        
        # Run screenshot comparison
        screenshot_passed = self.run_screenshot_comparison()
        
        # Combine results
        final_results = {
            'timestamp': timestamp,
            'odometry_test': {
                'passed': odometry_report['overall_result'] == 'PASS',
                'report': odometry_report
            },
            'screenshot_test': {
                'passed': screenshot_passed
            },
            'overall_result': 'PASS' if (odometry_report['overall_result'] == 'PASS' and screenshot_passed) else 'FAIL'
        }
        
        # Save combined results
        results_path = self.screenshot_dir / f"comprehensive_results_{timestamp}.json"
        with open(results_path, 'w') as f:
            json.dump(final_results, f, indent=2, default=str)
        
        # Print final summary
        self.print_final_summary(final_results)
        
        return final_results['overall_result'] == 'PASS'

    def print_final_summary(self, results):
        """Print comprehensive test summary"""
        self.get_logger().info("\n" + "=" * 70)
        self.get_logger().info("🏁 COMPREHENSIVE REGRESSION TEST RESULTS")
        self.get_logger().info("=" * 70)
        
        # Odometry results
        odom_status = "✅ PASS" if results['odometry_test']['passed'] else "❌ FAIL"
        self.get_logger().info(f"🧭 Odometry Quality: {odom_status}")
        
        if not results['odometry_test']['passed']:
            # Show which odometry tests failed
            for test_name, result in results['odometry_test']['report']['pass_fail_results'].items():
                if not result['passed']:
                    self.get_logger().info(f"   ❌ {test_name}: {result['value']:.2f} (threshold: {result['threshold']})")
        
        # Screenshot results
        screenshot_status = "✅ PASS" if results['screenshot_test']['passed'] else "❌ FAIL"
        self.get_logger().info(f"📸 Screenshot Comparison: {screenshot_status}")
        
        # Overall result
        overall_status = "✅ PASS" if results['overall_result'] == 'PASS' else "❌ FAIL"
        self.get_logger().info(f"\n🎯 OVERALL REGRESSION RESULT: {overall_status}")
        
        if results['overall_result'] == 'PASS':
            self.get_logger().info("🎉 All tests passed! Robot navigation system is healthy.")
        else:
            self.get_logger().info("⚠️  Some tests failed. Check logs for detailed analysis.")
        
        self.get_logger().info("=" * 70)

def main():
    print("=" * 70)
    print("🔄 COMPREHENSIVE REGRESSION TEST")
    print("=" * 70)
    print("Enhanced testing with odometry quality monitoring")
    print("Tests both visual regression and navigation system health")
    print("")
    
    rclpy.init()
    
    try:
        test = RegressionWithOdometry()
        
        # Run comprehensive test
        success = test.run_comprehensive_test()
        
        if success:
            print("\n🎉 COMPREHENSIVE REGRESSION TEST PASSED!")
            return 0
        else:
            print("\n❌ COMPREHENSIVE REGRESSION TEST FAILED!")
            return 1
            
    except KeyboardInterrupt:
        print("\n🛑 Test interrupted by user")
        return 0
        
    except Exception as e:
        print(f"\n❌ Test error: {e}")
        import traceback
        traceback.print_exc()
        return 1
        
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    exit(main())