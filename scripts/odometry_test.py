#!/usr/bin/env python3
"""
Odometry Quality Test for Regression Testing

This script monitors odometry quality during regression tests to detect:
- Position drift when stationary
- Orientation drift and noise
- Update frequency consistency
- Velocity command tracking accuracy
- Overall noise levels and stability

The test runs in parallel with rotation tests to provide comprehensive
odometry analysis during both stationary and movement phases.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np
import math
import time
import json
from pathlib import Path
from collections import deque
import threading

class OdometryQualityTest(Node):
    def __init__(self):
        super().__init__('odometry_quality_test')
        
        # Odometry subscriber - always use /odom (filtered output from EKF)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Cmd_vel subscriber to track commanded velocities
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Test data storage
        self.odom_data = []
        self.cmd_vel_data = []
        self.test_phases = {}
        self.current_phase = None
        
        # Real-time tracking for noise analysis
        self.position_history = deque(maxlen=50)  # Last 5 seconds at 10Hz
        self.orientation_history = deque(maxlen=50)
        self.velocity_history = deque(maxlen=50)
        self.timestamp_history = deque(maxlen=50)
        
        # Test parameters
        self.stationary_threshold = 0.01  # m/s - consider stationary below this
        self.max_position_drift = 0.05    # 5cm max drift when stationary
        self.max_orientation_drift = math.radians(10)  # 10 degrees max drift (adjusted for real robot)
        self.min_update_rate = 5.0        # Hz (lowered for simulation compatibility)
        self.max_update_rate = 50.0       # Hz
        self.max_velocity_error = 0.2     # 20% error threshold
        
        # Quality tracking
        self.quality_metrics = {
            'stationary_position_drift': [],
            'stationary_orientation_drift': [],
            'update_intervals': [],
            'velocity_errors': [],
            'position_noise': [],
            'orientation_noise': []
        }
        
        # Test state
        self.test_start_time = None
        self.last_odom_time = None
        self.test_running = False
        
        self.get_logger().info("🧭 Odometry Quality Test initialized")
        self.get_logger().info(f"   Position drift limit: {self.max_position_drift*100:.1f}cm")
        self.get_logger().info(f"   Orientation drift limit: {math.degrees(self.max_orientation_drift):.1f}°")
        self.get_logger().info(f"   Update rate range: {self.min_update_rate}-{self.max_update_rate}Hz")

    def odom_callback(self, msg):
        """Process odometry messages and track quality metrics"""
        current_time = time.time()
        
        if not self.test_running:
            return
            
        # Extract position and orientation
        pos_x = msg.pose.pose.position.x
        pos_y = msg.pose.pose.position.y
        
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 
                        1 - 2 * (q.y * q.y + q.z * q.z))
        
        # Extract velocities
        linear_vel = msg.twist.twist.linear.x
        angular_vel = msg.twist.twist.angular.z
        
        # Store data point
        data_point = {
            'timestamp': current_time,
            'position': [pos_x, pos_y],
            'orientation': yaw,
            'linear_velocity': linear_vel,
            'angular_velocity': angular_vel,
            'phase': self.current_phase
        }
        self.odom_data.append(data_point)
        
        # Track update intervals
        if self.last_odom_time is not None:
            interval = current_time - self.last_odom_time
            self.quality_metrics['update_intervals'].append(interval)
        self.last_odom_time = current_time
        
        # Add to rolling history
        self.position_history.append([pos_x, pos_y])
        self.orientation_history.append(yaw)
        self.velocity_history.append([linear_vel, angular_vel])
        self.timestamp_history.append(current_time)
        
        # Analyze noise and drift in real-time
        self._analyze_realtime_quality()

    def cmd_vel_callback(self, msg):
        """Track commanded velocities for comparison with odometry"""
        if not self.test_running:
            return
            
        current_time = time.time()
        cmd_data = {
            'timestamp': current_time,
            'linear_cmd': msg.linear.x,
            'angular_cmd': msg.angular.z,
            'phase': self.current_phase
        }
        self.cmd_vel_data.append(cmd_data)

    def _analyze_realtime_quality(self):
        """Analyze quality metrics on rolling window of recent data"""
        if len(self.position_history) < 10:
            return
            
        # Convert to numpy arrays for analysis
        positions = np.array(list(self.position_history))
        orientations = np.array(list(self.orientation_history))
        velocities = np.array(list(self.velocity_history))
        timestamps = np.array(list(self.timestamp_history))
        
        # Check if currently stationary (low velocity for recent samples)
        recent_velocities = velocities[-10:]  # Last 1 second
        linear_speeds = np.abs(recent_velocities[:, 0])
        angular_speeds = np.abs(recent_velocities[:, 1])
        is_stationary = (np.mean(linear_speeds) < self.stationary_threshold and 
                        np.mean(angular_speeds) < self.stationary_threshold)
        
        if is_stationary:
            # Analyze drift during stationary period
            recent_positions = positions[-10:]
            recent_orientations = orientations[-10:]
            
            # Position drift (max distance from mean position)
            mean_position = np.mean(recent_positions, axis=0)
            distances = np.linalg.norm(recent_positions - mean_position, axis=1)
            max_drift = np.max(distances)
            self.quality_metrics['stationary_position_drift'].append(max_drift)
            
            # Orientation drift
            mean_orientation = np.mean(recent_orientations)
            orientation_diffs = np.abs(recent_orientations - mean_orientation)
            # Handle angle wraparound
            orientation_diffs = np.minimum(orientation_diffs, 2*np.pi - orientation_diffs)
            max_orient_drift = np.max(orientation_diffs)
            self.quality_metrics['stationary_orientation_drift'].append(max_orient_drift)
        
        # General noise analysis
        if len(positions) >= 20:
            # Position noise (standard deviation)
            position_std = np.std(positions[-20:], axis=0)
            self.quality_metrics['position_noise'].append(np.mean(position_std))
            
            # Orientation noise
            orientation_std = np.std(orientations[-20:])
            self.quality_metrics['orientation_noise'].append(orientation_std)

    def start_test_phase(self, phase_name, duration=None):
        """Start a new test phase"""
        self.current_phase = phase_name
        self.test_phases[phase_name] = {
            'start_time': time.time(),
            'duration': duration,
            'start_data_index': len(self.odom_data)
        }
        
        self.get_logger().info(f"📊 Starting odometry test phase: {phase_name}")
        if duration:
            self.get_logger().info(f"   Duration: {duration}s")

    def end_test_phase(self, phase_name):
        """End current test phase"""
        if phase_name in self.test_phases:
            self.test_phases[phase_name]['end_time'] = time.time()
            self.test_phases[phase_name]['end_data_index'] = len(self.odom_data)
            
        self.get_logger().info(f"📊 Completed odometry test phase: {phase_name}")

    def start_test(self):
        """Start the odometry quality test"""
        self.test_start_time = time.time()
        self.test_running = True
        self.get_logger().info("🚀 Starting comprehensive odometry quality test")

    def stop_test(self):
        """Stop the odometry quality test"""
        self.test_running = False
        self.get_logger().info("🛑 Stopping odometry quality test")

    def analyze_velocity_tracking(self):
        """Analyze how well odometry tracks commanded velocities"""
        if len(self.cmd_vel_data) < 10 or len(self.odom_data) < 10:
            return
            
        # Match cmd_vel with closest odometry reading
        for cmd in self.cmd_vel_data:
            cmd_time = cmd['timestamp']
            
            # Find closest odometry reading
            closest_odom = min(self.odom_data, 
                             key=lambda x: abs(x['timestamp'] - cmd_time))
            
            time_diff = abs(closest_odom['timestamp'] - cmd_time)
            
            # Only analyze if readings are close in time
            if time_diff < 0.2:  # Within 200ms
                if abs(cmd['linear_cmd']) > 0.05:  # Non-zero command
                    error = abs(closest_odom['linear_velocity'] - cmd['linear_cmd'])
                    relative_error = error / abs(cmd['linear_cmd'])
                    self.quality_metrics['velocity_errors'].append(relative_error)

    def generate_quality_report(self):
        """Generate comprehensive quality analysis report"""
        self.analyze_velocity_tracking()
        
        report = {
            'test_duration': time.time() - self.test_start_time if self.test_start_time else 0,
            'total_samples': len(self.odom_data),
            'test_phases': dict(self.test_phases),
            'quality_metrics': {},
            'pass_fail_results': {},
            'overall_result': 'UNKNOWN'
        }
        
        # Analyze each quality metric
        metrics = self.quality_metrics
        
        # Update rate analysis
        if metrics['update_intervals']:
            intervals = np.array(metrics['update_intervals'])
            update_rate = 1.0 / np.mean(intervals)
            rate_std = np.std(1.0 / intervals)
            
            report['quality_metrics']['update_rate'] = {
                'mean_hz': float(update_rate),
                'std_hz': float(rate_std),
                'min_interval_ms': float(np.min(intervals) * 1000),
                'max_interval_ms': float(np.max(intervals) * 1000)
            }
            
            rate_ok = (self.min_update_rate <= update_rate <= self.max_update_rate)
            report['pass_fail_results']['update_rate'] = {
                'passed': rate_ok,
                'value': update_rate,
                'threshold': f"{self.min_update_rate}-{self.max_update_rate}Hz"
            }
        
        # Position drift analysis
        if metrics['stationary_position_drift']:
            pos_drift = np.array(metrics['stationary_position_drift'])
            max_drift = float(np.max(pos_drift))
            mean_drift = float(np.mean(pos_drift))
            
            report['quality_metrics']['position_drift'] = {
                'max_drift_cm': max_drift * 100,
                'mean_drift_cm': mean_drift * 100,
                'samples': len(pos_drift)
            }
            
            drift_ok = max_drift <= self.max_position_drift
            report['pass_fail_results']['position_drift'] = {
                'passed': drift_ok,
                'value': max_drift * 100,
                'threshold': f"{self.max_position_drift * 100:.1f}cm"
            }
        
        # Orientation drift analysis
        if metrics['stationary_orientation_drift']:
            orient_drift = np.array(metrics['stationary_orientation_drift'])
            max_drift = float(np.max(orient_drift))
            mean_drift = float(np.mean(orient_drift))
            
            report['quality_metrics']['orientation_drift'] = {
                'max_drift_deg': math.degrees(max_drift),
                'mean_drift_deg': math.degrees(mean_drift),
                'samples': len(orient_drift)
            }
            
            drift_ok = max_drift <= self.max_orientation_drift
            report['pass_fail_results']['orientation_drift'] = {
                'passed': drift_ok,
                'value': math.degrees(max_drift),
                'threshold': f"{math.degrees(self.max_orientation_drift):.1f}°"
            }
        
        # Velocity tracking analysis
        if metrics['velocity_errors']:
            vel_errors = np.array(metrics['velocity_errors'])
            max_error = float(np.max(vel_errors))
            mean_error = float(np.mean(vel_errors))
            
            report['quality_metrics']['velocity_tracking'] = {
                'max_error_percent': max_error * 100,
                'mean_error_percent': mean_error * 100,
                'samples': len(vel_errors)
            }
            
            vel_ok = max_error <= self.max_velocity_error
            report['pass_fail_results']['velocity_tracking'] = {
                'passed': vel_ok,
                'value': max_error * 100,
                'threshold': f"{self.max_velocity_error * 100:.1f}%"
            }
        
        # Noise analysis
        if metrics['position_noise']:
            pos_noise = np.array(metrics['position_noise'])
            report['quality_metrics']['position_noise'] = {
                'mean_std_cm': float(np.mean(pos_noise)) * 100,
                'max_std_cm': float(np.max(pos_noise)) * 100
            }
        
        if metrics['orientation_noise']:
            orient_noise = np.array(metrics['orientation_noise'])
            report['quality_metrics']['orientation_noise'] = {
                'mean_std_deg': float(np.mean(orient_noise)) * 180 / np.pi,
                'max_std_deg': float(np.max(orient_noise)) * 180 / np.pi
            }
        
        # Overall pass/fail
        all_passed = all(result.get('passed', True) for result in report['pass_fail_results'].values())
        report['overall_result'] = 'PASS' if all_passed else 'FAIL'
        
        return report

    def print_quality_report(self, report):
        """Print formatted quality report"""
        self.get_logger().info("\n" + "=" * 60)
        self.get_logger().info("🧭 ODOMETRY QUALITY TEST RESULTS")
        self.get_logger().info("=" * 60)
        
        # Test overview
        self.get_logger().info(f"📊 Test Duration: {report['test_duration']:.1f}s")
        self.get_logger().info(f"📊 Total Samples: {report['total_samples']}")
        
        # Pass/fail results
        self.get_logger().info(f"\n🎯 PASS/FAIL RESULTS:")
        for test_name, result in report['pass_fail_results'].items():
            status = "✅ PASS" if result['passed'] else "❌ FAIL"
            self.get_logger().info(f"   {test_name}: {status} ({result['value']:.2f} vs {result['threshold']})")
        
        # Detailed metrics
        self.get_logger().info(f"\n📈 DETAILED METRICS:")
        for metric_name, data in report['quality_metrics'].items():
            self.get_logger().info(f"   {metric_name}:")
            for key, value in data.items():
                self.get_logger().info(f"     {key}: {value}")
        
        # Overall result
        overall_status = "✅ PASS" if report['overall_result'] == 'PASS' else "❌ FAIL"
        self.get_logger().info(f"\n🏁 OVERALL RESULT: {overall_status}")
        self.get_logger().info("=" * 60)

    def save_detailed_logs(self, filepath):
        """Save detailed odometry logs for analysis"""
        report = self.generate_quality_report()
        
        detailed_data = {
            'quality_report': report,
            'odometry_data': self.odom_data,
            'cmd_vel_data': self.cmd_vel_data,
            'test_phases': self.test_phases
        }
        
        with open(filepath, 'w') as f:
            json.dump(detailed_data, f, indent=2, default=str)
        
        self.get_logger().info(f"📄 Detailed odometry logs saved to: {filepath}")

def main():
    print("🧭 Odometry Quality Test")
    print("========================")
    print("Comprehensive odometry analysis for regression testing")
    print("")
    
    rclpy.init()
    
    try:
        test = OdometryQualityTest()
        
        # Run test phases
        test.start_test()
        
        # Phase 1: Stationary baseline (10 seconds)
        test.start_test_phase("stationary_baseline", 10)
        time.sleep(10)
        test.end_test_phase("stationary_baseline")
        
        # Phase 2: Movement phase (20 seconds) - would run during robot rotation
        test.start_test_phase("movement_phase", 20)
        print("🔄 Movement phase - robot should be rotating now...")
        time.sleep(20)
        test.end_test_phase("movement_phase")
        
        # Phase 3: Post-movement settling (5 seconds)
        test.start_test_phase("post_movement", 5)
        time.sleep(5)
        test.end_test_phase("post_movement")
        
        test.stop_test()
        
        # Generate and print report
        report = test.generate_quality_report()
        test.print_quality_report(report)
        
        # Save detailed logs
        log_path = Path("regression") / "screenshots" / f"odometry_test_{int(time.time())}.json"
        log_path.parent.mkdir(exist_ok=True)
        test.save_detailed_logs(log_path)
        
        # Return exit code based on results
        return 0 if report['overall_result'] == 'PASS' else 1
        
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