#!/usr/bin/env python3
"""
Test Laser Scan Coordinate Stability (Configurable for Real Robot or Simulation)

This test validates that laser scan data stays fixed relative to the map frame
when the robot rotates, detecting the bug where laser scans incorrectly rotate
with the robot in RViz display.

Supports both:
- Simulation mode: Launches Gazebo Classic and SLAM navigation
- Real robot mode: Uses existing running system

The test:
1. Starts robot in environment with static obstacles (or uses existing system)
2. Rotates the robot in place while recording laser scans and poses  
3. Transforms all laser scan points to map coordinates
4. Verifies that the same obstacles remain at consistent map positions
5. Fails if obstacles appear to move/rotate (indicating scan rotating with robot)
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist, PointStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import time
import math
import subprocess
import sys
import threading
import numpy as np
import tf2_ros
from tf2_geometry_msgs import do_transform_point
from collections import defaultdict
import json
import os


class LaserScanStabilityNode(Node):
    def __init__(self):
        super().__init__('laser_scan_stability_test')
        
        # Publishers and subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # TF2 setup for coordinate transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.orientation = 0.0
        
        # Data collection
        self.scan_data = []
        self.pose_data = []
        self.data_lock = threading.Lock()
        self.collecting_data = False
        
    def odom_callback(self, msg):
        """Process odometry data"""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        # Convert quaternion to yaw
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        self.orientation = math.atan2(siny_cosp, cosy_cosp)
    
    def scan_callback(self, msg):
        """Process laser scan data and store with current pose"""
        if not self.collecting_data:
            return
            
        # Only store if we have valid pose data
        if self.x != 0.0 or self.y != 0.0:
            with self.data_lock:
                # Store synchronized scan and pose data
                scan_time = time.time()
                pose_data = {
                    'x': self.x,
                    'y': self.y, 
                    'orientation': self.orientation,
                    'timestamp': scan_time
                }
                
                # Store essential scan data
                scan_data = {
                    'ranges': list(msg.ranges),
                    'angle_min': msg.angle_min,
                    'angle_max': msg.angle_max,
                    'angle_increment': msg.angle_increment,
                    'frame_id': msg.header.frame_id,
                    'timestamp': scan_time
                }
                
                self.scan_data.append(scan_data)
                self.pose_data.append(pose_data)
                
                if len(self.scan_data) % 10 == 0:
                    print(f"  Collected {len(self.scan_data)} scan/pose pairs...")
    
    def rotate_robot(self, duration_seconds=12, angular_velocity=0.81):
        """Rotate robot in place while collecting data"""
        print(f"\n--- Rotating robot for {duration_seconds} seconds at {angular_velocity} rad/s ({math.degrees(angular_velocity):.1f}°/s) ---")
        print(f"Expected total rotation: {math.degrees(angular_velocity * duration_seconds):.1f}° ({(angular_velocity * duration_seconds)/(2*math.pi):.1f} full rotations)")
        
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = angular_velocity
        
        # Start data collection
        self.collecting_data = True
        
        start_time = time.time()
        while (time.time() - start_time) < duration_seconds:
            self.cmd_pub.publish(cmd)
            time.sleep(0.1)
        
        # Stop robot
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)
        
        # Stop data collection  
        self.collecting_data = False
        
        print(f"✓ Rotation complete. Collected {len(self.scan_data)} scan/pose pairs")
        return len(self.scan_data) > 20  # Need sufficient data
    
    def transform_scan_to_map(self, scan_data, pose_data):
        """Transform laser scan points to map coordinates"""
        try:
            # Get transform from laser frame to map frame
            scan_frame = scan_data['frame_id']
            if scan_frame.startswith('/'):
                scan_frame = scan_frame[1:]
            
            # Handle frame_id differences between simulation and real robot
            # Gazebo Classic uses "laser", real robot uses "laser_frame"
            if scan_frame not in ['laser', 'laser_frame']:
                self.get_logger().warn(f"Unexpected laser frame_id: {scan_frame}")
            
            transform = self.tf_buffer.lookup_transform(
                'map', scan_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            map_points = []
            ranges = scan_data['ranges']
            angle_min = scan_data['angle_min']
            angle_increment = scan_data['angle_increment']
            
            for i, range_val in enumerate(ranges):
                # Skip invalid readings
                if math.isinf(range_val) or math.isnan(range_val) or range_val <= 0.1:
                    continue
                    
                # Skip readings that are too far (likely max range)
                if range_val > 10.0:
                    continue
                
                # Calculate point in laser frame
                angle = angle_min + i * angle_increment
                laser_x = range_val * math.cos(angle)
                laser_y = range_val * math.sin(angle)
                
                # Create point in laser frame
                laser_point = PointStamped()
                laser_point.header.frame_id = scan_frame
                laser_point.point.x = laser_x
                laser_point.point.y = laser_y
                laser_point.point.z = 0.0
                
                # Transform to map frame
                map_point = do_transform_point(laser_point, transform)
                
                map_points.append({
                    'x': map_point.point.x,
                    'y': map_point.point.y,
                    'range': range_val,
                    'angle': angle
                })
            
            return map_points
            
        except Exception as e:
            # Enhanced debugging for transform issues
            error_msg = str(e)
            if "does not exist" in error_msg or "Invalid frame" in error_msg:
                print(f"  Error: Frame issue - {error_msg}")
                # Try to get available frames (simplified approach)
                try:
                    # Check if we can at least transform between known frames
                    test_transform = self.tf_buffer.lookup_transform(
                        'base_link', 'laser_frame', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1)
                    )
                    print(f"  Note: base_link->laser_frame transform works")
                except:
                    print(f"  Error: Even base_link->laser_frame transform fails")
            elif "future" in error_msg.lower() or "time" in error_msg.lower():
                print(f"  Error: Timing issue - {error_msg}")
            else:
                print(f"  Error: Transform failed - {error_msg}")
            
            return []
    
    def calculate_stability_score(self, all_map_points, simulation_mode=True):
        """Calculate how stable obstacles are in map coordinates
        
        Args:
            all_map_points: List of transformed laser scan points
            simulation_mode: If True, use more lenient thresholds for simulation noise
        """
        if len(all_map_points) < 10:
            return 0.0
        
        print(f"\n--- Analyzing stability of {len(all_map_points)} scans ---")
        
        # Grid-based approach: discretize map into cells and track occupancy
        cell_size = 0.1  # 10cm grid cells
        grid_occupancy = defaultdict(list)  # cell -> list of timestamps when occupied
        
        total_points = 0
        for scan_idx, map_points in enumerate(all_map_points):
            for point in map_points:
                # Convert to grid coordinates
                grid_x = int(round(point['x'] / cell_size))
                grid_y = int(round(point['y'] / cell_size))
                grid_cell = (grid_x, grid_y)
                
                grid_occupancy[grid_cell].append(scan_idx)
                total_points += 1
        
        print(f"  Total points analyzed: {total_points}")
        print(f"  Unique grid cells: {len(grid_occupancy)}")
        
        # Calculate consistency: cells that should be consistently occupied
        consistent_cells = 0
        inconsistent_cells = 0
        
        # A cell is considered "stable" if it's occupied in at least 25% of scans (more lenient for simulation)
        occupancy_ratio = 0.25 if simulation_mode else 0.3
        min_occupancy_threshold = max(3, len(all_map_points) * occupancy_ratio)
        
        stable_cells = []
        for cell, occupancy_list in grid_occupancy.items():
            if len(occupancy_list) >= min_occupancy_threshold:
                stable_cells.append((cell, len(occupancy_list)))
                
                # Check if this stable cell maintains consistent occupancy
                # (not jumping around in time)
                occupancy_array = np.array(occupancy_list)
                if len(occupancy_array) > 5:
                    # Check for temporal consistency - stable obstacles should
                    # appear consistently throughout the rotation
                    time_span = np.max(occupancy_array) - np.min(occupancy_array)
                    span_ratio = 0.7 if simulation_mode else 0.8  # More lenient span for simulation
                    expected_span = len(all_map_points) * span_ratio
                    
                    if time_span >= expected_span:
                        consistent_cells += 1
                    else:
                        inconsistent_cells += 1
        
        print(f"  Stable cells (occupied in ≥{min_occupancy_threshold} scans): {len(stable_cells)}")
        print(f"  Consistently stable cells: {consistent_cells}")
        print(f"  Inconsistently stable cells: {inconsistent_cells}")
        
        if len(stable_cells) == 0:
            print("  ❌ No stable obstacles detected - environment may be empty")
            return 0.0
        
        # Calculate stability score
        stability_score = consistent_cells / (consistent_cells + inconsistent_cells + 1e-6)
        
        # Additional check: variance in obstacle positions
        # For very stable obstacles, calculate position variance
        position_variance_score = 1.0
        if len(stable_cells) >= 5:
            # Check position variance for the most stable obstacles
            top_stable_cells = sorted(stable_cells, key=lambda x: x[1], reverse=True)[:5]
            
            total_variance = 0.0
            for cell, _ in top_stable_cells:
                # Get all points in this cell across all scans
                cell_points = []
                for scan_idx, map_points in enumerate(all_map_points):
                    for point in map_points:
                        grid_x = int(round(point['x'] / cell_size))
                        grid_y = int(round(point['y'] / cell_size))
                        if (grid_x, grid_y) == cell:
                            cell_points.append([point['x'], point['y']])
                
                if len(cell_points) > 3:
                    cell_points = np.array(cell_points)
                    x_var = np.var(cell_points[:, 0])
                    y_var = np.var(cell_points[:, 1]) 
                    avg_variance = (x_var + y_var) / 2
                    total_variance += avg_variance
            
            avg_position_variance = total_variance / len(top_stable_cells)
            
            # Good stability means low variance (obstacles don't move around)
            # Variance should be < cell_size^2 for truly stable obstacles
            # Use more lenient threshold for simulation to account for noise
            variance_multiplier = 0.77 if simulation_mode else 0.5  # Allow more variance in simulation
            max_acceptable_variance = (cell_size * variance_multiplier) ** 2
            position_variance_score = max(0.0, 1.0 - (avg_position_variance / max_acceptable_variance))
            
            print(f"  Average position variance: {avg_position_variance:.6f}")
            print(f"  Position variance score: {position_variance_score:.3f}")
        
        # Combined stability score
        final_score = (stability_score + position_variance_score) / 2.0
        
        print(f"  Temporal stability score: {stability_score:.3f}")
        print(f"  Position stability score: {position_variance_score:.3f}")
        print(f"  Final stability score: {final_score:.3f}")
        
        return final_score


def test_laser_scan_stability(use_simulation=True):
    """Test that laser scan coordinates remain stable in map frame during robot rotation
    
    Args:
        use_simulation: If True, launches Gazebo Classic. If False, uses real robot.
    """
    
    mode_str = "Gazebo Classic" if use_simulation else "Real Robot"
    print(f"=== Laser Scan Coordinate Stability Test ({mode_str}) ===")
    print("This test verifies laser scans stay fixed in map frame when robot rotates")
    print("(Tests for the bug where laser scans rotate with robot in RViz)")
    
    sim_proc = None
    nav_proc = None
    
    try:
        # Check if system is already running (set by regression mode)
        system_already_running = os.environ.get('SYSTEM_ALREADY_RUNNING', '').lower() == 'true'
        
        if system_already_running:
            print("\n✅ Using pre-launched system (SYSTEM_ALREADY_RUNNING=true)")
            print("Regression mode detected - skipping system launch")
            print("Using already-running Gazebo Classic and SLAM navigation")
            time.sleep(3)
        elif use_simulation:
            # 1. Launch simulation environment
            print("\n--- Launching Gazebo Classic simulation ---")
            sim_command = [
                'ros2', 'launch', 'yahboomcar_nav', 'gazebo_classic_nav_launch.py',
                'world_name:=navigation_test_classic'
            ]
            
            sim_proc = subprocess.Popen(
                sim_command, 
                stdout=subprocess.PIPE, 
                stderr=subprocess.PIPE
            )
            
            # Wait for simulation to start
            print("  Waiting for Gazebo Classic to initialize...")
            time.sleep(10)
            
            # 2. Launch SLAM navigation 
            print("\n--- Launching SLAM navigation system ---")
            nav_command = [
                'ros2', 'launch', 'yahboomcar_nav', 'slam_mapping_gazebo.py'
            ]
            
            nav_proc = subprocess.Popen(
                nav_command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE  
            )
            
            print("  Waiting for SLAM system to initialize...")
            time.sleep(8)
        else:
            # For real robot, assume system is already running
            print("\n--- Using real robot ---")
            print("Assuming system is already initialized with b4m_HA_launch.sh")
            print("Make sure:")
            print("  1. Robot is powered on and connected")
            print("  2. Navigation system is running")
            print("  3. SLAM/localization is active")
            time.sleep(5)
        
        # 3. Initialize ROS2 node
        rclpy.init()
        node = LaserScanStabilityNode()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        
        # Start executor in separate thread
        executor_thread = threading.Thread(target=executor.spin)
        executor_thread.daemon = True
        executor_thread.start()
        
        # Wait for topics to be available and check if we're receiving data
        print("\n--- Waiting for robot sensors and navigation ---")
        time.sleep(8)  # Increased wait time for SLAM to initialize
        
        # Debug: Check if we're receiving laser scans
        print(f"  Current pose: x={node.x:.3f}, y={node.y:.3f}, θ={math.degrees(node.orientation):.1f}°")
        
        # Check what topics are available
        available_topics = [topic_name for topic_name, _ in node.get_topic_names_and_types()]
        scan_topics = [t for t in available_topics if 'scan' in t.lower()]
        odom_topics = [t for t in available_topics if 'odom' in t.lower()]
        print(f"  Available scan topics: {scan_topics}")
        print(f"  Available odom topics: {odom_topics}")
        
        # Wait for first scan to arrive
        scan_timeout = 15  # 15 seconds max
        print(f"  Waiting for laser scans (timeout: {scan_timeout}s)...")
        scan_start_time = time.time()
        node.collecting_data = True  # Temporarily enable to check for data
        
        while (time.time() - scan_start_time) < scan_timeout:
            if len(node.scan_data) > 0:
                print(f"  ✓ Received {len(node.scan_data)} laser scan(s)")
                node.scan_data.clear()  # Clear test data
                node.pose_data.clear()
                break
            time.sleep(0.5)
        else:
            print(f"  ❌ No laser scans received within {scan_timeout} seconds")
            print(f"     This may indicate laser sensor or SLAM system issues")
        
        node.collecting_data = False
        
        # Wait for map frame to be established by SLAM
        print(f"  Waiting for map frame to be established by SLAM...")
        
        # For Cartographer, we need some initial movement to establish the map
        print(f"  Performing small initial rotation to help Cartographer establish map frame...")
        cmd = Twist()
        cmd.angular.z = 0.3  # Gentle rotation
        for _ in range(20):  # 2 seconds of gentle rotation
            node.cmd_pub.publish(cmd)
            time.sleep(0.1)
        cmd.angular.z = 0.0
        node.cmd_pub.publish(cmd)
        time.sleep(1.0)  # Let the system stabilize
        
        map_timeout = 30  # 30 seconds max for map frame
        map_start_time = time.time()
        map_frame_available = False
        
        while (time.time() - map_start_time) < map_timeout:
            try:
                # Try to get a transform to the map frame
                transform = node.tf_buffer.lookup_transform(
                    'map', 'laser_frame', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0)
                )
                print(f"  ✓ Map frame established! SLAM is ready for coordinate transforms")
                map_frame_available = True
                break
            except Exception as e:
                # Map frame not yet available, keep waiting
                time.sleep(1.0)
        
        if not map_frame_available:
            print(f"  ❌ Map frame not established within {map_timeout} seconds")
            print(f"     SLAM may need more laser scan data or environmental features")
            # Continue anyway - the test might still work with some scans
        
        # 4. Collect data during robot rotation (360+ degrees to verify laser scans don't spin)
        print("\n--- Starting data collection phase ---")
        print("Rotating robot 360+ degrees to verify laser scans remain fixed in map frame")
        success = node.rotate_robot(duration_seconds=12, angular_velocity=0.81)
        
        if not success:
            print("❌ Failed to collect sufficient data")
            return 1
            
        # 5. Transform all laser scans to map coordinates
        print("\n--- Transforming laser scans to map coordinates ---")
        all_map_points = []
        successful_transforms = 0
        
        for i, (scan_data, pose_data) in enumerate(zip(node.scan_data, node.pose_data)):
            if i % 5 == 0:
                print(f"  Processing scan {i+1}/{len(node.scan_data)}... (success: {successful_transforms}/{i+1})")
                
            map_points = node.transform_scan_to_map(scan_data, pose_data)
            if map_points:
                all_map_points.append(map_points)
                successful_transforms += 1
            
            # Small delay to avoid overwhelming the system
            time.sleep(0.01)
        
        print(f"  Transform results: {successful_transforms}/{len(node.scan_data)} scans successfully transformed")
        
        if len(all_map_points) < 10:
            print("❌ Failed to transform sufficient scans to map coordinates")
            return 1
            
        print(f"✓ Successfully transformed {len(all_map_points)} scans to map coordinates")
        
        # 6. Calculate stability score
        stability_score = node.calculate_stability_score(all_map_points, simulation_mode=use_simulation)
        
        # 7. Save results for debugging
        results = {
            'stability_score': float(stability_score),
            'scans_collected': len(node.scan_data),
            'scans_transformed': len(all_map_points),
            'test_duration': 25,
            'test_mode': mode_str,
            'success': bool(stability_score > 0.85)
        }
        
        results_file = '/tmp/laser_scan_stability_results.json'
        with open(results_file, 'w') as f:
            json.dump(results, f, indent=2)
        
        # 8. Evaluate results
        print(f"\n=== RESULTS ===")
        print(f"Test Mode: {mode_str}")
        print(f"Stability Score: {stability_score:.3f}")
        print(f"Scans Collected: {len(node.scan_data)}")
        print(f"Scans Transformed: {len(all_map_points)}")
        
        # Use different thresholds for simulation vs real robot
        # Simulation has inherent noise that makes perfect stability harder to achieve
        STABILITY_THRESHOLD = 0.80 if use_simulation else 0.85  # 80% for simulation, 85% for real robot
        
        if stability_score > STABILITY_THRESHOLD:
            print(f"\n✅ LASER SCAN STABILITY TEST PASSED!")
            print(f"Laser scans remain stable in map coordinates (score: {stability_score:.3f} > {STABILITY_THRESHOLD})")
            print("Robot rotation does not cause laser scan coordinates to rotate")
            return 0
        else:
            print(f"\n❌ LASER SCAN STABILITY TEST FAILED!")
            print(f"Laser scans are unstable in map coordinates (score: {stability_score:.3f} ≤ {STABILITY_THRESHOLD})")
            print("This indicates laser scans may be rotating with the robot")
            print("Check transform chain: base_link -> laser_frame -> map")
            return 1
            
    except Exception as e:
        print(f"Error during test: {e}")
        return 1
        
    finally:
        print("\nCleaning up...")
        if 'node' in locals():
            node.destroy_node()
        if 'executor' in locals():
            executor.shutdown()
        try:
            rclpy.shutdown()
        except Exception as e:
            # Ignore shutdown errors which can occur if already shutdown
            pass
        
        # Check if system was already running (for cleanup decision)
        system_already_running = os.environ.get('SYSTEM_ALREADY_RUNNING', '').lower() == 'true'
        
        # Only clean up if we launched the system ourselves (not in regression mode)
        if not system_already_running:
            if use_simulation:
                for proc in [sim_proc, nav_proc]:
                    if proc and proc.poll() is None:
                        proc.terminate()
                        try:
                            proc.wait(timeout=5)
                        except subprocess.TimeoutExpired:
                            proc.kill()
                
                # Cleanup Gazebo Classic processes
                subprocess.run(['pkill', '-f', 'gazebo'], capture_output=True)
                subprocess.run(['pkill', '-f', 'gzserver'], capture_output=True)
                subprocess.run(['pkill', '-f', 'gzclient'], capture_output=True)
                subprocess.run(['pkill', '-f', 'robot_state_publisher'], capture_output=True)
                subprocess.run(['pkill', '-f', 'cartographer'], capture_output=True)  # Updated for Cartographer
                time.sleep(2)
        else:
            print("Regression mode: Leaving system running (cleanup handled by main script)")


if __name__ == "__main__":
    # Check for command line argument to determine mode
    use_sim = True  # Default to simulation
    
    if len(sys.argv) > 1:
        if sys.argv[1] == "--real-robot":
            use_sim = False
        elif sys.argv[1] == "--simulation":
            use_sim = True
        elif sys.argv[1] == "--help":
            print("Usage: python3 test_laser_scan_stability_configurable.py [--simulation|--real-robot]")
            print("  --simulation: Run test in Gazebo Classic (default)")
            print("  --real-robot: Run test on real robot (system must be running)")
            sys.exit(0)
    
    # Check environment variable as alternative (for b4m_HA_launch.sh integration)
    if os.environ.get('TEST_MODE') == 'REAL_ROBOT':
        use_sim = False
    elif os.environ.get('TEST_MODE') == 'SIMULATION':
        use_sim = True
    
    sys.exit(test_laser_scan_stability(use_sim))