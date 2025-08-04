#!/usr/bin/env python3

"""
Map Validation Script for SLAM Testing
Validates SLAM map quality, obstacle detection, and saves maps
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Empty
import subprocess
import os
import time
import json
import numpy as np

class MapValidation(Node):
    def __init__(self):
        super().__init__('map_validation')
        
        # Subscribe to map topic
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        
        # Map saving service client (SLAM toolbox)
        self.save_map_client = self.create_client(Empty, '/slam_toolbox/save_map')
        
        # Map data
        self.current_map = None
        self.map_received = False
        
        # Validation results
        self.results = {
            'map_received': False,
            'map_size': None,
            'occupied_cells': 0,
            'free_cells': 0,
            'unknown_cells': 0,
            'obstacles_detected': 0,
            'map_quality_score': 0.0,
            'map_saved': False,
            'map_files_exist': False,
            'validation_success': False
        }
        
        self.get_logger().info("Map Validation initialized")
        
    def map_callback(self, msg):
        self.current_map = msg
        self.map_received = True
        self.results['map_received'] = True
        
        # Analyze map data
        self.analyze_map_data(msg)
        
    def analyze_map_data(self, map_msg):
        """Analyze occupancy grid data for quality metrics"""
        data = np.array(map_msg.data)
        
        # Count different cell types
        # Occupancy grid values: -1 (unknown), 0 (free), 100 (occupied)
        unknown_cells = np.sum(data == -1)
        free_cells = np.sum(data == 0)
        occupied_cells = np.sum(data >= 50)  # Threshold for occupied
        
        self.results.update({
            'map_size': (map_msg.info.width, map_msg.info.height),
            'occupied_cells': int(occupied_cells),
            'free_cells': int(free_cells),
            'unknown_cells': int(unknown_cells)
        })
        
        # Estimate obstacles (clusters of occupied cells)
        self.estimate_obstacles(data, map_msg.info.width, map_msg.info.height)
        
        # Calculate map quality score
        total_cells = len(data)
        if total_cells > 0:
            known_cells = free_cells + occupied_cells
            knowledge_ratio = known_cells / total_cells
            
            # Good SLAM map should have reasonable amount of both free and occupied space
            if free_cells > 0:
                occupied_ratio = occupied_cells / (free_cells + occupied_cells)
                balance_score = 1.0 - abs(occupied_ratio - 0.1)  # Expect ~10% occupied for indoor
            else:
                balance_score = 0.0
                
            self.results['map_quality_score'] = (knowledge_ratio * 0.7 + balance_score * 0.3)
        
        self.get_logger().info(f"Map analysis: {free_cells} free, {occupied_cells} occupied, {unknown_cells} unknown cells")
        self.get_logger().info(f"Map quality score: {self.results['map_quality_score']:.3f}")
        
    def estimate_obstacles(self, data, width, height):
        """Estimate number of distinct obstacles from occupied cells"""
        # Convert 1D array to 2D grid
        grid = data.reshape((height, width))
        
        # Find connected components of occupied cells
        occupied_mask = grid >= 50
        
        # Simple connected component analysis (flood fill)
        visited = np.zeros_like(occupied_mask)
        obstacle_count = 0
        
        for y in range(height):
            for x in range(width):
                if occupied_mask[y, x] and not visited[y, x]:
                    # Found new obstacle, flood fill to mark all connected cells
                    stack = [(x, y)]
                    cells_in_obstacle = 0
                    
                    while stack:
                        cx, cy = stack.pop()
                        if (0 <= cx < width and 0 <= cy < height and 
                            not visited[cy, cx] and occupied_mask[cy, cx]):
                            
                            visited[cy, cx] = True
                            cells_in_obstacle += 1
                            
                            # Add neighbors to stack
                            for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
                                stack.append((cx+dx, cy+dy))
                    
                    # Only count as obstacle if it has enough cells (filter noise)
                    if cells_in_obstacle >= 5:
                        obstacle_count += 1
        
        self.results['obstacles_detected'] = obstacle_count
        self.get_logger().info(f"Estimated obstacles in map: {obstacle_count}")
        
    def save_map_slam_toolbox(self):
        """Save map using SLAM toolbox service"""
        if not self.save_map_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("SLAM toolbox save_map service not available")
            return False
            
        try:
            request = Empty.Request()
            future = self.save_map_client.call_async(request)
            
            # Wait for service call to complete
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            
            if future.result() is not None:
                self.results['map_saved'] = True
                self.get_logger().info("Map saved via SLAM toolbox service")
                return True
            else:
                self.get_logger().error("SLAM toolbox save_map service call failed")
                return False
                
        except Exception as e:
            self.get_logger().error(f"Error calling save_map service: {e}")
            return False
    
    def save_map_nav2(self, map_name="slam_test_map"):
        """Fallback: Save map using nav2 map_saver"""
        try:
            # Use nav2 map_saver
            cmd = [
                'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                '-f', map_name,
                '--occ', '0.65',
                '--free', '0.25'
            ]
            
            result = subprocess.run(cmd, cwd='/tmp', capture_output=True, text=True, timeout=30)
            
            if result.returncode == 0:
                self.results['map_saved'] = True
                self.get_logger().info(f"Map saved via nav2 map_saver: {map_name}")
                return True
            else:
                self.get_logger().error(f"nav2 map_saver failed: {result.stderr}")
                return False
                
        except subprocess.TimeoutExpired:
            self.get_logger().error("Map saving timed out")
            return False
        except Exception as e:
            self.get_logger().error(f"Error saving map: {e}")
            return False
    
    def validate_map_files(self, map_name="slam_test_map"):
        """Check if map files were created successfully"""
        yaml_file = f"/tmp/{map_name}.yaml"
        pgm_file = f"/tmp/{map_name}.pgm"
        
        yaml_exists = os.path.exists(yaml_file)
        pgm_exists = os.path.exists(pgm_file)
        
        self.results['map_files_exist'] = yaml_exists and pgm_exists
        
        if yaml_exists:
            try:
                # Check YAML file content
                with open(yaml_file, 'r') as f:
                    content = f.read()
                    if 'image:' in content and 'resolution:' in content:
                        self.get_logger().info(f"Map YAML file valid: {yaml_file}")
                    else:
                        self.get_logger().warning(f"Map YAML file may be invalid: {yaml_file}")
            except Exception as e:
                self.get_logger().error(f"Error reading map YAML: {e}")
        
        if pgm_exists:
            try:
                # Check PGM file size
                size = os.path.getsize(pgm_file)
                if size > 1000:  # Should be reasonably large
                    self.get_logger().info(f"Map PGM file valid: {pgm_file} ({size} bytes)")
                else:
                    self.get_logger().warning(f"Map PGM file may be too small: {size} bytes")
            except Exception as e:
                self.get_logger().error(f"Error checking map PGM: {e}")
        
        return self.results['map_files_exist']
    
    def run_validation(self):
        """Run complete map validation sequence"""
        self.get_logger().info("Starting map validation sequence...")
        
        # Wait for map data
        timeout = 30.0
        start_time = time.time()
        
        while not self.map_received and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if not self.map_received:
            self.get_logger().error("No map data received within timeout")
            return False
        
        # Save map using SLAM toolbox
        self.get_logger().info("Attempting to save map via SLAM toolbox...")
        if not self.save_map_slam_toolbox():
            # Fallback to nav2 map_saver
            self.get_logger().info("Fallback: Attempting to save map via nav2 map_saver...")
            self.save_map_nav2()
        
        # Wait a moment for file system operations
        time.sleep(2)
        
        # Validate map files
        self.get_logger().info("Validating saved map files...")
        self.validate_map_files()
        
        # Overall success criteria
        self.results['validation_success'] = (
            self.results['map_received'] and
            self.results['map_quality_score'] > 0.3 and
            self.results['obstacles_detected'] >= 2 and  # Expect at least 2 obstacles
            (self.results['map_saved'] or self.results['map_files_exist'])
        )
        
        # Save validation results
        self.save_validation_results()
        
        return self.results['validation_success']
    
    def save_validation_results(self):
        """Save validation results to file"""
        results_file = '/tmp/map_validation_results.json'
        
        try:
            with open(results_file, 'w') as f:
                json.dump(self.results, f, indent=2)
            
            self.get_logger().info(f"Validation results saved to {results_file}")
            self.get_logger().info(f"Map validation success: {self.results['validation_success']}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to save validation results: {e}")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        validator = MapValidation()
        success = validator.run_validation()
        
        # Print final results
        print(f"\n{'='*50}")
        print("MAP VALIDATION RESULTS")
        print(f"{'='*50}")
        print(f"Map received: {validator.results['map_received']}")
        print(f"Map quality score: {validator.results['map_quality_score']:.3f}")
        print(f"Obstacles detected: {validator.results['obstacles_detected']}")
        print(f"Map saved: {validator.results['map_saved']}")
        print(f"Map files exist: {validator.results['map_files_exist']}")
        print(f"Overall success: {validator.results['validation_success']}")
        print(f"{'='*50}")
        
        return 0 if success else 1
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Map validation failed: {e}")
        return 1
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    exit(main())