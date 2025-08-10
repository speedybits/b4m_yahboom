#!/usr/bin/env python3
"""
Quick test to check laser scan visualization in RViz
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import time

class LaserVisualizationTest(Node):
    def __init__(self):
        super().__init__('laser_viz_test')
        
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.scan_count = 0
        self.last_scan = None
        
    def scan_callback(self, msg):
        self.scan_count += 1
        self.last_scan = msg
        
        # Log every 10th scan
        if self.scan_count % 10 == 0:
            valid_readings = [r for r in msg.ranges if 0.1 < r < 12.0]
            if valid_readings:
                min_dist = min(valid_readings)
                max_dist = max(valid_readings)
                avg_dist = sum(valid_readings) / len(valid_readings)
                print(f"Scan #{self.scan_count}: {len(valid_readings)} valid points, "
                      f"min={min_dist:.2f}m, max={max_dist:.2f}m, avg={avg_dist:.2f}m")
            else:
                print(f"Scan #{self.scan_count}: No valid readings!")
                
    def rotate_slowly(self):
        """Rotate robot slowly to test laser visibility"""
        print("\nRotating robot slowly for 10 seconds...")
        cmd = Twist()
        cmd.angular.z = 0.3  # Slow rotation
        
        start_time = time.time()
        while (time.time() - start_time) < 10:
            self.cmd_pub.publish(cmd)
            time.sleep(0.1)
            
        # Stop
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)
        print("Rotation complete")

def main():
    print("=== Laser Visualization Test ===")
    print("This test checks if laser scans are being published")
    print("and helps debug visualization issues in RViz\n")
    
    rclpy.init()
    node = LaserVisualizationTest()
    
    try:
        # Check for laser data
        print("Waiting for laser scan data...")
        timeout = 5
        start = time.time()
        while node.scan_count == 0 and (time.time() - start) < timeout:
            rclpy.spin_once(node, timeout_sec=0.1)
            
        if node.scan_count > 0:
            print(f"✅ Laser scan data detected! Rate: ~{node.scan_count/(time.time()-start):.1f} Hz")
            
            # Rotate to generate data
            node.rotate_slowly()
            
            # Spin for a bit more to collect data
            print("\nCollecting more laser data for 5 seconds...")
            start = time.time()
            while (time.time() - start) < 5:
                rclpy.spin_once(node, timeout_sec=0.1)
                
            print(f"\n✅ Total scans received: {node.scan_count}")
            print("\nRViz LaserScan Display Settings:")
            print("1. Topic: /scan")
            print("2. Size (m): 0.05")
            print("3. Style: Points or Squares")
            print("4. Color Transformer: FlatColor")
            print("5. Color: Red (255, 0, 0)")
            print("\nIf you don't see laser points, check:")
            print("- Fixed Frame is set to 'odom' or 'base_link'")
            print("- LaserScan display is enabled (checkbox)")
            print("- Size is large enough to see (try 0.1)")
            
        else:
            print("❌ No laser scan data received!")
            print("Check that Gazebo is running and robot is spawned")
            
    except KeyboardInterrupt:
        print("\nTest interrupted")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()