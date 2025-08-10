#!/usr/bin/env python3
"""Check if map is being published"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import sys

class MapChecker(Node):
    def __init__(self):
        super().__init__('map_checker')
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.map_received = False
        self.map_info = None
        
    def map_callback(self, msg):
        self.map_received = True
        self.map_info = msg.info
        print(f"✅ Map received! Size: {msg.info.width}x{msg.info.height}, "
              f"Resolution: {msg.info.resolution}m/pixel")
        
def main():
    rclpy.init()
    node = MapChecker()
    
    print("Checking for /map topic...")
    timeout = 10
    start = rclpy.time.Time()
    
    while not node.map_received and node.get_clock().now().nanoseconds/1e9 < timeout:
        rclpy.spin_once(node, timeout_sec=0.5)
        
    if node.map_received:
        print("✅ Map is being published by Cartographer")
        return 0
    else:
        print("❌ No map received - Cartographer may not be publishing yet")
        print("   Cartographer needs robot movement to start building the map")
        return 1
        
    rclpy.shutdown()

if __name__ == "__main__":
    sys.exit(main())