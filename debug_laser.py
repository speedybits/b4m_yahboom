#!/usr/bin/env python3
"""
Debug script to analyze laser scan data and front sector calculation
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import math

class LaserDebugger(Node):
    def __init__(self):
        super().__init__('laser_debugger')
        
        self.laser_sub = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.laser_callback, 
            10
        )
        self.count = 0
        
    def laser_callback(self, msg):
        self.count += 1
        if self.count % 10 != 0:  # Only process every 10th message
            return
            
        ranges = np.array(msg.ranges)
        ranges[np.isinf(ranges)] = msg.range_max
        ranges[np.isnan(ranges)] = msg.range_max
        
        print(f"\n=== LASER DEBUG ({self.count}) ===")
        print(f"Total points: {len(ranges)}")
        print(f"Angle min: {math.degrees(msg.angle_min):.1f}°")
        print(f"Angle max: {math.degrees(msg.angle_max):.1f}°")
        print(f"Angle increment: {math.degrees(msg.angle_increment):.2f}°")
        
        # Front sector calculation (10 degrees each side = 20 degrees total)
        front_angle = 10  # degrees on each side of center
        num_readings = len(ranges)
        readings_per_degree = num_readings / 360
        front_start = int((360 - front_angle) * readings_per_degree)
        front_end = int(front_angle * readings_per_degree)
        
        print(f"Readings per degree: {readings_per_degree:.2f}")
        print(f"Front sector indices: {front_start} to {len(ranges)-1} and 0 to {front_end-1}")
        
        # Get front ranges
        front_ranges = np.concatenate([ranges[front_start:], ranges[:front_end]])
        min_front_distance = np.min(front_ranges)
        
        # Get overall min
        min_overall = np.min(ranges)
        
        print(f"Front path distance: {min_front_distance:.2f}m")
        print(f"Overall closest: {min_overall:.2f}m")
        
        # Show some specific angles
        center_idx = len(ranges) // 2  # Should be 0 degrees (front)
        print(f"\nSample readings:")
        print(f"0° (front): {ranges[0]:.2f}m")
        print(f"90° (left): {ranges[len(ranges)//4]:.2f}m") 
        print(f"180° (back): {ranges[len(ranges)//2]:.2f}m")
        print(f"270° (right): {ranges[3*len(ranges)//4]:.2f}m")
        
        # Front sector details
        print(f"\nFront sector readings:")
        front_indices = list(range(front_start, len(ranges))) + list(range(0, front_end))
        for i, idx in enumerate(front_indices[::5]):  # Every 5th reading
            angle = (idx / num_readings) * 360
            if angle > 180:
                angle -= 360
            print(f"  {angle:+6.1f}°: {ranges[idx]:.2f}m")

def main():
    rclpy.init()
    
    debugger = LaserDebugger()
    
    print("Laser debugger started. Press Ctrl+C to stop.")
    
    try:
        rclpy.spin(debugger)
    except KeyboardInterrupt:
        print("\nLaser debugger stopped.")
    
    debugger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()