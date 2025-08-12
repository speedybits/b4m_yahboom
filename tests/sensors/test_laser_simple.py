#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
import math

class LaserPublisher(Node):
    def __init__(self):
        super().__init__('test_laser_publisher')
        self.publisher_ = self.create_publisher(LaserScan, '/scan', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz
        self.get_logger().info('Test laser publisher started')

    def timer_callback(self):
        msg = LaserScan()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser'
        
        # Laser parameters
        msg.angle_min = -math.pi
        msg.angle_max = math.pi
        msg.angle_increment = math.pi / 180  # 1 degree
        msg.time_increment = 0.0
        msg.scan_time = 0.1
        msg.range_min = 0.15
        msg.range_max = 12.0
        
        # Generate fake scan data (360 points)
        num_readings = int((msg.angle_max - msg.angle_min) / msg.angle_increment)
        msg.ranges = [1.0] * num_readings  # All points at 1 meter
        msg.intensities = []
        
        self.publisher_.publish(msg)
        self.get_logger().info('Published laser scan', once=True)

def main(args=None):
    rclpy.init(args=args)
    laser_publisher = LaserPublisher()
    try:
        rclpy.spin(laser_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        laser_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()