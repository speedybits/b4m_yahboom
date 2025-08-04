#!/usr/bin/env python3

"""
MQTT Navigation Test Script for SLAM Testing
Tests automated navigation using MQTT commands
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import paho.mqtt.client as mqtt
import json
import time
import math
import threading

class MQTTNavigationTest(Node):
    def __init__(self):
        super().__init__('mqtt_navigation_test')
        
        # MQTT settings
        self.mqtt_broker = "192.168.68.111"
        self.mqtt_port = 1883
        self.mqtt_username = "robot"
        self.mqtt_password = "robot123"
        
        # MQTT topics
        self.command_topic = "yahboom/navigation/command"
        self.status_topic = "yahboom/navigation/status"
        self.info_topic = "yahboom/navigation/info"
        
        # ROS subscriptions
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Navigation state
        self.current_odom = None
        self.current_test = 0
        self.test_start_time = None
        self.navigation_status = "idle"
        
        # Test waypoints (relative to start position)
        self.test_waypoints = [
            {"name": "test_point_1", "x": 0.5, "y": 0.5, "tolerance": 0.1},
            {"name": "test_point_2", "x": -0.5, "y": 0.5, "tolerance": 0.1},
            {"name": "test_point_3", "x": 0.0, "y": 0.0, "tolerance": 0.1}  # Return to start
        ]
        
        # Results tracking
        self.results = {
            'tests_completed': 0,
            'tests_successful': 0,
            'start_position': None,
            'waypoint_results': [],
            'mqtt_connected': False,
            'navigation_working': False,
            'total_test_time': 0.0,
            'overall_success': False
        }
        
        # MQTT client
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.username_pw_set(self.mqtt_username, self.mqtt_password)
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        
        # Connect to MQTT broker
        self.connect_mqtt()
        
        self.get_logger().info("MQTT Navigation Test initialized")
        
    def odom_callback(self, msg):
        self.current_odom = msg
        if self.results['start_position'] is None:
            self.results['start_position'] = {
                'x': msg.pose.pose.position.x,
                'y': msg.pose.pose.position.y
            }
            self.get_logger().info(f"Start position recorded: x={msg.pose.pose.position.x:.3f}, y={msg.pose.pose.position.y:.3f}")
    
    def connect_mqtt(self):
        """Connect to MQTT broker"""
        try:
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, 60)
            self.mqtt_client.loop_start()
            self.get_logger().info(f"Connecting to MQTT broker: {self.mqtt_broker}:{self.mqtt_port}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to MQTT broker: {e}")
    
    def on_mqtt_connect(self, client, userdata, flags, rc):
        """MQTT connection callback"""
        if rc == 0:
            self.results['mqtt_connected'] = True
            self.get_logger().info("Connected to MQTT broker successfully")
            
            # Subscribe to status and info topics
            client.subscribe(self.status_topic)
            client.subscribe(self.info_topic)
            self.get_logger().info(f"Subscribed to {self.status_topic} and {self.info_topic}")
        else:
            self.get_logger().error(f"Failed to connect to MQTT broker, return code {rc}")
    
    def on_mqtt_message(self, client, userdata, msg):
        """MQTT message callback"""
        try:
            topic = msg.topic
            payload = msg.payload.decode('utf-8')
            
            if topic == self.status_topic:
                # Parse navigation status
                try:
                    status_data = json.loads(payload)
                    self.navigation_status = status_data.get('status', 'unknown')
                    self.get_logger().info(f"Navigation status: {self.navigation_status}")
                    
                    # Check if current waypoint was reached
                    if self.navigation_status == "goal_reached" and self.current_test > 0:
                        self.handle_waypoint_reached()
                        
                except json.JSONDecodeError:
                    self.navigation_status = payload
                    
            elif topic == self.info_topic:
                self.get_logger().info(f"Navigation info: {payload}")
                
        except Exception as e:
            self.get_logger().error(f"Error processing MQTT message: {e}")
    
    def send_navigation_command(self, waypoint):
        """Send navigation command via MQTT"""
        if not self.results['mqtt_connected']:
            self.get_logger().error("Cannot send command - MQTT not connected")
            return False
        
        # Calculate absolute position from relative waypoint
        if self.results['start_position'] is None:
            self.get_logger().error("Cannot send command - start position not recorded")
            return False
        
        abs_x = self.results['start_position']['x'] + waypoint['x']
        abs_y = self.results['start_position']['y'] + waypoint['y']
        
        command = {
            "command": "goto",
            "waypoint_id": waypoint['name'],
            "position": {
                "x": abs_x,
                "y": abs_y
            },
            "orientation": {
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
                "w": 1.0
            }
        }
        
        try:
            command_json = json.dumps(command)
            result = self.mqtt_client.publish(self.command_topic, command_json)
            
            if result.rc == mqtt.MQTT_ERR_SUCCESS:
                self.get_logger().info(f"Sent navigation command to {waypoint['name']}: ({abs_x:.2f}, {abs_y:.2f})")
                return True
            else:
                self.get_logger().error(f"Failed to publish MQTT command, return code: {result.rc}")
                return False
                
        except Exception as e:
            self.get_logger().error(f"Error sending navigation command: {e}")
            return False
    
    def calculate_distance_to_target(self, waypoint):
        """Calculate current distance to target waypoint"""
        if not self.current_odom or not self.results['start_position']:
            return float('inf')
        
        target_x = self.results['start_position']['x'] + waypoint['x']
        target_y = self.results['start_position']['y'] + waypoint['y']
        
        current_x = self.current_odom.pose.pose.position.x
        current_y = self.current_odom.pose.pose.position.y
        
        dx = current_x - target_x
        dy = current_y - target_y
        
        return math.sqrt(dx*dx + dy*dy)
    
    def handle_waypoint_reached(self):
        """Handle successful waypoint completion"""
        if self.current_test > 0 and self.current_test <= len(self.test_waypoints):
            waypoint = self.test_waypoints[self.current_test - 1]
            distance = self.calculate_distance_to_target(waypoint)
            
            waypoint_result = {
                'waypoint': waypoint['name'],
                'target_position': waypoint,
                'final_distance': distance,
                'success': distance <= waypoint['tolerance'],
                'time_taken': time.time() - self.test_start_time if self.test_start_time else 0
            }
            
            self.results['waypoint_results'].append(waypoint_result)
            
            if waypoint_result['success']:
                self.results['tests_successful'] += 1
                self.get_logger().info(f"Waypoint {waypoint['name']} reached successfully (distance: {distance:.3f}m)")
            else:
                self.get_logger().warning(f"Waypoint {waypoint['name']} not reached within tolerance (distance: {distance:.3f}m)")
            
            # Start next test after brief delay
            threading.Timer(2.0, self.start_next_test).start()
    
    def start_next_test(self):
        """Start the next navigation test"""
        if self.current_test < len(self.test_waypoints):
            waypoint = self.test_waypoints[self.current_test]
            self.current_test += 1
            
            self.get_logger().info(f"Starting test {self.current_test}/{len(self.test_waypoints)}: {waypoint['name']}")
            
            if self.send_navigation_command(waypoint):
                self.test_start_time = time.time()
                self.results['navigation_working'] = True
            else:
                self.get_logger().error(f"Failed to send command for test {self.current_test}")
        else:
            # All tests completed
            self.complete_testing()
    
    def complete_testing(self):
        """Complete the navigation testing sequence"""
        self.results['tests_completed'] = len(self.results['waypoint_results'])
        
        # Calculate overall success
        success_rate = self.results['tests_successful'] / max(1, self.results['tests_completed'])
        self.results['overall_success'] = (
            self.results['mqtt_connected'] and
            self.results['navigation_working'] and
            success_rate >= 0.8  # At least 80% of tests must succeed
        )
        
        # Save results
        self.save_results()
        
        self.get_logger().info("MQTT Navigation testing completed")
        self.get_logger().info(f"Tests successful: {self.results['tests_successful']}/{self.results['tests_completed']}")
        self.get_logger().info(f"Overall success: {self.results['overall_success']}")
        
        # Shutdown after brief delay
        threading.Timer(3.0, lambda: rclpy.shutdown()).start()
    
    def run_test_sequence(self):
        """Run the complete MQTT navigation test sequence"""
        self.get_logger().info("Starting MQTT navigation test sequence...")
        
        # Wait for initial setup
        timeout = 10.0
        start_time = time.time()
        
        while (not self.results['mqtt_connected'] or 
               self.results['start_position'] is None) and \
              (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if not self.results['mqtt_connected']:
            self.get_logger().error("MQTT connection failed - cannot run navigation tests")
            return False
        
        if self.results['start_position'] is None:
            self.get_logger().error("Start position not available - cannot run navigation tests")
            return False
        
        # Start first test
        self.start_next_test()
        
        return True
    
    def save_results(self):
        """Save test results to file"""
        results_file = '/tmp/mqtt_navigation_test_results.json'
        
        try:
            with open(results_file, 'w') as f:
                json.dump(self.results, f, indent=2)
            
            self.get_logger().info(f"Test results saved to {results_file}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to save test results: {e}")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        tester = MQTTNavigationTest()
        
        if tester.run_test_sequence():
            # Spin until testing is complete
            rclpy.spin(tester)
        
        # Print final results
        print(f"\n{'='*50}")
        print("MQTT NAVIGATION TEST RESULTS")
        print(f"{'='*50}")
        print(f"MQTT connected: {tester.results['mqtt_connected']}")
        print(f"Navigation working: {tester.results['navigation_working']}")
        print(f"Tests completed: {tester.results['tests_completed']}")
        print(f"Tests successful: {tester.results['tests_successful']}")
        print(f"Overall success: {tester.results['overall_success']}")
        print(f"{'='*50}")
        
        return 0 if tester.results['overall_success'] else 1
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"MQTT navigation test failed: {e}")
        return 1
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    exit(main())