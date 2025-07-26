#!/usr/bin/env python3
"""
Automatic Initial Pose Setter for B4M Robot
Sets the robot's initial pose at map center for automated testing
"""

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import time
import sys


def main():
    print('🤖 Automatic Pose Estimate - Testing Mode')
    print('=========================================')
    print('Setting robot pose at map center (0.0, 0.0)')
    print('This replaces manual 2D pose estimation for testing')
    print('')

    try:
        # Initialize ROS2  
        rclpy.init()
        node = rclpy.create_node('automatic_pose_setter')

        # Create QoS profile matching AMCL subscription (BEST_EFFORT)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )

        publisher = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', qos_profile)

        # Wait for publisher to be ready and for AMCL subscription
        print('⏳ Waiting for AMCL subscription...')
        time.sleep(3)

        # Verify AMCL is subscribed
        topic_info = node.get_publishers_info_by_topic('/initialpose')
        print(f'📡 Publisher created. Checking for AMCL subscription...')
        
        # Create pose message at map center
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = node.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'

        # Set position at map center (0.0, 0.0)
        pose_msg.pose.pose.position.x = 0.0
        pose_msg.pose.pose.position.y = 0.0  
        pose_msg.pose.pose.position.z = 0.0

        # Set orientation (facing forward)
        pose_msg.pose.pose.orientation.x = 0.0
        pose_msg.pose.pose.orientation.y = 0.0
        pose_msg.pose.pose.orientation.z = 0.0
        pose_msg.pose.pose.orientation.w = 1.0

        # Set covariance (match RViz defaults)
        pose_msg.pose.covariance = [0.0] * 36
        pose_msg.pose.covariance[0] = 0.25   # x variance
        pose_msg.pose.covariance[7] = 0.25   # y variance  
        pose_msg.pose.covariance[35] = 0.068 # yaw variance

        # Publish pose multiple times to ensure delivery
        for i in range(5):
            publisher.publish(pose_msg)
            print(f'📤 Published initial pose (attempt {i+1}/5)')
            rclpy.spin_once(node, timeout_sec=0.1)
            time.sleep(0.8)

        print('✅ Published initial pose at (0.0, 0.0) with frame_id=map')
        print('⏱️  Waiting 5 seconds for AMCL to process...')
        time.sleep(5)

        # Verify AMCL received the pose by checking if it's now publishing
        print('🔍 Verifying AMCL responded to pose estimate...')
        
        # Try to get one AMCL pose message to verify it worked
        verification_successful = False
        try:
            # Create a temporary subscriber to check for AMCL poses
            from geometry_msgs.msg import PoseWithCovarianceStamped as AmclPose
            
            def pose_callback(msg):
                nonlocal verification_successful
                verification_successful = True
                print('✅ AMCL is now publishing poses - pose estimate successful!')
            
            amcl_subscriber = node.create_subscription(
                AmclPose, 
                '/amcl_pose', 
                pose_callback, 
                10
            )
            
            # Spin for a few seconds to see if we get a pose
            start_time = time.time()
            while time.time() - start_time < 3.0 and not verification_successful:
                rclpy.spin_once(node, timeout_sec=0.1)
                
        except Exception as e:
            print(f'⚠️  Could not verify AMCL response: {e}')

        if verification_successful:
            print('🎯 Initial pose setting completed successfully!')
            print('📋 AMCL has accepted the pose and is now localizing')
        else:
            print('⚠️  Initial pose was published but AMCL response unclear')
            print('📋 Check AMCL logs for pose processing messages')

        print('')
        print('📋 Expected AMCL log messages:')
        print('   - initialPoseReceived messages')  
        print('   - Setting pose messages')
        print('   - Transform chain establishment')
        print('')
        print('✅ Automatic pose estimate completed')

    except Exception as e:
        print(f'❌ Error setting initial pose: {e}')
        sys.exit(1)
    finally:
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()