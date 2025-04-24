#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

class PoseToOdomNode(Node):
    def __init__(self):
        super().__init__('pose_to_odometry_converter')
        # Create a publisher for Odometry messages on the 'odom' topic.
        self.odom_pub = self.create_publisher(Odometry, 'odom_from_slam_pose', 10)
        # Create a subscription to the PoseStamped messages.
        # Change 'pose' to your actual topic name if needed.
        self.create_subscription(PoseStamped, 'robot_pose_slam', self.pose_callback, 10)
        self.get_logger().info("Pose to Odometry node has started.")

    def pose_callback(self, msg: PoseStamped):
        # Create an Odometry message and transfer the pose information.
        odom = Odometry()
        # Use the same header as the incoming message.
        odom.header = msg.header
        # Set the child frame id (update this if needed).
        odom.child_frame_id = "base_link"
        # Copy the pose over.
        odom.pose.pose = msg.pose

        odom.pose.covariance = [
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        ]

        # these need to all change to what the wheelchair twist actually is
        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = 0.0

        odom.twist.covariance = [
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            ]

        self.odom_pub.publish(odom)
        self.get_logger().info("Published odometry message.")

def main(args=None):
    rclpy.init(args=args)
    node = PoseToOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
