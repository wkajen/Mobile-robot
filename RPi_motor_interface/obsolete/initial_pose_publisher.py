#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.timer = self.create_timer(1.0, self.publish_initial_pose)
        self.published = False

    def publish_initial_pose(self):
        if self.published:
            return

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.orientation.w = 1.0

        msg.pose.covariance = [
            0.25, 0, 0, 0, 0, 0,
            0, 0.25, 0, 0, 0, 0,
            0, 0, 0.0, 0, 0, 0,
            0, 0, 0, 0.0, 0, 0,
            0, 0, 0, 0, 0.0, 0,
            0, 0, 0, 0, 0, 0.06853891945200942
        ]

        self.publisher_.publish(msg)
        self.get_logger().info('Initial pose published.')
        self.published = True


def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

