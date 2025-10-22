#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class OdomListener(Node):
    def __init__(self):
        super().__init__('odom_listener')
        # Create subscription to /odometry/filtered
        self.subscription = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.listener_callback,
            10  # QoS depth
        )
        self.subscription  # prevent unused variable warning
        self.get_logger().info("Subscribed to /odometry/filtered")

    def listener_callback(self, msg):
        # Print a few key values for clarity
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        self.get_logger().info(
            f"Position -> x: {pos.x:.2f}, y: {pos.y:.2f}, z: {pos.z:.2f} | "
            f"Orientation -> z: {ori.z:.2f}, w: {ori.w:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = OdomListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
