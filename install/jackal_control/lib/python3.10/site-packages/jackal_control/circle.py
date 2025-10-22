#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import argparse

class CircleMover(Node):
    def __init__(self, radius):
        super().__init__('circle_mover')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.radius = radius
        self.speed = 0.2  # Linear speed in m/s
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = self.speed
        msg.angular.z = self.speed / self.radius  # ω = v / r
        self.publisher_.publish(msg)

def main():
    parser = argparse.ArgumentParser(description='Move robot in a circle.')
    parser.add_argument('--radius', type=float, required=True, help='Radius of the circle in meters')
    args = parser.parse_args()

    rclpy.init()
    node = CircleMover(radius=args.radius)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot before shutting down
        stop_msg = Twist()
        node.publisher_.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
