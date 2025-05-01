#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class RectifyNode(Node):
    def __init__(self):
        super().__init__('rectify_node')
        self.get_logger().info('RectifyNode has been started.')

def main(args=None):
    rclpy.init(args=args)
    node = RectifyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()