#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MyNode(Node):
  def __init__(self):
      super().__init__("sensors_publisher_node")
      self.get_logger().info('Running sensors_publisher_node...')

def main(args=None):
    rclpy.init(args=args) # initialize ROS2 communication

    node = MyNode() # create a node
    rclpy.spin(node) # keep the node running until it's stopped

    rclpy.shutdown()

if __name__ == '__main__':
    main()