#!/usr/bin/env python3
# above is the interpretor line, which tells ROS that this is a Python script
"""
This script creates a ROS2 node that subscribes to the pose of a turtle in the 
turtlesim package and logs the pose whenever it changes.

Classes:
--------
PoseSubscriber - A ROS2 node that subscribes to the pose of a turtle.

Methods:
--------
__init__ - Initializes the node and sets up a subscription to the pose of the 
turtle.
pose_callback - Logs the current pose of the turtle.

Functions:
----------
main - Initializes the ROS2 library, creates an instance of the PoseSubscriber 
node, spins the node so it can send and receive messages, and then shuts down 
the ROS2 library when the node is done.
"""
import rclpy  
from rclpy.node import Node
from turtlesim.msg import Pose


class PoseSubscriber(Node):
  def __init__(self):
    super().__init__('pose_subscriber')
    # (data type, topic name, callback function, queue size)
    self.pose_subscriber = self.create_subscription(
      Pose, "/turtle1/pose", self.pose_callback, 10)
    
  def pose_callback(self, msg: Pose):
    self.get_logger().info("(" + str(msg.x) + ", " + str(msg.y) + ")")
    
def main(args=None):
  rclpy.init(args=args)
  
  node = PoseSubscriber()
  rclpy.spin(node)
    
  rclpy.shutdown()