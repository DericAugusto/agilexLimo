#!/usr/bin/env python3
# above is the interpretor line, which tells ROS that this is a Python script
import rclpy  
from rclpy.node import Node
"""
This script creates a ROS2 node that logs a message every second.

Classes:
--------
MyNode - A ROS2 node that logs a message every second.

Methods:
--------
__init__ - Initializes the node, sets up a counter and a timer that calls the 
time_callback method every second.
time_callback - Logs a message and increments the counter.

Functions:
----------
main - Initializes the ROS2 library, creates an instance of the MyNode class, 
spins the node so it can send and receive messages, and then shuts down the ROS2
library when the node is done.
"""

class MyNode(Node):
  def __init__(self):
    super().__init__('first_node') # initialize the superclass
    self.counter = 0
    self.create_timer(1.0, self.time_callback)
    
  def time_callback(self):
    self.get_logger().info('Hello nº ' + str(self.counter)) # log a message
    self.counter += 1

def main(args=None):
  rclpy.init(args=args) # initialize ROS
  
  node = MyNode() # create an instance of our class
  rclpy.spin(node) # keep the program running until a shutdown signal is received
  
  
  
  rclpy.shutdown() # shutdown ROS

if __name__ == '__main__': 
  main() 