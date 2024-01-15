#!/usr/bin/env python3
import rclpy  
from rclpy.node import Node
from geometry_msgs.msg import Twist
"""
This script creates a ROS2 node that publishes velocity commands to a turtle in 
the turtlesim package to make it move in a circle.

Classes:
--------
DrawCircle - A ROS2 node that publishes velocity commands to a turtle.

Methods:
--------
__init__ - Initializes the node, publisher, and timer.
send_velocity_command - Publishes a velocity command to the turtle.

Functions:
----------
main - Initializes the ROS2 library, creates an instance of the DrawCircle node,
spins the node so it can send and receive messages, and then shuts down the ROS2
library when the node is done.
"""


class DrawCircle(Node):
  def __init__(self):
    super().__init__('draw_circle')
    # create a Publisher - (data type, topic name, queue size)
    # atention to be the exact same name and type
    self.cmd_vel_pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)  
    self.timer = self.create_timer(0.5, self.send_velocity_command)
    self.get_logger().info('Draw Circle node has been started')
    
  def send_velocity_command(self):
    msg = Twist () # message from the class Twist
    msg.linear.x = 2.0
    msg.angular.z = 1.0
    self.cmd_vel_pub.publish(msg)

def main(args=None):
  rclpy.init(args=args)
  node = DrawCircle()
  rclpy.spin(node)
  rclpy.shutdown()