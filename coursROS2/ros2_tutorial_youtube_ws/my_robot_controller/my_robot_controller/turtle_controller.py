#!/usr/bin/env python3
# above is the interpretor line, which tells ROS that this is a Python script
import rclpy  
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen
from functools import partial
"""
This script creates a ROS2 node that controls a turtle's movement in the 
turtlesim package. It does this by subscribing to the turtle's pose and 
publishing velocity commands based on the turtle's current position.

Classes:
--------
TurtleController - A ROS2 node that controls a turtle's movement.

Methods:
--------
__init__ - Initializes the node, sets up a subscription to the turtle's pose, 
and a publisher for velocity commands. It also logs a message indicating that 
the node has started.

pose_callback - This method is triggered whenever a new pose message is received
from the turtle's pose topic. It evaluates the turtle's current position. If 
the turtle is outside a defined square 
(x > 9.0 or x < 2.0 or y > 9.0 or y < 2.0), it publishes a velocity command to 
make the turtle turn right and move forward, effectively guiding it back 
towards the defined square. If the turtle is inside the square, it publishes a 
velocity command to make the turtle move straight ahead at a faster speed, 
allowing it to roam freely within the defined area.
"""



class TurtleController(Node):
  def __init__(self):
    super().__init__('turtle_controller')
    
    # to indicates if the turtle is on the left or right side
    self.previous_x = 0 
  
    # create a Subscriber - (data type, topic name, callback function, queue size)
    self.pose_subscriber = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10) 
    # create a Publisher - (data type, topic name, queue size)
    self.cmd_vel_pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)  
    
    self.get_logger().info('Turtle Controller node has been started')
    
    
  def pose_callback(self, pose: Pose):
    cmd = Twist() # message from the class Twist
    
    # if the turtle is outside the square, turn right
    if pose.x > 9.0 or pose.x < 2.0 or pose.y > 9.0 or pose.y < 2.0:
      cmd.linear.x = 1.0
      cmd.angular.z = 0.9
    else:
      cmd.linear.x = 5.0
      cmd.angular.z = 0.0
    
    self.cmd_vel_pub.publish(cmd)
    
    # if the turtle passes the middle, its trace'll change colors
    # self.previous_x is used to reduce the number of service calls
    if pose.x > 5.5 and self.previous_x <= 5.5:
      self.previous_x = pose.x
      self.get_logger().info("color changed to red")
      self.call_set_pen_service(255, 0, 0, 3, False) 
    elif pose.x <= 5.5 and self.previous_x > 5.5:
      self.previous_x = pose.x
      self.get_logger().info("color changed to green")
      self.call_set_pen_service(0, 255, 0, 3, False) 
    
  # service client to set the pen color
  def call_set_pen_service(self, r, g, b, width, off):
    client = self.create_client(SetPen, "/turtle1/set_pen")
    
    while not client.wait_for_service(1.0):
      self.get_logger().warn('Waiting for service /turtle1/set_pen to become available')
    
    # fill in the request data structure
    request = SetPen.Request()
    request.r = r
    request.g = g
    request.b = b
    request.width = width
    request.off = off
    
    # call the service without blocking the thread
    future = client.call_async(request) 
    future.add_done_callback(partial(self.callback_set_pen))
    
  # callback function for the service client
  def callback_set_pen(self, future):
    try:
      response = future.result()
      #self.get_logger().info('Pen color has been changed')
    except Exception as e:
      self.get_logger().error('Service call failed: %r' % (e,))
    

def main(args=None):
  rclpy.init(args=args)
  
  node = TurtleController()
  rclpy.spin(node)
  
  rclpy.shutdown()