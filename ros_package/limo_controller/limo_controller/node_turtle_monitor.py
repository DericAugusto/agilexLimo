#!/usr/bin/env python3
# coding=UTF-8
"""
node_turtle_monitor.py

This module contains the TurtleMonitor class which is a ROS2 node for monitoring 
and controlling a turtle in the turtlesim simulator. It subscribes to the 
'linear_velocity' and 'steering_angle' topics to control the turtle's movement 
and to the '/turtle1/pose' topic to monitor the turtle's position.

Classes:
--------
TurtleMonitor(Node): A ROS2 node for monitoring and controlling a turtle in the 
turtlesim simulator.

Methods:
--------
__init__(): Constructor that initializes the node, sets up the subscribers for 
'linear_velocity', 'steering_angle', and '/turtle1/pose' topics, and the 
publisher for '/turtle1/cmd_vel' topic.

velocity_callback(msg): Updates the linear velocity based on the received 
message.

steering_callback(msg): Updates the steering angle based on the received 
message.

pose_callback(pose: Pose): Handles the received pose of the turtle.
"""
import rclpy  
from rclpy.node import Node
from std_msgs.msg import Float32
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist


class TurtleMonitor(Node):
  def __init__(self):
    super().__init__('turtle_monitor')
    self.linear_velocity = 0.0
    self.steering_angle = 0.0
    
    # subscribing to the vehicle commands
    self.linear_velocity_subscriber = self.create_subscription(
      Float32, "/linear_velocity", self.velocity_callback, 10)
    self.steering_angle_subscriber = self.create_subscription(
      Float32, "/steering_angle", self.steering_callback, 10)
    
    # subscriber to know turtle position and do the callback
    self.pose_subscriber = self.create_subscription(
      Pose, "/turtle1/pose", self.pose_callback, 10) 
    
    # publishing to turtlesim
    self.cmd_vel_pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)  
    
  def velocity_callback(self, msg):
    self.linear_velocity = msg.data

  def steering_callback(self, msg):
    self.steering_angle = msg.data
    
  def pose_callback(self, pose: Pose):
    cmd = Twist() 
    
    # logging the turtle position, linear velocity, and steering angle
    self.get_logger().info(
      "\nturtle position : (" + str(round(pose.x, 3)) + ", " + str(round(pose.y, 3)) + ")\n" +
      "linear velocity of vehicle : " + str(self.linear_velocity) + "\n" +
      "steering angle of vehicle : " + str(self.steering_angle) + "\n"
    )
    
    # publishing the vehicle commands
    cmd.linear.x = self.linear_velocity
    cmd.angular.z = self.steering_angle
    
    # publishing to turtlesim
    self.cmd_vel_pub.publish(cmd)

def main(args=None):
  rclpy.init(args=args)
  
  turtle_monitor_node = TurtleMonitor()
  print("\nInitializing the turtle monitor node...\n")
  rclpy.spin(turtle_monitor_node)
   
  turtle_monitor_node.destroy_node()
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()