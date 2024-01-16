#!/usr/bin/env python3
# coding=UTF-8
"""
This Python script is designed for controlling Agilex_Limo using ROS 2 
(Robot Operating System 2).
It subscribes to two ROS topics, /linear_velocity and /steering_angle, to 
receive control commands for the LIMO device.
The script uses the 'pylimo' package to interface with the LIMO hardware or 
simulation.

Classes:
- ApplyControl: A ROS node class that handles the control logic and interaction 
with the LIMO device.

main: The main function initializes the ROS node, creates an instance of LIMO,
and spins the ApplyControl node.

"""
import rclpy  
from rclpy.node import Node
from std_msgs.msg import Float32
from pylimo import limo

class ApplyControl(Node):
  def __init__(self, limo_instance):
    super().__init__('apply_control')
    super().init('apply_control')
    self.limo = limo_instance
    self.linear_velocity = 0.0
    self.steering_angle = 0.0
    self.linear_velocity_subscriber = self.create_subscription(
      Float32, "/linear_velocity", self.velocity_callback, 10)
    self.steering_angle_subscriber = self.create_subscription(
      Float32, "/steering_angle", self.steering_callback, 10)
  
  def velocity_callback(self, msg):
    self.linear_velocity = msg.data
    self.send_motion_command()

  def steering_callback(self, msg):
    self.steering_angle = msg.data
    self.send_motion_command()
      
  def send_motion_command(self):
    self.limo.SetMotionCommand(
      linear_vel = self.linear_velocity,
      angular_vel = self.steering_angle)
    
  
def main(args=None):
  rclpy.init(args=args)
  
  limo_instance = limo.LIMO()
  limo_instance.EnableCommand()
  
  apply_control_node = ApplyControl(limo_instance)
  print("The commands are being applied to the vehicle!")
  rclpy.spin(apply_control_node)
  
  apply_control_node.destroy_node()
  rclpy.shutdown()