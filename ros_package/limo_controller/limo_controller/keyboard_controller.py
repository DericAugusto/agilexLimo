#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from pynput import keyboard
from std_msgs.msg import Float32 

    
class VehicleControlNode(Node):
  def __init__(self):
    super().__init__('vehicle_control_node')
    # Publishers for linear velocity and steering angle
    self.linear_vel_publisher = self.create_publisher(Float32, 'linear_velocity', 10)
    self.steering_angle_publisher = self.create_publisher(Float32, 'steering_angle', 10)
    self.linear_vel = 0.0
    self.steering_angle = 0.0
    self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
    self.listener.start()
    self.timer = self.create_timer(0.1, self.publish_control_command)

  def publish_control_command(self):
    # Create and publish a Float32 message for linear velocity
    linear_vel_msg = Float32()
    linear_vel_msg.data = self.linear_vel
    self.linear_vel_publisher.publish(linear_vel_msg)
    
    # Create and publish a Float32 message for steering angle
    steering_angle_msg = Float32()
    steering_angle_msg.data = self.steering_angle
    self.steering_angle_publisher.publish(steering_angle_msg)

  def on_press(self, key):
    if key == keyboard.Key.up:
      self.linear_vel = 0.5
    elif key == keyboard.Key.right:
      self.steering_angle = min(self.steering_angle + 0.001, 10.0)
    elif key == keyboard.Key.left:
      self.steering_angle = max(self.steering_angle - 0.001, -10.0)

  def on_release(self, key):
    if key == keyboard.Key.up:
      self.linear_vel = 0.0
    elif key in (keyboard.Key.left, keyboard.Key.right):
      pass  # Keep the current steering angle value

def main(args=None):
  rclpy.init(args=args)
  
  vehicle_control_node = VehicleControlNode()
  print('\nReading from keyboard\n---------------------------\nUse\
  arrow keys to move the vehicle.')
  rclpy.spin(vehicle_control_node)
  vehicle_control_node.listener.stop()
  vehicle_control_node.destroy_node()
  
  rclpy.shutdown()

if __name__ == '__main__':
  main()