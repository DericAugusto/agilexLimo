#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import tkinter as tk
import threading

class VehicleControlNode(Node):
  def __init__(self):
    super().__init__('interface_controller')
    self.linear_vel_publisher = self.create_publisher(Float32, 'linear_velocity', 10)
    self.steering_angle_publisher = self.create_publisher(Float32, 'steering_angle', 10)
    self.linear_vel = 0.0
    self.steering_angle = 0.0

  def publish_control_command(self):
    linear_vel_msg = Float32()
    linear_vel_msg.data = self.linear_vel
    self.linear_vel_publisher.publish(linear_vel_msg)

    steering_angle_msg = Float32()
    steering_angle_msg.data = self.steering_angle
    self.steering_angle_publisher.publish(steering_angle_msg)

  def update_linear_velocity(self, vel):
    self.linear_vel = vel
    self.publish_control_command()

  def update_steering_angle(self, angle):
    self.steering_angle = angle
    self.publish_control_command()

def create_interface(node):
  root = tk.Tk()
  root.title("Vehicle Control Interface")

  # Buttons for controlling linear velocity
  tk.Button(root, text="Increase Speed", command=lambda: node.update_linear_velocity(0.5)).pack()
  tk.Button(root, text="Stop", command=lambda: node.update_linear_velocity(0.0)).pack()

  # Buttons for controlling steering angle
  tk.Button(root, text="Steer Left", command=lambda: node.update_steering_angle(-10.0)).pack()
  tk.Button(root, text="Straight", command=lambda: node.update_steering_angle(0.0)).pack()
  tk.Button(root, text="Steer Right", command=lambda: node.update_steering_angle(10.0)).pack()

  return root

def main(args=None):
  rclpy.init(args=args)
  vehicle_control_node = VehicleControlNode()
  gui = create_interface(vehicle_control_node)

  # Run the Tkinter event loop in a separate thread
  gui_thread = threading.Thread(target=gui.mainloop, daemon=True)
  gui_thread.start()

  print("ROS2 Node and Tkinter GUI started")
  rclpy.spin(vehicle_control_node)

  vehicle_control_node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()
