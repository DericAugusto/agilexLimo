#!/usr/bin/env python3
"""
node_clicking_controller.py

This module contains the VehicleControlNode class which is a ROS2 node for 
controlling a vehicle using a graphical user interface.
It uses the tkinter library to create the interface and publishes the linear 
velocity and steering angle commands
to the 'linear_velocity' and 'steering_angle' topics respectively.

Classes:
--------
VehicleControlNode(Node): A ROS2 node for controlling a vehicle using a 
graphical user interface.

Methods:
--------
publish_control_command(): Publishes the current linear velocity and steering 
angle as Float32 messages.
increment_linear_velocity(increment): Increments the linear velocity by the 
given amount and publishes the updated command.
increment_steering_angle(increment): Increments the steering angle by the given
amount and publishes the updated command.
"""
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import tkinter as tk
import threading


class VehicleControlNode(Node):
  def __init__(self):
    super().__init__('clicking_controller')
    self.linear_vel_publisher = self.create_publisher(
      Float32, 'linear_velocity', 10
    )
    self.steering_angle_publisher = self.create_publisher(
      Float32, 'steering_angle', 10
    )
    self.linear_vel = 0.0
    self.steering_angle = 0.0

  def publish_control_command(self):
    linear_vel_msg = Float32()
    linear_vel_msg.data = self.linear_vel
    self.linear_vel_publisher.publish(linear_vel_msg)

    steering_angle_msg = Float32()
    steering_angle_msg.data = self.steering_angle
    self.steering_angle_publisher.publish(steering_angle_msg)

  def increment_linear_velocity(self, increment):
    self.linear_vel += increment
    self.publish_control_command()

  def increment_steering_angle(self, increment):
    self.steering_angle += increment
    self.publish_control_command()
  
  def update_linear_velocity(self, vel):
    self.linear_vel = vel
    self.publish_control_command()

  def update_steering_angle(self, angle):
    self.steering_angle = angle
    self.publish_control_command()
    
  def stop_vehicle(self):
    self.update_linear_velocity(0.0)
    self.update_steering_angle(0.0)

def create_interface(node):
  root = tk.Tk()
  root.title("Vehicle Control Interface")
  
  root.geometry('100x50') # -> set the window size here

  # Get the directory where the script is located
  script_dir = os.path.dirname(os.path.realpath(__file__))

  # Load images for buttons and subsample them to make them smaller
  # -> adjust the subsample values here
  forward_img = tk.PhotoImage(
    file = os.path.join(script_dir, 'imgs/up.png')).subsample(5,5)
  backward_img = tk.PhotoImage(
    file = os.path.join(script_dir, 'imgs/down.png')).subsample(5,5)
  left_img = tk.PhotoImage(
    file = os.path.join(script_dir, 'imgs/left.png')).subsample(5,5)
  right_img = tk.PhotoImage(
    file = os.path.join(script_dir, 'imgs/right.png')).subsample(5,5)
  stop_img = tk.PhotoImage(
    file = os.path.join(script_dir, 'imgs/middle.png')).subsample(5,5)

  # Create buttons using the images
  button_up = tk.Button(
    root, image=forward_img, 
    command = lambda: node.increment_linear_velocity(0.1)
  )
  button_up.image = forward_img  # to solve garbage collection problem
  button_down = tk.Button(
    root, image=backward_img, 
    command = lambda: node.increment_linear_velocity(-0.1)
  )
  button_down.image = backward_img  # Keep a reference!
  button_left = tk.Button(
    root, image=left_img, 
    command = lambda: node.increment_steering_angle(0.1)
  )
  button_left.image = left_img  # Keep a reference!
  button_right = tk.Button(
    root, image=right_img, 
    command = lambda: node.increment_steering_angle(-0.1)
  )
  button_right.image = right_img  # Keep a reference!
  button_stop = tk.Button(
    root, image=stop_img, 
    command = node.stop_vehicle
  )
  button_stop.image = stop_img  # Keep a reference!

  # Arrange buttons in a grid that matches the arrow keys layout
  button_up.grid(row=0, column=1)
  button_left.grid(row=1, column=0)
  button_stop.grid(row=1, column=1)
  button_right.grid(row=1, column=2)
  button_down.grid(row=2, column=1)

  return root

def run_tkinter(node):
  gui = create_interface(node)
  gui.mainloop()

def main(args=None):
  rclpy.init(args=args)
  vehicle_control_node = VehicleControlNode()

  # Run ROS2 node in a separate daemon thread
  node_thread = threading.Thread(
  target = rclpy.spin, args=(vehicle_control_node,), daemon=True)
  node_thread.start()

  # Create and run the Tkinter GUI in the main thread
  print("ROS2 Node and Tkinter GUI started")
  gui = create_interface(vehicle_control_node)
  gui.mainloop()  # This will block until the GUI window is closed

  # After closing Tkinter GUI, 
  # shutdown ROS and wait for the node thread to finish
  rclpy.shutdown()
  node_thread.join()

if __name__ == '__main__':
  main()
