import tkinter as tk
from PIL import Image, ImageTk
import os
import numpy as np
from queue import PriorityQueue
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import threading

class ControlTrajectoryNode(Node):
  def __init__(self):
    super().__init__('trajectory_controller')
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


class MapMarkerApp():
  def __init__(self, root, matrix, control_node):
    self.root = root
    self.root.title("Map Marker")
    self.matrix = matrix
    self.animation_running = False
    self.animation_paused = False
    self.current_path_index = 0
    self.vehicle = None
    self.path = []
    self.control_node = control_node

    script_dir = os.path.dirname(os.path.realpath(__file__))
    self.image = Image.open(script_dir + "/imgs/track.png")
    self.photo = ImageTk.PhotoImage(self.image)
    self.setup_canvas()
    self.create_buttons()
    self.points = []
    self.canvas.bind("<Button-1>", self.mark_point) 
    self.frame_delay = 0.01
    
    
  def setup_canvas(self):
    self.canvas = tk.Canvas(self.root, width=self.photo.width(), 
                            height=self.photo.height())
    self.canvas.pack()
    self.canvas.create_image(0, 0, image=self.photo, anchor=tk.NW)


  def create_buttons(self):
    self.linear_button = tk.Button(self.root, text="Linear Line",
                                   command=lambda: self.set_mode('linear'))
    self.linear_button.pack(side=tk.TOP, fill=tk.X)

    self.street_button = tk.Button(self.root, text="Street Line",
                                   command=lambda: self.set_mode('street'))
    self.street_button.pack(side=tk.TOP, fill=tk.X)

    self.start_button = tk.Button(self.root, text="Start",
                                  command=self.start_animation)
    self.start_button.pack(side=tk.TOP, fill=tk.X)

    self.stop_button = tk.Button(self.root, text="Stop",
                                 command=self.stop_animation)
    self.stop_button.pack(side=tk.TOP, fill=tk.X)

    self.reset_button = tk.Button(self.root, text="Reset",
                                  command=self.reset)
    self.reset_button.pack(side=tk.TOP, fill=tk.X)


  def start_animation(self):
    self.animation_running = True
    self.animation_paused = False
    if self.mode == 'linear':
      self.animate_vehicle(self.points, 1)
    elif self.mode == 'street':
      self.animate_vehicle(self.path, 1)


  def stop_animation(self):
    self.animation_paused = True


  def reset(self):
    self.canvas.delete("all")
    self.canvas.create_image(0, 0, image=self.photo, anchor=tk.NW)
    self.points.clear()
    self.mode = 'linear'
    self.animation_running = False
    self.animation_paused = False
    self.current_path_index = 0
    self.path.clear()
    if self.vehicle:
      self.canvas.delete(self.vehicle)
      self.vehicle = None
    
    
  def set_mode(self, mode):
    self.mode = mode
    print(f"Mode set to: {self.mode}")


  def mark_point(self, event):
    if self.matrix[event.y, event.x] != 0:
      if len(self.points) < 2:
        color = 'blue' if len(self.points) == 0 else 'red'
        self.plot_point((event.x, event.y), color)
        self.points.append((event.x, event.y))

        if len(self.points) == 2:
          if self.mode == 'linear':
            self.draw_linear_line()
          elif self.mode == 'street':
            self.draw_street_line()
   
            
  def plot_point(self, coordinate, color='red', label=None):
    x, y = coordinate
    radius = 5
    oval_id = self.canvas.create_oval(
      x - radius, y - radius, x + radius, y + radius, fill=color
    )
    if label:
      self.canvas.create_text(
        x, y + radius + 10, text=label, fill=color
      )
    return oval_id


  def draw_linear_line(self):
    self.canvas.create_line(self.points[0][0], self.points[0][1],
                            self.points[1][0], self.points[1][1],
                            fill='red', width=2)
    print("Linear line drawn between:", self.points)
    self.animate_vehicle(self.points, 1)


  def draw_street_line(self):
    # Clear previous street lines
    self.canvas.delete("street_line")

    # Assume points are given in (x, y) format and convert to (row, col)
    start_point = (self.points[0][1], self.points[0][0])
    goal_point = (self.points[1][1], self.points[1][0])

    # Find closest '2' points in the matrix
    start_closest = find_closest_two(self.matrix, start_point)
    goal_closest = find_closest_two(self.matrix, goal_point)

    # Perform A* search
    path = a_star_search(self.matrix, start_closest, goal_closest)
    
    # If a path is found, draw it on the canvas
    if path:
      # Plot the points
      self.plot_point(start_closest, 'green', 'start_inline_point')
      self.plot_point(goal_closest, 'green', 'arrival_inline_point')
      
      # Draw the path
      for i in range(len(path) - 1):
        point1 = path[i]
        point2 = path[i + 1]
        self.canvas.create_line(
          point1[0], point1[1],  # Adjust for pixel position if needed
          point2[0], point2[1],
          fill='red', width=2, tags=("point")
        )
      print("Street line drawn using A* between:", start_closest, goal_closest)

    if path:
      self.animate_vehicle(path, 1)


  def animate_vehicle(self, path, linear_velocity):
    # Load and resize vehicle image
    script_dir = os.path.dirname(os.path.realpath(__file__))
    vehicle_img = Image.open(script_dir + "/imgs/vehicle.png")
    new_size = (85, 110)  # Adjust size as needed
    vehicle_img_resized = vehicle_img.resize(new_size, Image.ANTIALIAS)
    vehicle_photo = ImageTk.PhotoImage(vehicle_img_resized)

    # Create or update vehicle image on canvas
    if not self.vehicle:
      self.vehicle = self.canvas.create_image(
        path[0][0], path[0][1], image=vehicle_photo)
      self.canvas.image = vehicle_photo  # Prevent garbage collection
    else:
      self.canvas.itemconfig(self.vehicle, image=vehicle_photo)

    # Start or resume animation
    self.animation_running = True

    for i in range(self.current_path_index, len(path)):
      if not self.animation_running or self.animation_paused:
        self.current_path_index = i  # Save current index for resuming
        break

      # Calculate the distance and direction to the next point
      dx = path[i][0] - self.canvas.coords(self.vehicle)[0]
      dy = path[i][1] - self.canvas.coords(self.vehicle)[1]
      distance = (dx**2 + dy**2)**0.5
      steps = max(int(distance / (linear_velocity / 0.005 * self.frame_delay)), 1)
      
      for _ in range(steps):
        if not self.animation_running or self.animation_paused:
          self.current_path_index = i
          return
        self.canvas.move(self.vehicle, dx / steps, dy / steps)
        self.canvas.update()
        time.sleep(self.frame_delay)
        
    # TODO: Implement control logic here
    # self.control_node.increment_linear_velocity(0.1) 
    # self.control_node.update_linear_velocity(calculated_linear_vel)
    # self.control_node.increment_steering_angle(calculated_steering_angle) 
    # self.control_node.update_steering_angle(calculated_steering_angle)

    # Reset the current path index if the end of the path is reached
    if self.current_path_index >= len(path):
      self.current_path_index = 0

    
def find_closest_two(matrix, coord):
  # Validate input
  if not (0 <= coord[1] < len(matrix) and 0 <= coord[0] < len(matrix[0])):
    raise ValueError("Given coordinate is out of matrix bounds")

  min_distance = float('inf')
  closest_coord = None

  # Iterate through the matrix to find the closest '2'
  for i in range(len(matrix)):
    for j in range(len(matrix[i])):
      if matrix[i][j] == 2:
        # The difference here is that we consider 'i' as y and 'j' as x
        distance = np.linalg.norm(np.array([j, i]) - np.array(coord))
        if distance < min_distance:
          min_distance = distance
          closest_coord = (j, i)  # Notice the order of j, i

  return closest_coord
  
  
def a_star_search(matrix, start, goal):
  neighbors = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # 4-way connectivity

  def heuristic(a, b):
    return np.linalg.norm(np.array(a) - np.array(b))

  frontier = PriorityQueue()
  frontier.put((0, start))
  came_from = {start: None}
  cost_so_far = {start: 0}

  while not frontier.empty():
    _, current = frontier.get()

    if current == goal:
      break

    for dx, dy in neighbors:
      next = (current[0] + dx, current[1] + dy)
      if 0 <= next[0] < len(matrix) and 0 <= next[1] < len(matrix[0]):
        if matrix[next[1]][next[0]] == 2:  # Check if the value is 2
          new_cost = cost_so_far[current] + 1
          if next not in cost_so_far or new_cost < cost_so_far[next]:
            cost_so_far[next] = new_cost
            priority = new_cost + heuristic(goal, next)
            frontier.put((priority, next))
            came_from[next] = current

  # Reconstruct path
  current = goal
  path = []
  while current != start:
    path.append(current)
    current = came_from[current]
  path.append(start)  # optional
  path.reverse()  # optional
  return path


def main():
  # Initialize ROS2 node
  rclpy.init(args=args)
  control_trajectory_node = ControlTrajectoryNode()
  
  # Run ROS2 node in a separate daemon thread
  node_thread = threading.Thread(
  target = rclpy.spin, args=(control_trajectory_node,), daemon=True)
  node_thread.start()
  
  # Get the directory where the script is located
  script_dir = os.path.dirname(os.path.realpath(__file__))

  # Load the matrix from the .npy file
  matrix_path = script_dir + "/data/track_matrix.npy"
  matrix = np.load(matrix_path)

  # Start the Tkinter event loop
  root = tk.Tk()
  app = MapMarkerApp(root, matrix, control_trajectory_node)
  root.mainloop()

  # After closing Tkinter GUI, 
  # shutdown ROS and wait for the node thread to finish
  rclpy.shutdown()
  node_thread.join()
  
if __name__ == '__main__':
  main()
