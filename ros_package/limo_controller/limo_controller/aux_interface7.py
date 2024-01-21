import tkinter as tk
from PIL import Image, ImageTk
import os
import numpy as np
from queue import PriorityQueue
import time

class MapMarkerApp:
  def __init__(self, root, matrix):
    self.root = root
    self.root.title("Map Marker")
    self.matrix = matrix
    
    # Mode for line drawing
    self.mode = 'linear'  # default to linear mode

    # Get the directory where the script is located
    script_dir = os.path.dirname(os.path.realpath(__file__))

    # Load the background image
    self.image = Image.open(script_dir + "/imgs/track.png")
    self.photo = ImageTk.PhotoImage(self.image)

    # Create a canvas to fit the image
    self.canvas = tk.Canvas(root, width=self.photo.width(), 
                            height=self.photo.height())
    self.canvas.pack()

    # Display the image on the canvas
    self.canvas.create_image(0, 0, image=self.photo, anchor=tk.NW)

    # Create buttons for switching modes
    self.linear_button = tk.Button(root, text="Linear Line",
                                   command=lambda: self.set_mode('linear'))
    self.linear_button.pack(side=tk.TOP, fill=tk.X)

    self.street_button = tk.Button(root, text="Street Line",
                                   command=lambda: self.set_mode('street'))
    self.street_button.pack(side=tk.TOP, fill=tk.X)

    # Store the marked points
    self.points = []

    # Bind the click event
    self.canvas.bind("<Button-1>", self.mark_point)
  
    # Delay between frames in seconds (adjust as needed)
    self.frame_delay = 0.01  
    
    # Animation control flags
    self.animation_running = False
    self.animation_paused = False

    # Additional buttons for animation control
    self.start_button = tk.Button(root, text="Start",
                                  command=self.start_animation)
    self.start_button.pack(side=tk.TOP, fill=tk.X)

    self.stop_button = tk.Button(root, text="Stop",
                                  command=self.stop_animation)
    self.stop_button.pack(side=tk.TOP, fill=tk.X)

    self.reset_button = tk.Button(root, text="Reset",
                                  command=self.reset)
    self.reset_button.pack(side=tk.TOP, fill=tk.X)
    
    
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
    
    # Get the directory where the script is located
    script_dir = os.path.dirname(os.path.realpath(__file__))
    
    # Load the vehicle image
    vehicle_img = Image.open(script_dir + "/imgs/vehicle.png")
    
    # Resize the vehicle image to make it larger
    # The tuple (width, height) determines the new size
    # Adjust the size as needed for your specific use case
    new_size = (80, 110)  # Increase the size as required
    vehicle_img_resized = vehicle_img.resize(new_size, Image.ANTIALIAS)
    vehicle_photo = ImageTk.PhotoImage(vehicle_img_resized)
    
    # Calculate pixel movement per frame based on linear_velocity and frame delay
    pixels_per_frame = linear_velocity / 0.005 * self.frame_delay
    
    # Create a vehicle image item on the canvas at the start position
    vehicle = self.canvas.create_image(path[0][0], path[0][1], image=vehicle_photo)
    
    # Keep a reference to the photo object to prevent garbage collection
    self.canvas.image = vehicle_photo
    
    # Animate the vehicle along the path
    for point in path[1:]:
      # Calculate the distance and direction to the next point
      dx = point[0] - self.canvas.coords(vehicle)[0]
      dy = point[1] - self.canvas.coords(vehicle)[1]
      distance = (dx**2 + dy**2)**0.5
      steps = int(distance / pixels_per_frame)
      
      # Move the vehicle towards the next point
      for _ in range(steps):
        self.canvas.move(vehicle, dx / steps, dy / steps)
        self.canvas.update()
        time.sleep(self.frame_delay)
    
    # Set the vehicle to the final position
    self.canvas.coords(vehicle, path[-1][0], path[-1][1])
    
  
    # Keep a reference to the photo object to prevent garbage collection
    self.canvas.image = vehicle_photo

    # Animate the vehicle along the path
    for point in path[1:]:
        # Exit loop if animation is stopped or reset
        if not self.animation_running or self.animation_paused:
          break
    
    
  def start_animation(self):
    if not self.animation_running:
      self.animation_running = True
      self.animation_paused = False
      if self.mode == 'linear':
        self.animate_vehicle(self.points, 1)  # Start linear animation
      elif self.mode == 'street' and hasattr(self, 'path'):
        self.animate_vehicle(self.path, 1)  # Start street animation
      
        
  def stop_animation(self):
    self.animation_paused = True


  def reset(self):
    self.canvas.delete("all")
    self.canvas.create_image(0, 0, image=self.photo, anchor=tk.NW)
    self.points = []
    self.mode = 'linear'
    self.animation_running = False
    self.animation_paused = False
    
    
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

  # Get the directory where the script is located
  script_dir = os.path.dirname(os.path.realpath(__file__))

  # Load the matrix from the .npy file
  matrix_path = script_dir + "/data/track_matrix.npy"
  matrix = np.load(matrix_path)

  # Create the main window
  root = tk.Tk()

  # Create an instance of the application
  app = MapMarkerApp(root, matrix)

  # Start the Tkinter event loop
  root.mainloop()
  
  
if __name__ == '__main__':
  main()
