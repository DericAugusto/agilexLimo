import numpy as np
import os
import tkinter as tk
from PIL import Image, ImageTk
import random
from queue import PriorityQueue


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


def display_image_with_points(image_path, matrix):
  root = tk.Tk()
  root.title("Image with Points")

  image = Image.open(image_path)
  tk_image = ImageTk.PhotoImage(image)

  canvas = tk.Canvas(root, width=image.width, height=image.height)
  canvas.pack()
  canvas.create_image(0, 0, anchor='nw', image=tk_image)

  def plot_point(coordinate, color='red', label=None):
    x, y = coordinate
    radius = 5
    # Add the tag "point" to the oval
    oval_id = canvas.create_oval(
      x - radius, y - radius, x + radius, y + radius, fill=color, tags=("point")
    )
    if label:
      # Add the tag "point" to the text
      canvas.create_text(
        x, y + radius + 10, text=label, fill=color, tags=("point")
      )
    return oval_id

  def update_points():
    canvas.delete("point")  # Clear previous points

    # Generate two random coordinates
    start_coord = (
      random.randint(0, len(matrix[0]) - 1),
      random.randint(0, len(matrix) - 1)
    )
    arrival_coord = (
      random.randint(0, len(matrix[0]) - 1),
      random.randint(0, len(matrix) - 1)
    )

    # Find the closest points with value 2
    start_closest = find_closest_two(matrix, start_coord)
    arrival_closest = find_closest_two(matrix, arrival_coord)
    path = a_star_search(matrix, start_closest, arrival_closest)

    if path:  # Check if a path is found
      # Plot the points
      plot_point(start_coord, 'red', 'start_point')
      plot_point(arrival_coord, 'blue', 'arrival_point')
      plot_point(start_closest, 'green', 'start_inline_point')
      plot_point(arrival_closest, 'green', 'arrival_inline_point')
      
      # Draw the path
      for i in range(len(path) - 1):
        point1 = path[i]
        point2 = path[i + 1]
        canvas.create_line(
          point1[0], point1[1],  # Adjust for pixel position if needed
          point2[0], point2[1],
          fill='red', width=2, tags=("point")
        )


    # Schedule next update
    root.after(500, update_points)  # 500 milliseconds

  update_points()
  root.mainloop()


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

# Get the directory where the script is located
script_dir = os.path.dirname(os.path.realpath(__file__))

# Load the matrix from the .npy file
matrix_path = os.path.join(script_dir, "data/track_matrix.npy")
matrix = np.load(matrix_path)

image_path = os.path.join(script_dir, "imgs/track.png")
display_image_with_points(image_path, matrix)
