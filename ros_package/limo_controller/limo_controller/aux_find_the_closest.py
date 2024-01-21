import numpy as np
import os
import tkinter as tk
from PIL import Image, ImageTk
import random

def display_image_with_points(image_path, matrix):
  root = tk.Tk()
  root.title("Image with Points")

  image = Image.open(image_path)
  tk_image = ImageTk.PhotoImage(image)

  canvas = tk.Canvas(root, width=image.width, height=image.height)
  canvas.pack()
  canvas.create_image(0, 0, anchor='nw', image=tk_image)

  def plot_point(coordinate, color='red'):
    # Assuming coordinate is given in (column, row) format
    x, y = coordinate[0], coordinate[1]
    radius = 5
    # In the create_oval call, x and y are used as is because the canvas uses the screen coordinate system
    return canvas.create_oval(x - radius, y - radius, x + radius, y + radius, fill=color)

      
  def update_points():
    num_rows, num_cols = len(matrix), len(matrix[0])
    random_row, random_col = random.randint(0, num_rows - 1), random.randint(0, num_cols - 1)
    coord = (random_row, random_col)
    closest = find_closest_two(matrix, coord)
    print(f"Random Coordinate: {coord}, Closest '2': {closest}")

    # Clear previous points and plot new ones
    canvas.delete("point")
    point1 = plot_point(coord, 'red')
    point2 = plot_point(closest, 'blue')
    canvas.addtag_withtag("point", point1)
    canvas.addtag_withtag("point", point2)

    # Schedule next update
    root.after(800, update_points)  # 1 second

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
