import tkinter as tk
from PIL import Image, ImageTk
import os
import numpy as np

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
    self.image = Image.open(script_dir + "/imgs/environment.png")
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
    
    
  def set_mode(self, mode):
    self.mode = mode
    print(f"Mode set to: {self.mode}")


  def mark_point(self, event):
    # Check if the click is in a valid area
    if self.matrix[event.y, event.x] != 0:
      # Limit to 2 points
      if len(self.points) < 2:
        # Create an oval mark for the point
        radius = 5
        x1, y1 = (event.x - radius), (event.y - radius)
        x2, y2 = (event.x + radius), (event.y + radius)
        # Use a different color for the starting point
        color = 'blue' if len(self.points) == 0 else 'red'
        self.canvas.create_oval(x1, y1, x2, y2, fill=color, outline=color)

        # Save the point
        self.points.append((event.x, event.y))

        # If two points are marked and mode is linear, draw a linear line
        if len(self.points) == 2:
          if self.mode == 'linear':
            self.draw_linear_line()
          elif self.mode == 'street':
            self.draw_street_line()


  def draw_linear_line(self):
    self.canvas.create_line(self.points[0][0], self.points[0][1],
                            self.points[1][0], self.points[1][1],
                            fill='green', width=2)
    print("Linear line drawn between:", self.points)


  def draw_street_line(self):
    # Placeholder for street line drawing logic
    # TODO: Implement the logic to follow the streets
    print("Street line mode is not implemented yet.")


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
matrix_path = script_dir + "/data/track_matrix.npy"
matrix = np.load(matrix_path)

# Create the main window
root = tk.Tk()

# Create an instance of the application
app = MapMarkerApp(root, matrix)

# Start the Tkinter event loop
root.mainloop()
