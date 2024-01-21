import tkinter as tk
from PIL import Image, ImageTk
import numpy as np
import os

class MapMarkerApp:
  def __init__(self, root, matrix):
    self.root = root
    self.root.title("Map Marker")
    self.matrix = matrix  # The matrix indicating valid and invalid areas
    
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

    # Store the marked points
    self.points = []

    # Bind the click event
    self.canvas.bind("<Button-1>", self.mark_point)

  def mark_point(self, event):
    # Check if the click is in a valid area
    if self.matrix[event.y, event.x] != 0:
      # Limit to 2 points
      if len(self.points) < 2:
        # Create an oval mark for the point
        radius = 5
        color = 'blue' if len(self.points) == 0 else 'red'
        self.canvas.create_oval(event.x - radius, event.y - radius,
                                event.x + radius, event.y + radius,
                                fill=color, outline=color)

        # Save the point
        self.points.append((event.x, event.y))


# Get the directory where the script is located
script_dir = os.path.dirname(os.path.realpath(__file__))

# Load the matrix from the .npy file
matrix_path = script_dir + "/data/track_matrix.npy"
matrix = np.load(matrix_path)

# Create the main window
root = tk.Tk()

# Create an instance of the application with the matrix as an argument
app = MapMarkerApp(root, matrix)

# Start the Tkinter event loop
root.mainloop()
