import tkinter as tk
from PIL import Image, ImageTk
import os

class MapMarkerApp:
  def __init__(self, root):
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

# Create the main window
root = tk.Tk()

# Create an instance of the application
app = MapMarkerApp(root)

# Start the Tkinter event loop
root.mainloop()
