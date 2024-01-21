import tkinter as tk
from PIL import Image, ImageTk
import os

class MapMarkerApp:
  def __init__(self, root):
    self.root = root
    self.root.title("Map Marker")

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
    self.canvas.create_image(0, 0, image=self.photo, anchor=tk.NW)
    self.canvas.pack()

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
      color = 'blue' if len(self.points) == 0 else 'red'
      self.canvas.create_oval(x1, y1, x2, y2, fill=color, outline=color)

      # Save the point
      self.points.append((event.x, event.y))

      # If two points are marked and mode is linear, draw a line
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
    # Hardcoded waypoints that represent the red path on the image
    # You will need to adjust these based on the actual positions
    waypoints = [
      (start_x, start_y), # The starting waypoint
      # Add more waypoints that represent the turns in the path
      (waypoint1_x, waypoint1_y),
      (waypoint2_x, waypoint2_y),
      # ...
      (end_x, end_y) # The ending waypoint
    ]

  # Find the closest waypoints to the clicked points
  start_waypoint = self.find_closest_waypoint(self.points[0], waypoints)
  end_waypoint = self.find_closest_waypoint(self.points[1], waypoints)

  # Find indices of the closest waypoints1
  start_index = waypoints.index(start_waypoint)
  end_index = waypoints.index(end_waypoint)

  # Determine the direction to iterate through the waypoints
  step = 1 if start_index < end_index else -1

  # Draw the line through the waypoints in the correct order
  for i in range(start_index, end_index + step, step):
    self.canvas.create_line(waypoints[i][0], waypoints[i][1],
                            waypoints[i + step][0], waypoints[i + step][1],
                            fill='orange', width=2)

  print("Street line drawn between:", self.points)

def find_closest_waypoint(self, point, waypoints):
  # Find the waypoint closest to the given point
  return min(waypoints, key=lambda wp: (wp[0] - point[0]) ** 2 + (wp[1] - point[1]) ** 2)


# Create the main window
root = tk.Tk()

# Create an instance of the application
app = MapMarkerApp(root)

# Start the Tkinter event loop
root.mainloop()
