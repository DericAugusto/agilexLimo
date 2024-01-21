from PIL import Image
import numpy as np
import os


def image_to_matrix(image_path):
  # Load the image
  img = Image.open(image_path)
  img = img.convert('RGB')  # Ensure it's in RGB format

  # Initialize the matrix
  matrix = np.zeros((img.height, img.width), dtype=int)

  # Process each pixel
  for y in range(img.height):
    for x in range(img.width):
      r, g, b = img.getpixel((x, y))
      if r == 255 and g == 255 and b == 255:
        # White pixel
        matrix[y, x] = 0
      elif r == 0 and g == 255 and b == 0:
        # Pure green pixel
        matrix[y, x] = 2
      else:
        # Any other color
        matrix[y, x] = 1

  return matrix


def matrix_to_image(matrix, output_path):
  # Create a new image with the same dimensions as the matrix
  img = Image.new('RGB', (matrix.shape[1], matrix.shape[0]))

  # Process each element in the matrix
  for y in range(matrix.shape[0]):
    for x in range(matrix.shape[1]):
      if matrix[y, x] == 1:
        # Pure black pixel
        img.putpixel((x, y), (0, 0, 0))
      elif matrix[y, x] == 2:
        # Pure green pixel
        img.putpixel((x, y), (0, 255, 0))
      else:
        # White pixel
        img.putpixel((x, y), (255, 255, 255))

  # Save or display the image
  img.save(output_path)
  img.show()
  
  
def export_matrix_as_binary(matrix, filepath):
  np.save(filepath, matrix)


def get_matrix_size(matrix):
  if not isinstance(matrix, np.ndarray):
    raise TypeError("Input must be a NumPy array.")

  return matrix.shape


def main(args=None):
  # Get the directory where the script is located
  script_dir = os.path.dirname(os.path.realpath(__file__))
  image_path = "/imgs/track_large.png"
  image_matrix = image_to_matrix(script_dir + image_path)
  
  output_path_image = script_dir + "/imgs/track_matrix.png"
  matrix_to_image(image_matrix, output_path_image)
  output_path_matrix = script_dir + "/data/track_matrix.npy"
  export_matrix_as_binary(image_matrix, output_path_matrix)
  
  size = get_matrix_size(image_matrix)
  print("Matrix size:", size)
  
  
if __name__ == '__main__':
  main()

