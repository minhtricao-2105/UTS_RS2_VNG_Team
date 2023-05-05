import numpy as np

# Define the camera parameters
pixel_width = 1280  # Width of the image in pixels
pixel_height = 720  # Height of the image in pixels
focal_length = 848  # Focal length of the camera in pixels
camera_transform = np.array([[0.02, 0.1, -1, 0], [-1, 0.02, 0.02, 0], [0, 1, 0.1, 0], [-0.01, 0.3, 0.4, 1]])  # Camera transform in global frame

# Define the point in Pixel frame
pixel_x = 200  # X-coordinate of the point in pixels
pixel_y = 250  # Y-coordinate of the point in pixels

# Compute the normalized coordinates of the point
normalized_x = (pixel_x - pixel_width / 2) / focal_length
normalized_y = (pixel_height / 2 - pixel_y) / focal_length

# Compute the distance from the camera to the global point
distance = 0.2  # Distance from the camera to the global point in meters

# Compute the position of the point in the camera frame
camera_position = np.array([distance * normalized_x, distance * normalized_y, distance, 1])

# Map the point from the camera frame to the global frame
global_position = np.matmul(camera_transform, camera_position)

# Print the result
print("Pixel frame coordinates: ({}, {})".format(pixel_x, pixel_y))
print("Global frame coordinates: ({}, {}, {})".format(global_position[0], global_position[1], global_position[2]))
