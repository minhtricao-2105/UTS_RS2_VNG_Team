import pyrealsense2 as rs
from PIL import Image

# Initialize the RealSense pipeline and configure it
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
pipeline.start(config)

# Capture a frame from the RealSense camera
frames = pipeline.wait_for_frames()
color_frame = frames.get_color_frame()

# Convert the color frame to a PIL Image object
image = Image.frombytes('RGB', (color_frame.width, color_frame.height), color_frame.get_data())

# Save the image to a file
image.save('image.jpg')

# Clean up and close the RealSense pipeline
pipeline.stop()
