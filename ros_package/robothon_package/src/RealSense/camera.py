# This function will create a rosnode to receive the data from the realsens
import rospy
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Create a CvBridge object to convert ROS messages to OpenCV Images
bridge = CvBridge()

# Define a callback funtion to process the RBG image
def rgb_callback(data):
    # This function will convert a ros image to openCV image
    # The format of OpenCV image is BGR color space with 8 bits per channel
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")

    #Display the image in a window 
    cv.imshow("BRG", cv_image)

    cv.waitKey(1)

# Define a callback function to process the depth image
def depth_callback(data):

    # Convert the ROS image to an OpenCV image
    cv_image = bridge.imgmsg_to_cv2(data, "32FC1")

    # Normalize the depth values for display
    cv_image = cv.normalize(cv_image, None, 0, 1, cv.NORM_MINMAX)

    # Display the image in a window
    cv.imshow("Depth", cv_image)
    
    cv.waitKey(1)