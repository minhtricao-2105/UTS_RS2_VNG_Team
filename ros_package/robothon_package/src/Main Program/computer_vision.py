## @file
# @brief This script is used for computer vision.
#
# It imports various libraries and functions necessary for computer vision, such as rospy, cv2, numpy, etc.
#
# @details
# It defines the following functions:
#
# - depth_callback: Callback function for the first image
# - batteries_callback: Callback function for the batteries image
# - canny_edge: Perform Canny edge detection
# - crop: Crop an image to a rectangle in the center of the image
# - compare_two_image: Compare two images and find the differences
#
# @see function.py
# @see colorLibrary.py
# @see sensor_msgs/Image
# @see std_msgs/String
# @see std_msgs/Float32MultiArray
#
# @author Minh Tri Cao
# @date May 9, 2023

#!/usr/bin/env python3
# THESE LIBRARY IS USED FOR COMPUTER VISION:
import rospy
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from function import*
from colorLibrary import*
import imutils
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray

# Create a CvBridge object to convert ROS messages to OpenCV Images
bridge = CvBridge()
bridge_1 = CvBridge()

# Create global variables:
image = None
image_1 = None

#Setting the flag so that the image only take once time only
flag = 1

## @brief Callback function for processing depth image messages
#
# This function is called every time an image message is received. 
# The function converts the ROS image message to an OpenCV image, and sets the global image variable to the converted image. 
# The function also sets the flag to 1 when the first message is received, indicating that the image has been initialized.
#
# @param data The depth image message
# @return None

def depth_callback(data):
    
    global image
    global flag
    global red_flag

    #At the begining flag is false, so take the first figure:
    if flag == 1:
        # Convert ROS image message to OpenCV image
        image = bridge.imgmsg_to_cv2(data, "bgr8")
        
        flag +=1

##  @Brief Callback function for processing battery images
#   This function is a callback for processing battery images received from a ROS node. 
#   It converts the ROS image message to an OpenCV image, detects the container to store the batteries, and then detects the location and type of the batteries using computer vision techniques.
#   @param  data The image data received from the ROS node
#   @return None


def batteries_callback(data):

    global image_1
    global flag
    
    if flag == 2:
        print("Take the second figure")
        # Convert ROS image message to OpenCV image
        image_1 = bridge_1.imgmsg_to_cv2(data, "bgr8")

        flag +=1

        # Detect the container to store the batteries first:
        colour_detection()

        # Detect the location of the batteries and the type of the batteries:
        perform_computervision()

##  @brief  Apply Canny edge detection on an image
#   This function takes an input image and applies Canny edge detection to find edges within the image. The function returns the resulting image with the edges highlighted.
#   @param img The input image to apply Canny edge detection on
#   @return The resulting image with edges highlighted

def canny_edge(img):

    img = cv.convertScaleAbs(img, alpha=1, beta=110)

    grey = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    # grey = cv.GaussianBlur(grey, (5,5),1)

    grey = crop(grey,1.8)

    T, thresh = cv.threshold(grey, 40, 255, 0)

    # Perform Canny edge detection
    edges = cv.Canny(grey, 70, 200)

    return edges


## @brief Crop an image to a rectangle in the center of the image
#
# This function crops an input image to a rectangle in the center of the image. The size of the rectangle is determined by the `factor` input, which should be a positive integer. The function returns the cropped image as a new image with the same dimensions as the original image.
#
# @param img The input image to be cropped
# @param factor The factor used to calculate the size of the rectangle to crop
#

def crop(img,factor):
    # Get the dimensions of the image
    height, width = img.shape[:2]

    # Calculate the dimensions of the rectangle to crop
    w = int(width / (factor*2.2))
    h = int(height / (factor))


    x = (width - w) // 2
    y = (height - h) // 2

    # Crop the image
    cropped = img[y:y+h, x:x+w]

    # Create a black image with the same size as the original image
    black = np.zeros((height, width), dtype=np.uint8)

    # Calculate the dimensions to paste the cropped image onto the black image
    x_offset = (width - w) // 2
    y_offset = (height - h) // 2

    # Paste the cropped image onto the black image
    black[y_offset:y_offset+h, x_offset:x_offset+w] = cropped

    return black

## @brief Compare two images using Canny edge detection and contour analysis
#
# This function compares two input images using Canny edge detection and contour analysis. The function takes two images, `image` and `image_1`, and applies Canny edge detection to each image. The absolute difference between the edge maps is computed and thresholded to create a binary mask. The function then performs contour analysis on the binary mask to extract contours. Contours with an area less than 20 are discarded. The function then compares each contour to a set of predefined coordinates, stored in the `coordinates` variable, and returns a list of tuples containing the x and y coordinates of the center of the contour, the radius of the contour, and an index corresponding to the predefined coordinates that the contour is closest to.
#
# @return A list of tuples containing the x and y coordinates of the center of the contour, the radius of the contour, and an index corresponding to the predefined coordinates that the contour is closest to.

def compare_two_image():
    #Calling the global variable:
    global image
    global image_1

    # Calling two image after applying canny edge detection:
    edge = canny_edge(image)

    edge_1 = canny_edge(image_1)

    # Compute the absolute difference between the edge maps
    edges_diff = cv.absdiff(edge, edge_1)

    # Threshold the difference image to create a binary mask
    thresh = cv.threshold(edges_diff, 100, 150, cv.THRESH_BINARY)[1]

    contours = cv.findContours(thresh, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    contours = imutils.grab_contours(contours)

    new_contours = list()

    for contour in contours:
        if cv.contourArea(contour) > 20:
            new_contours.append(contour)
    
    # store it in the vector:
    coordinates = find_first_position()
    
    # store the result after:
    result = []
    for contour in new_contours:
        
        (x1,y1), radius = cv.minEnclosingCircle(contour)
        
        center = (int(x1), int(y1))
        
        radius = int(radius)
        
        perimeter = cv.arcLength(contour,True)
        
        area = cv.contourArea(contour)

        if(area > 30 and perimeter > 150):

            for (x, y, r, index) in coordinates:
                dx = abs(x1 - x)
                dy = abs(y1 - y)

                if(dx < 15 and dy < 15):
                    if(r<20):
                        result.append((x1,y1,0
                                       ,index))
                    elif(r>20):
                        result.append((x1,y,1,index))

    return result

## @brief Perform computer vision to detect the position of objects in an image

# This function performs computer vision to detect the position of batteries in an image. 
# It first calls the `compare_two_image()` function to compare two input images using Canny edge detection and contour analysis. 
# It then compares the resulting contours with a set of predefined coordinates, stored in the `find_first_position()` function, to determine the position of the batteries
# Finally, the function displays the image with detected objects using `cv.imshow()`, waits for user input, and returns.

# @return None

def perform_computervision():
    
    # Canny Edge Detection when we have the figure:
    print('Processing CV')

    final = np.empty((0,3), int)

    for (x,y,r,i) in compare_two_image():
        for(x1,y1,r1,i1) in find_first_position():
            if i == i1:
                final = np.vstack((final, [x1, y1, 0.2]))

    final = np.unique(final, axis=0)
  
    message = Float32MultiArray(data=final.flatten())

    publisher_2.publish(message)

    # for(x,y,r,i) in final:
    #     # print(x, y, r)
        
    #     if r == 1:
    #         print("AA Batery at position:",x,y)
    #         cv.circle(image_1, (x, y), 6, (0, 0, 255), -9)
    #     if r == 0:
    #         cv.circle(image_1, (x, y), 6, (0, 0, 0), -9)
    #         print("AAA Batery at position:",x,y)

    print('DONE THE CV')

    cv.imshow('hello', image_1)

    cv.waitKey(0)

    # cv.destroyAllWindows()
    input("Done The Vision Part, Remove the Cable to Continue .... \n")

## @brief Detects the color of battery containers in an image using OpenCV
#
# This function detects the color of battery containers in an input image using OpenCV. 
# The function first creates two arrays, `yellow_array` and `orange_array`, to store the center of each color. 
# The function then applies contour analysis on the yellow and orange contours detected in the image using `yellow_detection` and `orange_detection` functions. 
# The contour with an area greater than a certain threshold value is considered a battery container, and its center is added to the corresponding array. 
# The function then flattens the arrays and sends the data to robot nodes via a ROS topic using `Float32MultiArray` messages.
#
# @return None

def colour_detection():

    #Create an array to store the center of each color:

    # Yellow Array to store the AAA Batteries Containers
    yellow_array = np.empty((0,2),int)

    # Orange Array to store the AA Batteries Containers
    orange_array = np.empty((0,2),int)

    # Yellow Dectection start from here:
    for cnt in yellow_detection(image):
        area = cv.contourArea(cnt)
        if area > 4000: # Adjust the threshold value as needed
            x,y,w,h = cv.boundingRect(cnt)
            cv.rectangle(image,(x,y),(x+w,y+h),(0,0,255),5)

            #Find the center of the contour:
            M = cv.moments(cnt)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                
                yellow_array = np.vstack((yellow_array, [cx,cy]))
    
    # Now send the data to the robot nodes:
    message = Float32MultiArray(data = yellow_array.flatten())

    # Publish the message on the topic
    publisher_3.publish(message)

    # Orange Detection start from here:
    for cnt in orange_detection(image):
        area = cv.contourArea(cnt)
        if area > 6500: # Adjust the threshold value as needed
            x,y,w,h = cv.boundingRect(cnt)
            cv.rectangle(image,(x,y),(x+w,y+h),(0,255,255),7)

            #Find the center of the contour:
            M = cv.moments(cnt)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

                #Append to orange array:
                orange_array = np.vstack((orange_array, [cx,cy]))

    # Now send the data to the robot nodes:
    message = Float32MultiArray(data = orange_array.flatten())

    # Publish the message on the topic
    publisher_3.publish(message)
    

rospy.init_node('Realsense')

rospy.Subscriber("/camera/color/image_raw", Image, depth_callback)

# Create a first publisher to publish a message to a robot nodes
sub_1 = publisher_1 = rospy.Publisher('Computer_Vision', String, queue_size = 20)

# A secondary publisher will be established to transmit data obtained after performing computer vision tasks.
publisher_2 = rospy.Publisher('Image_Data',Float32MultiArray, queue_size = 20)

# A third publisher will be establisted to transmit data for colour detection:
publisher_3 = rospy.Publisher('Color_data', Float32MultiArray, queue_size = 20)

input("Press any key to capture a second image\n")

sub_2 = rospy.Subscriber("/camera/color/image_raw", Image, batteries_callback)

# wait for a key press
rospy.spin()






