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

###########################################################################################
###########################################################################################
################################ BELONG TO COMPUTER VISION ################################ 

# Create a CvBridge object to convert ROS messages to OpenCV Images
bridge = CvBridge()
bridge_1 = CvBridge()

# Create global variables:
image = None
image_1 = None

#Setting the flag so that the image only take once time only
flag = 1

# Callback function for the first image
def depth_callback(data):
    global image
    global flag
    global red_flag

    #At the begining flag is false, so take the first figure:
    if flag == 1:
        # Convert ROS image message to OpenCV image
        image = bridge.imgmsg_to_cv2(data, "bgr8")
        
        flag +=1

# Callback function for the batteries image
def batteries_callback(data):
    global image_1
    global flag
    
    if flag == 2:
        print("Take the second figure")
        # Convert ROS image message to OpenCV image
        image_1 = bridge_1.imgmsg_to_cv2(data, "bgr8")

        flag +=1

        perform_computervision()

def canny_edge(img):

    img = cv.convertScaleAbs(img, alpha=1, beta=110)

    grey = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    # grey = cv.GaussianBlur(grey, (5,5),1)

    grey = crop(grey,1.7)

    T, thresh = cv.threshold(grey, 40, 255, 0)

    # Perform Canny edge detection
    edges = cv.Canny(grey, 75, 200)

    return edges

# @brief: Crop Image function:
# @detail: This function will crop the middle of the figure and then combine it with the a black blank space so that it will have the same size of the initial image
# @input: image -> a image
# @input: factor -> the size of the crop image

def crop(img,factor):
    # Get the dimensions of the image
    height, width = img.shape[:2]

    # Calculate the dimensions of the rectangle to crop
    w = int(width / (factor*2.3))
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
        
        if(area > 35 and perimeter > 150):
            # cv.circle(image_1, center, radius, (255,0,0), 5)
            
            for (x, y, r, index) in coordinates:
                dx = abs(x1 - x)
                dy = abs(y1 - y)
                if(dx < 15 and dy < 15):
                    if(r<20):
                        result.append((x1,y1,0,index))
                    elif(r>20):
                        result.append((x1,y,1,index))

    return  result

def perform_computervision():
    
    # Canny Edge Detection when we have the figure:
    print('Processing CV')

    final = np.empty((0,3), int)

    for (x,y,r,i) in compare_two_image():
        for(x1,y1,r1,i1) in find_first_position():
            if i == i1:
                final = np.vstack((final, [x1, y1, r]))

    final = np.unique(final, axis=0)
  
    message = Float32MultiArray(data=final.flatten())

    publisher_2.publish(message)

    for(x,y,r) in final:
        # print(x, y, r)
        
        if r == 1:
            print("AA Batery at position:",x,y)
            cv.circle(image_1, (x, y), 6, (0, 0, 255), -9)
        if r == 0:
            cv.circle(image_1, (x, y), 6, (0, 0, 0), -9)
            print("AAA Batery at position:",x,y)

    print('DONE THE CV')

    cv.imshow('hello', image_1)

    cv.waitKey(0)

    # cv.destroyAllWindows()
    input("Done The Vision Part, Remove the Cable to Continue .... \n")

        
###########################################################################################
################################ BELONG TO COMPUTER VISION ################################  

rospy.init_node('Realsense')

rospy.Subscriber("/camera/color/image_raw", Image, depth_callback)

# Create a first publisher to publish a message to a robot nodes
sub_1 = publisher_1 = rospy.Publisher('Computer_Vision', String, queue_size=20)

# A secondary publisher will be established to transmit data obtained after performing computer vision tasks.
publisher_2 = rospy.Publisher('Image_Data',Float32MultiArray, queue_size=20)

input("Press any key to capture a second image\n")

sub_2 = rospy.Subscriber("/camera/color/image_raw", Image, batteries_callback)

# wait for a key press
rospy.spin()





