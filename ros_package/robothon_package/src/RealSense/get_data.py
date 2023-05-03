# This function will create a rosnode to receive the data from the realsens
import rospy
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from function import*
from colorLibrary import*
import imutils

# Create a CvBridge object to convert ROS messages to OpenCV Images
bridge = CvBridge()


# Define a callback funtion to process the RBG image
def rgb_callback(data):
    
    img = bridge.imgmsg_to_cv2(data, "bgr8")

    #Create an array to store the center of each color
    yellow_array = []
    green_array = []
    orange_array = []

    # Yellow Dectection:
    for cnt in yellow_detection(img):
        area = cv.contourArea(cnt)
        if area > 4000: # Adjust the threshold value as needed
            x,y,w,h = cv.boundingRect(cnt)
            cv.rectangle(img,(x,y),(x+w,y+h),(0,0,255),5)

            #Find the center of the contour:
            M = cv.moments(cnt)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                
                yellow_array.append((cx,cy))

                # Draw a red dot at the center point
                cv.circle(img, (cx, cy), 5, (0, 0, 255), -2)
                
                #Print it on the figure
                cv.putText(img, f'({cx}, {cy})', (cx, cy), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
    
    # Orange Detection:
    for cnt in orange_detection(img):
        area = cv.contourArea(cnt)
        if area > 6500: # Adjust the threshold value as needed
            x,y,w,h = cv.boundingRect(cnt)
            cv.rectangle(img,(x,y),(x+w,y+h),(0,255,255),7)

            #Find the center of the contour:
            M = cv.moments(cnt)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

                # Draw a red dot at the center point
                cv.circle(img, (cx, cy), 5, (255, 0, 255), -2)
            
                #Print it on the figure
                cv.putText(img, f'({cx}, {cy})', (cx, cy), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2) 

                #Append to orange array:
                orange_array.append((cx,cy))
    

    # grey = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    # grey = cv.GaussianBlur(grey, (3,3),0)
    
    # T, thresh = cv.threshold(grey, 100, 255, cv.THRESH_BINARY)

    # contours = cv.findContours(thresh, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    # contours = imutils.grab_contours(contours)

    # # print(len(contours))
    # new_contours = list()

    # for contour in contours:
    #     if cv.contourArea(contour) > 50:
    #         new_contours.append(contour)
    
    # for contour in new_contours:
    #     cv.drawContours(thresh, [contour], 0, (0,255,0),3)

    cv.imshow('Color Detection', img)
    # cv.imshow("BRG", cv_image)
        
    # If this is the first frame, turn off the subscriber
    # print("hello")

    # img_1 = thresh

    # # if counter == 1:
    # #     rospy.signal_shutdown('message')

    # return thresh
    cv.waitKey(1)


rospy.init_node("Realsense")

print('In the loop')

rospy.Subscriber("/camera/color/image_raw", Image, rgb_callback)

rospy.Rate(100)

# rospy.Subscriber("/camera/depth/image_rect_raw", Image, depth_callback)

rospy.spin()


cv.destroyAllWindows()
