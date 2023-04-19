# Import Library for the Open CV and Numpy
import cv2 as cv
import numpy as np
from function import*

# Import Figure from the folder, delete this one when you have a camera:
img = cv.imread('Photo/tsukasa.png')

# Change 0.2 to suitable value
img = rescaleFrame(img, 0.25)

img = cv.convertScaleAbs(img, alpha=1, beta=50)

hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

#Detect Green Color: BGR
lower_green = np.array([50, 50, 50])
upper_green = np.array([70, 255, 255])

#Create a mask
mask = cv.inRange(hsv, lower_green, upper_green)

#File a contours in the mask
contours, hierarchy = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

# Loop over the contours and draw a bounding box around the yellow objects
for cnt in contours:
    area = cv.contourArea(cnt)
    if area > 500: # Adjust the threshold value as needed
        x,y,w,h = cv.boundingRect(cnt)
        cv.rectangle(img,(x,y),(x+w,y+h),(255,0,0),5)

        #Find the center of the contour:
        M = cv.moments(cnt)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            
            # Draw a red dot at the center point
            cv.circle(img, (cx, cy), 5, (0, 0, 255), -2)
            
            # Print the coordinates of the center point
            print("Center point: ({}, {})".format(cx, cy))

            #Print it on the figure
            cv.putText(img, f'({cx}, {cy})', (cx, cy), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)


# Apply a median blur to the mask to reduce noise
mask = cv.medianBlur(mask, 5)

# Show the figure:
cv.imshow('Person', img)

# Wait for another key board
cv.waitKey(0)
cv.destroyAllWindows()