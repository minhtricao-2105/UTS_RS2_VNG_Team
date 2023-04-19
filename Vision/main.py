from colorLibrary import*
import cv2 as cv
import numpy as np
from function import*

# Import Figure from the folder, delete this one when you have a camera:
img = cv.imread('Photo/tsukasa.png')

#First, rescale a figure:
img = rescaleFrame(img, 0.25)

#Create an array to store the center of each color
yellow_array = []
green_array = []
orange_array = []

# Green detection:
for cnt in green_detection(img):
    area = cv.contourArea(cnt)
    if area > 500: # Adjust the threshold value as needed
        x,y,w,h = cv.boundingRect(cnt)
        cv.rectangle(img,(x,y),(x+w,y+h),(255,0,0),5)

        #Find the center of the contour:
        M = cv.moments(cnt)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            
            green_array.append((cx,cy))

            # Draw a red dot at the center point
            cv.circle(img, (cx, cy), 5, (0, 0, 255), -2)

            #Print it on the figure
            cv.putText(img, f'({cx}, {cy})', (cx, cy), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)

# Yellow Dectection:
for cnt in yellow_detection(img):
    area = cv.contourArea(cnt)
    if area > 800: # Adjust the threshold value as needed
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

#Orange Detection:
for cnt in orange_detection(img):
    area = cv.contourArea(cnt)
    if area > 5000: # Adjust the threshold value as needed
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

print("Yellow:", yellow_array)
print("Orange:", orange_array)
print("Green:", green_array)

# Detect whenever it is a battery:


# Show the figure:
cv.imshow('Person', img)


# Wait for another key board
cv.waitKey(0)
cv.destroyAllWindows()