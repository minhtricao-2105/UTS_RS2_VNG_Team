# Import Library for the Open CV and Numpy
import cv2 as cv
import numpy as np
from function import*

# Import Figure from the folder, delete this one when you have a camera:
img = cv.imread('Photo/Kubo_san.png')

# Change 0.2 to suitable value
img = rescaleFrame(img, 0.25)

img = cv.convertScaleAbs(img, alpha=1, beta=-20)

#Detect Green Color: BGR
lower_green = np.array([33, 100, 55])
upper_green = np.array([94, 164, 115])

#Create a mask
mask = cv.inRange(img, lower_green, upper_green)

# Apply a median blur to the mask to reduce noise
mask = cv.medianBlur(mask, 5)

# Show the figure:
cv.imshow('Person', mask)

# Wait for another key board
cv.waitKey(0)
cv.destroyAllWindows()