#Import Library used in the project:

import numpy as np
import cv2 as cv
import math
from math import sqrt, pow
from function import*

#Import picture:
img = cv.imread('WebCam/pic2.jpg')

#Get the height of the figure => so that we can calculate the scale

#Scale the figure down for good resolution:
scale = 1-((img.shape[0]-336)/img.shape[0])
#Formating the img

img = rescaleFrame(img,scale)

print("Height: ", img.shape[0])
print("Width: ", img.shape[1])

cv.imshow('news',img)

# Create a blank with the same size of the figure
blank = np.zeros(img.shape, dtype = 'uint8')

# Convert the figure into gray
gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

# Define the brightness level (-100 to 100)
brightness = 100

# Create a matrix of zeros with the same shape and type as the input image
gray = np.zeros_like(img)

# Subtract the brightness level from each pixel value
gray = cv.convertScaleAbs(img, alpha=1, beta=brightness)

# Blur before apple the canny function
gray= cv.GaussianBlur(gray, (5, 5), 0)

# canny edge
canny = cv.Canny(gray, 100,250)

# Find non-zero pixel positions
positions = cv.findNonZero(canny)

# Find 4 coordinate point of a figure:
findCordinate(positions, img)

# Draw a rectangle for 4 coorditane
drawRec(img, findCordinate(positions, img))

#The function findCordinate() is store the value of 4 specific points
array = findCordinate(positions, img)

# findEdgeNear(canny, array[0], img, array)
for point in array:
    print(point)


# Exit if the 'q' key is pressed
while True:
    cv.imshow('new horisan', img)
    cv.imshow('hello', canny)
    if cv.waitKey(1) == ord('q'):
        break
cv.destroyAllWindows()