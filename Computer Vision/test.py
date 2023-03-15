#Import Library used in the project:

import numpy as np
import cv2 as cv
import math
from math import sqrt, pow
from function import*

#Import picture:
img = cv.imread('Photo/photo4.PNG')

#Formating the img

img = rescaleFrame(img)

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


#Find 4 position of an edge:
point_edge = findCordinate(positions, img)

#Find and edge:
# drawRec(img, point_edge[0],point_edge[1],point_edge[2],point_edge[3])

#Displace points
displacePoints(img, point_edge[0],point_edge[1],point_edge[2],point_edge[3])

#find midpoint
midpoint = arrayMidPoint(point_edge)

displacePoints(img, midpoint[0], midpoint[1], midpoint[2], midpoint[3])

displacePoint(img,findMiddle(midpoint[0],midpoint[2]))

#find the edge inside start here:
displacePoint(img, findnNear(positions, findMiddle(midpoint[0],findMiddle(midpoint[0],midpoint[2]))))


#Show the firgue:
findEdgeNear(canny, findMiddle(midpoint[0],midpoint[2]),img, point_edge)
cv.imshow('new', canny)


cv.waitKey(0)
cv.destroyAllWindows()