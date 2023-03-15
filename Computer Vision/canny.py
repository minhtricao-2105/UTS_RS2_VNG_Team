#Import Library used in the project:

import numpy as np
import cv2 as cv
import math
from math import sqrt, pow
from function import*

#Import picture:
img = cv.imread('Photo/photo3.jpg')

#Formating the img

img = rescaleFrame(img)

# Create a blank with the same size of the figure
blank = np.zeros(img.shape, dtype = 'uint8')

# Convert the figure into gray
gray= cv.cvtColor(img, cv.COLOR_BGR2GRAY)

# Blur before apple the canny function
gray= cv.GaussianBlur(gray, (9, 9), 0)

# canny edge
canny = cv.Canny(gray, 125,225)

# Find non-zero pixel positions
positions = cv.findNonZero(canny)

#Show the firgue:
cv.imshow('new', canny)
cv.imshow('new1',img)

cv.waitKey(0)
cv.destroyAllWindows()