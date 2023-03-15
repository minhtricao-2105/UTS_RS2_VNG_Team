import numpy as np
import cv2 as cv
import math
from math import sqrt, pow
img = cv.imread('Photo/photo3.jpg')

#Need to rescale a little bit
def rescaleFrame(frame, scale = 0.7):
    width     = int(frame.shape[1] * scale)
    height    = int(frame.shape[0] * scale)
    dimension = (width, height)

    return cv.resize(frame, dimension, interpolation = cv.INTER_AREA)

img = rescaleFrame(img)

# Create a blank with the same size of the figure
blank = np.zeros(img.shape, dtype = 'uint8')

# Convert the figure into gray
gray= cv.cvtColor(img, cv.COLOR_BGR2GRAY)

# Blur before apple the canny function
gray= cv.GaussianBlur(gray, (7, 7), 0)

# canny edge
canny = cv.Canny(gray, 150,225)

cv.imshow("canny",canny)

#find Condur
# contours, hierarchy = cv.findContours(canny, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

# cv.drawContours(img, contours, -1, (0, 0, 255), 4)

# Find non-zero pixel positions
positions = cv.findNonZero(canny)

# Print the first 5 pixel positions
print(len(positions))

# find y min:
min_y_idx = np.argmin(positions[:, 0, 1])

# find y max:
max_y_idx = np.argmax(positions[:, 0, 1])

# find x max:
max_x_idx = np.argmax(positions[:, 0, 0])

# Get coordinates of point with minimum y value
min_y_point = positions[min_y_idx][0]
max_y_point = positions[max_y_idx][0]
max_x_point = positions[max_x_idx][0]

# Find the height and width 
height, width = img.shape[:2]

# Find another point:
def distance(p1,p2):
    distance_1 = sqrt(pow(p1[0]-p2[0],2)+pow(p1[1]-p2[1],2))
    return distance_1

arrayDistance_0 = []
arrayDistance_1 = []
arrayDistance_2 = []
arrayDistance_3 = []


# distance from 0 0
for i in positions:
    arrayDistance_0.append(distance([0, 0], i[0]))

# distance from 0 0
for i in positions:
    arrayDistance_1.append(distance([width, 0], i[0]))

# distance from 0 0
for i in positions:
    arrayDistance_2.append(distance([width,height], i[0]))

# distance from 0 0
for i in positions:
    arrayDistance_3.append(distance([0,height], i[0]))


random_point_0 = positions[np.argmin(arrayDistance_0)][0]
random_point_1 = positions[np.argmin(arrayDistance_1)][0]
random_point_2 = positions[np.argmin(arrayDistance_2)][0]
random_point_3 = positions[np.argmin(arrayDistance_3)][0]


# print(random_point_4)
#print(arrayDistance)
        
newArray = [random_point_0, random_point_1, random_point_2, random_point_3]
print("Coordinates of point with minimum y value:", min_y_point)

#Display a figure
cv.circle(img, tuple(random_point_0),10,(255,0,255),thickness = 3)
cv.circle(img, tuple(random_point_1),10,(255,0,255),thickness = 3)
cv.circle(img, tuple(random_point_2),10,(255,0,255),thickness = 3)
cv.circle(img, tuple(random_point_3),10,(255,0,255),thickness = 3)




cv.imshow('new hori', img)


# newArray  = []
# for i in positions:
   
#     if i[0,0]> min_y_point[0] and i[0,0] < max_x_point[0] and i[0,1] > min_y_point[1] and i[0,1] < max_y_point[1]:
#         continue
#     else:
#         newArray.append(i)


        
# newArray = np.array(newArray)
# # newArray = newArray.reshape((-1, 1, 2))
# cv.drawContours(img, [newArray], -1, (0, 255, 0), 3)




# Draw the rectangle
cv.line(img, tuple(random_point_0), tuple(random_point_1), (0, 255, 0), 3)
cv.line(img, tuple(random_point_1), tuple(random_point_2), (0, 255, 0), 3)
cv.line(img, tuple(random_point_2), tuple(random_point_3), (0, 255, 0), 3)
cv.line(img, tuple(random_point_3), tuple(random_point_0), (0, 255, 0), 3)

cv.imshow('new horisan', img)
# Draw the contours

cv.waitKey(0)




cv.destroyAllWindows()