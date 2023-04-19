import numpy as np
import cv2 as cv
import math
from math import sqrt, pow

# Rescale Function:
def rescaleFrame(frame, scale):
    width     = int(frame.shape[1] * scale)
    height    = int(frame.shape[0] * scale)
    dimension = (width, height)

    return cv.resize(frame, dimension, interpolation = cv.INTER_AREA)

# Find distance from 2 points:
def distance(p1,p2):
    distance_1 = sqrt(pow(p1[0]-p2[0],2)+pow(p1[1]-p2[1],2))
    return distance_1

#find nearest point
def findnNear(positions, point):
    arrayDistance_0 = []
    for i in positions:
        arrayDistance_0.append(distance(point, i[0]))
    random_point_0 = positions[np.argmin(arrayDistance_0)][0]
    return random_point_0


def findCordinate(positions, img):
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

    newArray = [random_point_0, random_point_1, random_point_2, random_point_3]
    
    return newArray

#Draw a rectangle for 4 points
def drawRec(img, array):
    cv.line(img, tuple(array[0]), tuple(array[1]), (235, 97, 35), 3)
    cv.line(img, tuple(array[1]), tuple(array[2]), (235, 97, 35), 3)
    cv.line(img, tuple(array[2]), tuple(array[3]), (235, 97, 35), 3)
    cv.line(img, tuple(array[3]), tuple(array[0]), (235, 97, 35), 3)
    

#Displace 4 points:
def displacePoints(img, random_point_0, random_point_1, random_point_2, random_point_3):
    cv.circle(img, tuple(random_point_0),10,(0,0,255),thickness = 3)
    cv.circle(img, tuple(random_point_1),10,(0,0,255),thickness = 3)
    cv.circle(img, tuple(random_point_2),10,(0,0,255),thickness = 3)
    cv.circle(img, tuple(random_point_3),10,(0,0,255),thickness = 3)    

#Find a middle point:
def findMiddle(point1, point2):
    middle_x = (point1[0] + point2[0])/2
    middle_y = (point1[1] + point2[1])/2
    return [int(middle_x), int(middle_y)]

#Find an array of midpoint
def arrayMidPoint(array):
    a0 = findMiddle(array[0],array[1])
    a1 = findMiddle(array[1],array[2])
    a2 = findMiddle(array[2],array[3])
    a3 = findMiddle(array[3],array[0])

    return [a0, a1, a2, a3]

#displace 1 points
def displacePoint(img, point):
    cv.circle(img, tuple(point),10,(0,0,255),thickness = 3)

#find a edge near an edge
def findEdgeNear(edge, point, img, array):
    contours, _ = cv.findContours(edge, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    min_distance = float('inf')
    closest_contour = None
    for contour in contours:
        distance = cv.pointPolygonTest(contour, tuple(point), True)
        if distance >= 0 and distance < min_distance:
            min_distance = distance
            closest_contour = contour
    # Draw the closest contour on the original image
    if closest_contour is not None:
        cv.drawContours(img, [closest_contour], -1, (255, 0, 0), 6)
    else: drawRec(img,array)
    

