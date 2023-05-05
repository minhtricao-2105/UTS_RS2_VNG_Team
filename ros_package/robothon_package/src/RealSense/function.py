import numpy as np
import cv2 as cv
import math
from math import sqrt, pow
from colorLibrary import*
import imutils
from colorLibrary import*

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
    
#Image Progressing for battery:


# Crop the middle of the image and then combine with the black blank space
# So that it will have the same size of the initial image:
def crop(img,factor):
    # Get the dimensions of the image
    height, width = img.shape[:2]

    # Calculate the dimensions of the rectangle to crop
    w = int(width / (factor*2))
    h = int(height / factor)


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

#Image processing apply canny edge to detect the edge of the image

def processing(img):

    img = cv.convertScaleAbs(img, alpha=1, beta=100)

    grey = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    grey = cv.GaussianBlur(grey, (3,3),1)

    grey = crop(grey,1.5)

    T, thresh = cv.threshold(grey, 40, 255, 0)

    # Perform Canny edge detection
    edges = cv.Canny(grey, 30, 150)

    return edges

def show_circle():
    
    # Using the first image in here:
    global image

    # # Apply the canny detection in here:
    # edge = canny_edge(image)

    # Apply Hough Circle Transform
    circles = cv.HoughCircles(edge , cv.HOUGH_GRADIENT, dp=1, minDist=25,
                          param1=50, param2=25, minRadius=0, maxRadius=100)
    
    # Draw the detected circles on the image
    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        for (x, y, r) in circles:
            if r > 30:
                r = 21

            cv.circle(image, (x, y), r, (0, 0, 255), 3)
            # Add the (x,y) coordinates as text to the image
            text = "({},{})".format(x, y)
            cv.putText(image, text, (x - 15, y - 15), cv.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 255), 1)

            # Point out the center:
            cv.circle(image, (x, y), 3, (0, 0, 255), -1)

    return image

def find_first_position():

    # store it in the vector:
    coordinates = []

    # Apply the position of each hole
    #position 1
    coordinates.append((286,146,16,1))
    #position 2
    coordinates.append((286,202,16,2))
    #position 3
    coordinates.append((286,260,16,3))
    #position 4
    coordinates.append((286,303,16,4))
    #position 5
    coordinates.append((329,146,16,5))
    #position 6
    coordinates.append((329,203,23,6))
    #position 7
    coordinates.append((329,260,23,7))
    #position 8
    coordinates.append((329,303,16,8))
    #position 9
    coordinates.append((375,146,23,9))
    #position 10
    coordinates.append((375,203,16,10))
    #position 11
    coordinates.append((375,260,16,11))
    #position 12
    coordinates.append((375,303,16,12))

    # Return a list to store the coordinates of the centers of detected circular edges   
    return coordinates
