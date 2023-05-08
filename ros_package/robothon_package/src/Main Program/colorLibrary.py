import cv2 as cv
import numpy as np
from function import*

def yellow_detection(img):
    #Change the brightness of a figure:
    # Can adjust beta from -100 to 100 (negative is darker and positive is lighter)
    img = cv.convertScaleAbs(img, alpha=1, beta=-40)

    #Convert to HSV Format:
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

    # Create a threshhold for the yellow
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([30, 255, 255])

    #Create a mask
    mask = cv.inRange(hsv, lower_yellow, upper_yellow)

    #File a contours in the mask
    contours, hierarchy = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    
    return contours

def orange_detection(img):
    #Change the brightness of a figure:
    # Can adjust beta from -100 to 100 (negative is darker and positive is lighter)
    img = cv.convertScaleAbs(img, alpha=1, beta=-20)

    #Convert to HSV Format:
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

    lower_orange = np.array([5, 100, 100])
    upper_orange = np.array([15, 255, 255])

    #Create a mask
    mask = cv.inRange(hsv, lower_orange, upper_orange)

    #File a contours in the mask
    contours, hierarchy = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    
    return contours

def green_detection(img):
    #Change the brightness of a figure:
    # Can adjust beta from -100 to 100 (negative is darker and positive is lighter)
    img = cv.convertScaleAbs(img, alpha=1, beta=-69)

    #Convert to HSV Format:
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

    #Detect Green Color: BGR
    lower_green = np.array([50, 50, 50])
    upper_green = np.array([70, 255, 255])

    #Create a mask
    mask = cv.inRange(hsv, lower_green, upper_green)

    # mask = cv.medianBlur(mask, 3)

    #File a contours in the mask
    contours, hierarchy = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    
    return contours

def battery_detection(img):
    
    # img = cv.convertScaleAbs(img, alpha=1, beta=-69)

    #Convert to HSV Format:
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)


    # Define lower and upper threshold for green color in HSV
    lower_green = np.array([50, 50, 50])
    upper_green = np.array([70, 255, 255])

    mask = cv.inRange(hsv, lower_green, upper_green)

    # Apply morphology techniques to remove noise and fill holes
    kernel = np.ones((5,5), np.uint8)
    mask = cv.erode(mask, kernel, iterations=1)
    mask = cv.dilate(mask, kernel, iterations=1)

    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    return contours

