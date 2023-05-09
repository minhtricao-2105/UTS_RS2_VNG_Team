import cv2 as cv
import numpy as np
from function import*

##  @file
#   @brief This file provides functions for detecting different colors in an image.
#   
#   It provides functions for detecting yellow, orange, and green colors, as well as
#   detecting batteries in an image. These functions use OpenCV to convert images to
#   the HSV color space and apply thresholds to isolate the desired color. Contours
#   are then found in the resulting mask to identify the object(s) of interest.
#
#   @author Minh Tri Cao
#   @date May 9, 2023


##
#   @brief  Detect yellow contours in an image
#   This function changes the brightness of the input image, converts it to HSV format, creates a threshold for yellow color, creates a mask, and finds contours in the mask.
#   @param img The input image
#   @return A list of contours

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


##
#   @brief  Detect orange contours in an image
#   This function changes the brightness of the input image, converts it to HSV format, creates a threshold for orange color, creates a mask, and finds contours in the mask.
#   @param img The input image
#   @return A list of contours

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


