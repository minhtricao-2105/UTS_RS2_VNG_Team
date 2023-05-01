# This function will create a rosnode to receive the data from the realsens
import rospy
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from function import*
import imutils

img = cv.imread('Realsense/hoshino.png')

img = rescaleFrame(img,0.25)

def processing(img):
    
    grey = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    grey = cv.GaussianBlur(grey, (7,7),0)

    T, thresh = cv.threshold(grey, 75, 255, 1)

    contours = cv.findContours(thresh, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    contours = imutils.grab_contours(contours)

    new_contours = list()

    for contour in contours:
        if cv.contourArea(contour) > 150:
            new_contours.append(contour)

    print(len(new_contours))

    thresh_copy = thresh.copy()

    for contour in new_contours:
        cv.drawContours(img, [contour], -1, (255,0,0),5)

    return img

cv.imshow('hello', processing(img))

cv.waitKey(0)

cv.destroyAllWindows()