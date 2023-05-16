##  @file
#   @brief This file contains the necessary functions for implementing the computer vision aspect of the project.
#   
#   The functions in this file are used to process images and extract information that is relevant to the project.
#   The primary libraries used in this file are OpenCV and NumPy. OpenCV is used for image processing, while NumPy is used for numerical operations.
#   The functions in this file are designed to identify and track objects, detect colors, and perform other computer vision tasks that are essential to the functionality of the project.
#   
#   @author Minh Tri Cao
#
#   @date May 9, 2023

import rospy
import numpy as np
import sys
import cv2 as cv
import math
from math import sqrt, pow
from colorLibrary import*
# import imutils
from colorLibrary import*
from ur3module import *
# import moveit_commander
import moveit_msgs.msg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

##  @brief Rescales the input frame with the given scale factor
#   @param frame The input frame to be resized
#   @param scale The scale factor to resize the frame
#   @return The resized frame
##
def rescaleFrame(frame, scale):
    width     = int(frame.shape[1] * scale)
    height    = int(frame.shape[0] * scale)
    dimension = (width, height)

    return cv.resize(frame, dimension, interpolation = cv.INTER_AREA)

def setUpRobot(maxVelocity):
    # Initialize the moveit_commander and rospy nodes
    moveit_commander.roscpp_initialize(sys.argv)
    
    # Initialize the MoveIt planning scene, robot commander, and arm group
    scene = moveit_commander.PlanningSceneInterface()
    robot = moveit_commander.RobotCommander()
    arm = moveit_commander.MoveGroupCommander("manipulator")

    # Set the reference frame and end effector link
    arm.set_pose_reference_frame("base_link")
    arm.set_end_effector_link("ee_link")

    # Set the maximum velocity scaling factor
    arm.set_max_velocity_scaling_factor(maxVelocity)

    # Get the current joint values
    current_joint_values = arm.get_current_joint_values()

    return arm


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


##  @brief Crop an image using a factor and return the cropped image
#   @param img: Input image to be cropped
#   @param factor: A factor by which image will be cropped (e.g., 2 means image will be cropped to 1/2 of its original size)
#   @return Cropped image
##
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

##  @brief Applies Canny edge detection to the input image
#   @param img Input image to be processed
#   @return The processed image with Canny edge detection applied
##
def processing(img):

    img = cv.convertScaleAbs(img, alpha=1, beta=100)

    grey = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    grey = cv.GaussianBlur(grey, (3,3),1)

    grey = crop(grey,1.5)

    T, thresh = cv.threshold(grey, 40, 255, 0)

    # Perform Canny edge detection
    edges = cv.Canny(grey, 30, 150)

    return edges

##  @brief This function applies Hough Circle Transform to detect circles in an image and draw them
#   @return cv::Mat The input image with detected circles drawn on it
##
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

def control_robot():

    # store it in the vector:
    coordinates = []

    # Apply the position of each hole

    ## DONE
    #position 1
    coordinates.append((600,500,26,1))
    #position 2
    coordinates.append((500,500,16,2))
    #position 3
    coordinates.append((400,500,16,3))
    #position 4
    coordinates.append((300,500,16,4))

    ##
    #position 5
    coordinates.append((600,430,16,5))
    #position 6 (AA Battery)
    coordinates.append((500,430,23,6))
    #position 7 (AA Battery)
    coordinates.append((400,430,23,7))
    #position 8
    coordinates.append((300,430,16,8))

    #position 9 (AA Battery)
    coordinates.append((600,360,26,9))
    #position 10
    coordinates.append((500,360,16,10))
    #position 11
    coordinates.append((400,360,16,11))
    #position 12(AA Battery)
    coordinates.append((300,360,26,12))

    # Return a list to store the coordinates of the centers of detected circular edges   
    return coordinates

def find_first_position():

    # store it in the vector:
    coordinates = []

    # Apply the position of each hole
    #position 1 
    coordinates.append((270,137,26,1))
    #position 2
    coordinates.append((270,205,16,2))
    #position 3
    coordinates.append((270,268,16,3))
    #position 4 
    coordinates.append((270,358,16,4))

    #position 5
    coordinates.append((314,137,16,5))
    #position 6 (AA Battery)
    coordinates.append((314,205,23,6))
    #position 7 (AA Battery)
    coordinates.append((314,270,23,7))
    #position 8
    coordinates.append((313,333,16,8))

    #position 9 (AA Battery)
    coordinates.append((368,134,26,9))
    #position 10
    coordinates.append((366,204,16,10))
    #position 11
    coordinates.append((366,270,16,11))
    #position 12(AA Battery)
    coordinates.append((363,334,26,12))

    # Return a list to store the coordinates of the centers of detected circular edges   
    return coordinates

##  @brief Convert pixel coordinates to local camera frame coordinates
#
#   The function converts pixel coordinates to local camera frame coordinates using the camera intrinsic and extrinsic parameters.
#   First, the pixel coordinates are converted to normalized image coordinates. Then, the normalized image coordinates are converted
#   to camera coordinates using the camera height. Finally, the camera coordinates are transformed to local camera frame coordinates using the camera extrinsic parameters.
#
#   @param locations numpy array of shape (n, 2) containing the pixel coordinates to be converted
#   @return numpy array of shape (n, 3) containing the corresponding local camera frame coordinates
##

def transfer_local(locations):
    # camera intrinsic parameters
    fx = 617.306
    fy = 617.714
    cx = 327.984
    cy = 242.966

    # camera extrinsic parameters
    camera_height = 0.2  # height of camera above ground
    R = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])  # identity rotation matrix
    t = np.array([0.0, 0.0, 0.0])  # zero translation vector

    # convert pixel coordinates to normalized image coordinates
    x = (locations[:, 0] - cx) / fx
    y = (locations[:, 1] - cy) / fy

        # convert normalized image coordinates to camera coordinates
    Xc = x * camera_height / fx
    Yc = y * camera_height / fy
    Zc = camera_height * np.ones_like(x)
    
    # transform camera coordinates to local camera frame
    Rt = np.hstack((R, t[:, np.newaxis]))
    Rt = np.vstack((Rt, [0, 0, 0, 1]))
    locations_cam = np.hstack((Xc[:, np.newaxis], Yc[:, np.newaxis], Zc[:, np.newaxis], np.ones_like(x)[:, np.newaxis]))
    locations_local = np.dot(Rt, locations_cam.T).T[:, :3]

    return locations_local

def cam_move(cam,robot,T):
    cam.T = robot.fkine(robot.q)*T

def cam_to_global(pixel_x, pixel_y, camera_transform):
    # Define the camera parameters
    pixel_width = 640  # Width of the image in pixels
    pixel_height = 480  # Height of the image in pixels
    focal_length = 848  # Focal length of the camera in pixels
    
    # Compute the normalized coordinates of the point
    normalized_x = (pixel_x - pixel_width / 2) / focal_length
    normalized_y = (pixel_height / 2 - pixel_y) / focal_length

    # Compute the distance from the camera to the global point
    distance = 0.18  # Distance from the camera to the global point in meters

    # Compute the position of the point in the camera frame
    camera_position = np.array([distance * normalized_x, distance * normalized_y, distance, 1])

    # Map the point from the camera frame to the global frame
    global_position = np.matmul(camera_transform, camera_position)

    return global_position

def move_to_pin(robot, q_curr, global_position, offset_z = 0.175, turn = False):
    q_sample = [63.33, -105.04, 89.33, -75.14, -87.53, 335.72]
    q_sample = [x*pi/180 for x in q_sample]
    ee_orientation = SE3.Rt(robot.fkine(q_sample).R)

    # offset_z = 0.175
    obj_pose=SE3(global_position[0], global_position[1], global_position[2]+ offset_z)*ee_orientation

    q_guess = [1.3279389,  -1.41725404,  0.17017859, -0.62366733, -1.53202614, -0.20099896+2*pi] #guess value for IK solver
    q_goal = solve_for_valid_ik(robot=robot, obj_pose=obj_pose, q_guess = q_guess, elbow_up_request = False, shoulder_left_request= False)
    
    # if abs(q_goal[-1] - q_sample[-1]) > pi:
    #     q_goal[-1] += 2*pi
    q_goal[-1] = q_sample[-1]
    if turn: q_goal[-1] += pi/2
    # print(q_goal)

    path = rtb.jtraj(q_curr, q_goal,50)
    return path

def move_up_down(robot, q_curr, dir = 'up'):
    
    lift = 0.05 #default lifting distance

    if dir == 'up': pass
    elif dir == 'down':
        lift = -lift
    else:
        print("ERROR DIRECTION INPUT!")
        return []
    
    q_guess_2 = [ 1.32582823, -1.48473735,  1.1266249,  -1.23659264, -1.53197561,  6.08007491] #guess value for IK solver
    q_lifts = []
    step_num = 4
    step_lift = lift/(step_num-1)
    
    #generate 4 poses for lifting up/moving down
    for i in range(step_num):
        if i == 0 : 
            q_lifts.append(q_curr)
            continue
        pose_lift = robot.fkine(q_curr)*SE3(-(step_lift*i),0,0)
        q_lift = solve_for_valid_ik(robot=robot, obj_pose=pose_lift, q_guess = q_guess_2, elbow_up_request = False, shoulder_left_request= False)
        q_lift[-1] = q_lift[-1] + 2*pi
        q_lifts.append(q_lift)

    path_lift = rtb.mstraj(viapoints=np.array([q for q in q_lifts]),dt=0.01,tacc=0.05,qdmax = np.pi)

    return path_lift

def add_trajectory(total_path, goal, execution_time):

    duration_seconds = 5.0

    for i in range(len(total_path)):

        point = JointTrajectoryPoint()

        point.positions = total_path[i]

        point.time_from_start = rospy.Duration.from_sec((i+1)*(duration_seconds/len(total_path))) + rospy.Duration.from_sec(execution_time + 1)

        goal.trajectory.points.append(point)

    return goal
