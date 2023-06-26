##  @file
#   @brief This file contains the necessary functions for implementing the computer vision aspect of the project.
#   
#   The functions in this file are used to process images and extract information that is relevant to the project.
#   The primary libraries used in this file are OpenCV and NumPy. OpenCV is used for image processing, while NumPy is used for numerical operations.
#   The functions in this file are designed to identify and track objects, detect colors, and perform other computer vision tasks that are essential to the functionality of the project.
#   
#   @author Minh Tri Cao 
#   @author Ho Minh Quang Ngo

#   @date May 9, 2023

# Import Library:
import numpy as np
import cv2 as cv
import rospy, time, actionlib, moveit_msgs.msg, moveit_commander, math, sys, imutils, swift
import roboticstoolbox as rtb 
# import math
from numpy import linalg
# User Library:
from colorLibrary import*
from ur3module import *
from gripperFunction import*

# Python Library:
from math import sqrt, pow, pi, degrees
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from sensor_msgs.msg import Image as SensorImage
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String, Float32MultiArray, Int32
from cv_bridge import CvBridge
from spatialmath import SE3
from onrobot_rg_control.msg import OnRobotRGOutput

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

def crop_2(img,factor):
    # Get the dimensions of the image
    height, width = img.shape[:2]

    # Calculate the dimensions of the rectangle to crop
    w = int(width / factor*1.5)
    h = int(height / factor*1.25)


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

def control_robot():

    # store it in the vector:
    coordinates = []

    # Apply the position of each hole
    #position 1
    coordinates.append((600,550,26,1))
    #position 2
    coordinates.append((500,550,16,2))
    #position 3
    coordinates.append((400,550,16,3))
    #position 4
    coordinates.append((300,550,26,4))

    ##
    #position 5
    coordinates.append((600,480,16,5))
    #position 6 (AA Battery)
    coordinates.append((500,480,23,6))
    #position 7 (AA Battery)
    coordinates.append((400,480,23,7))
    #position 8
    coordinates.append((300,480,16,8))

    #position 9 (AA Battery)
    coordinates.append((600,410,16,9))
    #position 10
    coordinates.append((500,410,16,10))
    #position 11
    coordinates.append((400,410,16,11))
    #position 12(AA Battery)
    coordinates.append((300,410,16,12))

    # Return a list to store the coordinates of the centers of detected circular edges   
    return coordinates

def find_first_position():

    # store it in the vector:
    coordinates = []

    # Apply the position of each hole
    #position 1 
    coordinates.append((245,137,26,1))
    #position 2
    coordinates.append((245,203,16,2))
    #position 3
    coordinates.append((245,268,16,3))
    #position 4 
    coordinates.append((245,333,26,4))

    #position 5
    coordinates.append((290,137,16,5))
    #position 6 (AA Battery)
    coordinates.append((290,203,23,6))
    #position 7 (AA Battery)
    coordinates.append((290,278,23,7))
    #position 8
    coordinates.append((290,333,16,8))

    #position 9 (AA Battery)
    coordinates.append((338,137,16,9))
    #position 10
    coordinates.append((338,203,16,10))
    #position 11
    coordinates.append((338,278,16,11))
    #position 12(AA Battery)
    coordinates.append((338,328,16,12))

    # Return a list to store the coordinates of the centers of detected circular edges   
    return coordinates

def cam_move(cam,robot,T):
    """
    Fuction to update pose of the object following robot's end-effector
    """
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

def rotate_ee(q_curr, turn = 0.0):
    q_curr[-1] += pi - pi/4.6 # correction
    q_goal = list(q_curr)
    if turn!= 0: q_goal[-1] += turn * pi/180
    path = rtb.jtraj(q_curr, q_goal, 30)
    return path

def move_to_pin(robot, q_curr, global_position, offset_z = 0.175, turn = 0):
    q_sample = [63.33, -105.04, 89.33, -75.14, -87.53, 335.72]
    q_sample = [x*pi/180 for x in q_sample]
    ee_orientation = SE3.Rt(robot.fkine(q_sample).R)

    # offset_z = 0.175
    obj_pose=SE3(global_position[0], global_position[1], global_position[2]+ offset_z)*ee_orientation

    q_guess = [1.3279389,  -1.41725404,  0.17017859, -0.62366733, -1.53202614, -0.20099896+2*pi] #guess value for IK solver
    q_goal = solve_for_valid_ik(robot=robot, obj_pose=obj_pose, q_guess = q_guess, elbow_up_request = False, shoulder_left_request= False)
    
    q_goal[-1] = q_sample[-1]
    if turn!=0: q_goal[-1] += turn * pi/180
    # print(q_goal)

    path = rtb.jtraj(q_curr, q_goal,30)
    return path

def move_up_down(robot, q_curr, dir = 'up', lift = 0.03,  q_guess_2 = [1.32582823, -1.48473735,  1.1266249,  -1.23659264, -1.53197561,  6.08007491]):
    
    # lift = 0.03 #default lifting distance
    if dir == 'up': pass
    elif dir == 'down':
        lift = -lift
    else:
        print("ERROR DIRECTION INPUT!")
        return []
    
    # q_guess_2 = [ 1.32582823, -1.48473735,  1.1266249,  -1.23659264, -1.53197561,  6.08007491] #guess value for IK solver
    q_lifts = []
    step_num = 4
    step_lift = lift/(step_num-1)
    
    #generate 4 poses for lifting up/moving down
    for i in range(step_num):
        if i == 0 : 
            q_lifts.append(q_curr)
            continue
        # pose_lift = robot.fkine(q_curr)*SE3(-(step_lift*i),0,0)
        pose_lift = SE3(0,0,step_lift*i) * robot.fkine(q_curr)
        q_lift = solve_for_valid_ik(robot=robot, obj_pose=pose_lift, q_guess = q_guess_2, elbow_up_request = False, shoulder_left_request= False)
        q_lift[-1] = q_lift[-1] + 2*pi
        q_lifts.append(q_lift)

    path_lift = rtb.mstraj(viapoints=np.array([q for q in q_lifts]),dt=0.01,tacc=0.05,qdmax = np.pi)

    return path_lift

# Use test_coin_move to get data
def flick_coin(q_at_coin, q_flick):
    path_flick = rtb.jtraj(q_at_coin, q_flick, 20)
    return path_flick    

def RMRC_motion(robot, path, threshold = 0.01):
    pose_list = [robot.fkine(q) for q in path.q]
    
    for pose in pose_list:
        print("Step:", i)
        arrived = False
        while np.linalg.norm(robot.fkine(robot.q).A[0:3,3] - pose.A[0:3,3]) > threshold:
            v, arrived = rtb.p_servo(robot.fkine(robot.q), pose, 1.5, threshold=threshold)
            robot.qd = np.linalg.pinv(robot.jacobe(robot.q)) @ v
            # cam_move(cam, robot, TCR)
            # cam_move(gripper, robot, TGR)
            # env.add(collisionObj.Sphere(radius=0.005, pose = robot.fkine(robot.q) * SE3(0.2, 0,0),color = (0.1,0.5,0.1,1)))
            # env.step(dt)
        i+=1
    print("FINAL ERROR: ", np.linalg.norm(robot.fkine(robot.q).A[0:3,3] - pose_list[-1].A[0:3,3]))

def move_simulation_robot(robot, path, env, dt, gripper = False, cam = False, pin = False, TCR = SE3(0,0,0), TGR = SE3(0,0,0), TCP = SE3(0,0,0)):
    dt = 0.02
    for q in path:
        robot.q = q
        if not isinstance(cam,bool): cam_move(cam,robot,TCR)
        if not isinstance(gripper,bool): cam_move(gripper,robot,TGR)
        if not isinstance(pin,bool): cam_move(pin, robot, TCP)
        env.step(dt)

def add_trajectory(total_path, goal, execution_time, duration_seconds = 1.0):

    for i in range(len(total_path)):

        point = JointTrajectoryPoint()
        
        point.positions = total_path[i]
        
        point.time_from_start = rospy.Duration.from_sec((i+1)*(duration_seconds/len(total_path))) + rospy.Duration.from_sec(execution_time + 1)

        goal.trajectory.points.append(point)

    return goal

def set_up_action_client():
    
    # Create a JointTrajectory message
    joint_traj = JointTrajectory()

    # Set the joint names
    joint_traj.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

    # Fill in the header
    joint_traj.header.frame_id = "base_link"

    # Create a FollowJointTrajectoryGoal message:
    goal = FollowJointTrajectoryGoal()
    
    # Set the joint names:
    goal.trajectory.joint_names = joint_traj.joint_names

    # Set Sequence
    
    goal.trajectory.header.seq = 1

    # Set time stamp
    goal.trajectory.header.stamp = rospy.Time.now()

    # Set Tolerance
    goal.goal_time_tolerance = rospy.Duration.from_sec(0.05)

    return goal

def battery_graph_search(arr):

    return arr

def hole_cordinate():
    # store it in the vector:
    coordinates = []

    # Apply the position of each hole
    ## DONE
    # position 1
    coordinates.append((242,35,1.0))

    #position 2
    coordinates.append((242,135,1.0))

    #position 3
    coordinates.append((630,35,1.0))

    #position 4
    coordinates.append((630,135,1.0))

    # position 5
    coordinates.append((660,820,0.0))

    return coordinates

def send_action_client(arrived, path, goal, start_time, client, speed = 1):
    
    end_time = time.perf_counter()

    execution_time = end_time - start_time

    while arrived == True:
        
        add_trajectory(path, goal, execution_time, speed)

        client.send_goal(goal)

        client.wait_for_result()

        result = client.get_result()

        arrived = False

def sort_battery(location_array):
    new = []
    for i in location_array:
        if i[2] == 0.0:
            for j in location_array:
                if j[1] == i[1]:
                    if abs(j[0] - i[0]) == 100 and j[2] == 1:
                        new.append(j)
                        break
                else:
                    continue
            new.append(i)
            break
        else:
            continue

    for k in location_array:
        if k not in new:
            new.append(k)

    return new

def combine_trajectory(q1, q2):
    
    total_q = []

    for q in q1.q:
        total_q.append(q)
    for q in q2.q:
        total_q.append(q)

    return total_q


def combine_trajectories(qlist):
    total_q = []
    for trajectory in qlist:
       for q in trajectory.q:
            total_q.append(q)
    return total_q
       
            

def get_task2_param(robot, joint_home_radian, lift_coin_up, T_coin, is_battery_there = '12', num_AA = 2):
    # '1': only battery 1,'2': only battery 2, '12': both batteries
    
    TCC = T_coin

    # Data belong to battery 1:
    battery_1_position = [45.52, -68.18, 27.30, -47.27, -87.16, 315.94]
    battery_1_position = [math.radians(joint_home_degree) for joint_home_degree in battery_1_position]

    # Data belong to battery 2:
    battery_2_position = [56.25, -75.63, 41.35, -57.66, -89.34, 324.05]
    battery_2_position = [math.radians(joint_home_degree) for joint_home_degree in battery_2_position]

    # Dropping data
    #-> For coin
    q_drop = [99.21, -67.98, 25.29, -53.84, -85.79, 351.35]
    q_drop_radian = [math.radians(joint_home_degree) for joint_home_degree in q_drop]

    #-> For batteries
    q_droppin_1 = [78.1, -96.71, 64.42, -58.05, -88.22, 252.02]
    q_droppin_1 = [math.radians(joint_home_degree) for joint_home_degree in q_droppin_1]
    q_droppin_2 = [70.67, -97.03, 61.32, -54.39, -88.19, 242.99]
    q_droppin_2 = [math.radians(joint_home_degree) for joint_home_degree in q_droppin_2]    

    if num_AA >= 4:
        q_droppin_1 = list(q_drop_radian)
        q_droppin_2 = list(q_drop_radian)
    elif num_AA == 3 and (is_battery_there == '1' or is_battery_there == '2'):
        q_droppin_1 = list(q_droppin_2)
    elif num_AA == 3 and is_battery_there == '12':
        q_droppin_1 = list(q_droppin_2)
        q_droppin_2 = list(q_drop_radian)    

    # ROUTE FOR MOVING BATTERY 1:
    #-> From coin position to picking position
    pick1_way1 = [36.05, -63.35, 24.29, -48.96, -86.55, 217.81]
    pick1_way1 = [math.radians(joint_home_degree) for joint_home_degree in pick1_way1]
    pick1_way2 = [26.63, -49.31, 19.13, -27.04, -68.63, 203.57]
    pick1_way2 = [math.radians(joint_home_degree) for joint_home_degree in pick1_way2]

    pick1_path1 = rtb.mstraj(viapoints=np.array([q_drop_radian, pick1_way1, pick1_way2]),dt=0.01,tacc=0.05,qdmax = pi)

    #-> From picking position taking out
    pick1_way3 = [56.74, -86.43, 46.07, -51.44, -88.32, 234.33]
    pick1_way3 = [math.radians(joint_home_degree) for joint_home_degree in pick1_way3]

    path_up_pickway1 = move_up_down(robot, pick1_path1.q[-1], lift = 0.005) 
    path_to_pickway1 = rtb.jtraj(path_up_pickway1.q[-1], pick1_way3, 30)

    pick1_path2 = combine_trajectories([path_up_pickway1, path_to_pickway1])

    #-> To dropping position 1
    path1_droppin1 = rtb.jtraj(pick1_path2[-1], q_droppin_1, 30)
    path1_droppin2 = move_up_down(robot, path1_droppin1.q[-1], 'down', lift = 0.02)

    path_droppin_bat1 = combine_trajectories([path1_droppin1, path1_droppin2])

    #-> Combine route for battery 1
    path1_instruction ={'picking': pick1_path1.q, 'taking': pick1_path2, 'dropping': path_droppin_bat1}

    # ROUTE FOR MOVING BATTERY 2:
    #-> From current position to picking position
    pick2_way1 = [94.24, -79.7, 61.95, -74.67, -118.79, 84.73 + 360]
    pick2_way1 = [math.radians(joint_home_degree) for joint_home_degree in pick2_way1]

    pick2_way2 = [82.72, -69.40, 56.97, -72.54, -120.00, 82.96 + 360]
    pick2_way2 = [math.radians(joint_home_degree) for joint_home_degree in pick2_way2]

    q_pick2_start = path_droppin_bat1[-1] # default if is_batery_there == '12' or '1'
    if is_battery_there == '2': # if only battery 2 is put in, taking from the coin drop position
        q_pick2_start = q_drop_radian

    pick2_path1 = rtb.mstraj(viapoints=np.array([q_pick2_start, joint_home_radian, pick2_way1, pick2_way2]),dt=0.01,tacc=0.05,qdmax = pi)

    #-> From picking position taking out
    drag_pose = SE3(-0.06, 0, 0.005) * robot.fkine(pick2_way2)
    q_sol_1 = solve_for_valid_ik(robot= robot, obj_pose = drag_pose, q_guess = pick2_way2)
    q_sol_1[-1] = pick2_way2[-1]

    down_pose = SE3(0.008, 0, -0.0068) * robot.fkine(q_sol_1)
    q_sol_2 = solve_for_valid_ik(robot= robot, obj_pose = down_pose, q_guess = list(q_sol_1))
    q_sol_2[-1] = q_sol_1[-1]

    final_ee_pose = SE3(0.058, 0, 0.06)*robot.fkine(q_sol_2)
    q_sol_3 = solve_for_valid_ik(robot= robot, obj_pose = final_ee_pose, q_guess = list(q_sol_2))
    q_sol_3[-1] = q_sol_2[-1]

    pick2_path2 = rtb.mstraj(viapoints=np.array([pick2_way2, q_sol_1, q_sol_2, q_sol_3]),dt=0.01,tacc=0.05,qdmax = pi) 

    #->  To dropping position 2
    path2_droppin1 = rtb.jtraj(pick2_path2.q[-1], q_droppin_2, 30)
    path2_droppin2 = move_up_down(robot, path2_droppin1.q[-1], 'down', lift = 0.04)

    path_droppin_bat2 = combine_trajectories([path2_droppin1, path2_droppin2])

    #-> Combine route for battery 2
    path2_instruction ={'picking': pick2_path1.q, 'taking': pick2_path2.q, 'dropping': path_droppin_bat2}

    # LISTS FOR HOLDING DATA
    flip_coin_data = []
    rot_coin = []
    path_instruction = []

    if is_battery_there == '1': # if only battery 1 is put in 
        flip_coin_data.append(battery_1_position)
        rot_coin.append([-0.002,0,0.0043,14])
        path_instruction.append(path1_instruction)

    if is_battery_there == '2': # if only battery 2 is put in
        flip_coin_data.append(battery_2_position)
        rot_coin.append([0.002,0,0.0043,-14])
        path_instruction.append(path2_instruction)

    if is_battery_there == '12': # if both batteries are put in
        flip_coin_data.append(battery_1_position)
        flip_coin_data.append(battery_2_position)
        rot_coin.append([-0.002,0,0.0043,14])
        rot_coin.append([0.002,0,0.0043,-14])
        path_instruction.append(path1_instruction)
        path_instruction.append(path2_instruction)

    # ROUTE FOR FLICKING BATTERY:
    flick_instruction = []
    path_flick_instruction = {}

    for i in range(len(flip_coin_data)):
        q_ini = lift_coin_up
        if i == 1:  q_ini = flick_instruction[0]['flick'][-1]

        #-> From current position to above battery 
        battery_position = flip_coin_data[i]
        path_to_battery = rtb.jtraj(q_ini, battery_position, 30) 
        path_down_battery = move_up_down(robot, path_to_battery.q[-1], 'down', lift = 0.05)
        
        path_flick_instruction['reach'] = combine_trajectories([path_to_battery, path_down_battery])

        ## Start flipping the battery
        q_start = list(path_down_battery.q[-1])
        coin_T = robot.fkine(path_down_battery.q[-1]) * TCC
        coin_desired_T = transl(rot_coin[i][0],0,0.0043) @ coin_T.A @ troty(rot_coin[i][3]*pi/180) # apply lifting and rotation
        
        q_coin_end = solve_for_valid_ik(robot=robot, obj_pose= SE3(coin_desired_T), relative_pose = TCC.inv(), q_guess = q_start)
        q_coin_end[-1] = q_start[-1]

        # Create a trajectory to flip the battery
        path_coin_rot = rtb.jtraj(q_start, q_coin_end, 20)
        path_coin_up = rtb.jtraj(path_coin_rot.q[-1], battery_position, 20)
        
        path_flick_instruction['flick'] = combine_trajectories([path_coin_rot, path_coin_up])
        
        # Add instruction to list of instruction
        flick_instruction.append(path_flick_instruction.copy())

    return flick_instruction, path_instruction
