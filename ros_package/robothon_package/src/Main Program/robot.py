#!/usr/bin/env python3

##  @file
#   @brief This file provides functions for controlling the robot.
#   
#
#   @author Minh Tri Cao
#   @author Ho Minh Quang Ngo
#
#   @date May 9, 2023


#Library Belongs to ROS:
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32
import moveit_commander
import moveit_msgs.msg
import numpy as np
from function import *

#Library Belongs to Robot Toolbox:
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import roboticstoolbox as rtb 
from spatialmath import SE3
from math import pi
from math import degrees
import swift
import time
from ur3module import *

#Library Belongs to Gripper
from onrobot_rg_control.msg import OnRobotRGOutput
from gripperFunction import *

# Array to store the position of the yellow container (AAA Batteries) from the Vision Nodes
colour_array_yellow = []

# Array to store the position of the orange container (AA Batteries) from the Vision Nodes
colour_array_orange = []

# Array to store the position of the batteries from the Vision nodes
location_array = []

# Array to store the transfer from pixel to local frame of camera:
transfer = []

# Create a global variable for the robot to wait for the computer vision part complete:
running_ = False

# Create a flag variable moveit commander
moveIt = False

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

##  @brief Callback function for receiving data from a ROS message.
#
#   This function is called when a ROS message is received by the subscriber. The data from the message is stored in a global variable, and a pixel-to-local-frame transfer value is also calculated and stored in another global variable.
#
#   @param msg The ROS message containing data to be processed.
#
def callback_2(msg):
    
    #Call out the global variable to store the data from the CV nodes:
    global location_array

    # Extract the array from the message data field
    location_array = [msg.data[i:i+3] for i in range(0, len(msg.data), 3)]

    # Try to print out the location of the batteries from the computer vision node:
    # rospy.loginfo('Location array: %s', location_array)


##  @brief Callback function to receive color data from CV nodes.
#   
#   This function receives color data from CV nodes and stores them in global variables.
#   The first data received will be stored in the yellow color array, and the second data will be stored in the orange color array.
#   @param msg A message containing color data received from CV nodes.
def callback_3(msg):
    
    #Call out the global variable to store the data from the CV nodes:
    global colour_array_orange
    global colour_array_yellow

    # At the begining, flag will equal to 0
    flag = 0

    # The first one will be the yellow container:
    if flag == 0:
        colour_array_yellow = [msg.data[i:i+2] for i in range(0, len(msg.data), 2)]

        # rospy.loginfo('Yellow array: %s', colour_array_yellow)

        flag += 1

    # The second one will be the orange container:
    if flag == 1:
        colour_array_orange = [msg.data[i:i+2] for i in range(0, len(msg.data), 2)]
        
        # rospy.loginfo('Orange array: %s', colour_array_orange)
   
rospy.init_node('robot_node')

pub_1 = rospy.Publisher('Move_home',Int32,queue_size=1)

############## BELONG TO THE ROBOT OVER HERE ###########################

while moveIt == False:
    rospy.loginfo("[UPDATE]: Moving Robot to Home!!")
    moveit_commander.roscpp_initialize(sys.argv)
    joint_home_degree = [57.11, -106.32, 56.84, -44.7, -85.81, 328.40]
    joint_home_radian = [math.radians(joint_home_degree) for joint_home_degree in joint_home_degree]
    arm = setUpRobot(0.5)
    arm.go(joint_home_radian)
    moveIt = True
    mess = Int32()
    mess.data = 1
    pub_1.publish(mess)
    break

# Subcribe Declaration:
subscriber_2 = rospy.Subscriber('Image_Data', Float32MultiArray, callback_2)

subscriber_3 = rospy.Subscriber('Color_data', Float32MultiArray, callback_3)

pub = rospy.Publisher('OnRobotRGOutput', OnRobotRGOutput, queue_size=1)

# while len(location_array) == 0:
#     rospy.loginfo("[WARNING]: Waiting for Computer Vision Part")
#     if(len(location_array) != 0):
#         rospy.loginfo("[UPDATE]: Robot recevieved data from Vision Node!")
#         rospy.loginfo('[UPDATE]: BATTERIES LOCATION: %s', location_array)
#         running_ = True
#         break

rospy.loginfo("Wait 5 seconds")

rospy.sleep(5)

hole = []
duplicate = False
position = hole_cordinate()

location_array = sort_battery(location_array)

print("Battery Order: ", sort_battery(location_array))

## DROPPING ORDER ---------------------------------------------- #
for i in location_array:
    if i[2] == 0.0:
        for j in position: 
            if j[2] == 0.0  and j not in hole:
                hole.append(j)
                break
            else:
                continue
    elif i[2] == 1.0:
        for j in position: 
            if j[2] == 1.0  and j not in hole:
                hole.append(j)
                break
            else:
                continue
  
rospy.loginfo('Hole Location: %s', hole)
## DROPPING ORDER - DONE ---------------------------------------- #   

running_ = True
## MAIN EXECUTION ------------------------------------------------------------------------------------------------------------ #   
while running_ == True:
    ## INITIAL SET UP -------------------------------------------------------------------------------------------------------- #
    goal = set_up_action_client()

    # This allows for the time taken to send the message. If the network is fast this could be reduced
    buffer_seconds = 1

    # This is how many seconds the movement will take
    duration_seconds = 5

    # Call the client
    # client = actionlib.SimpleActionClient('eff_joint_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    client = actionlib.SimpleActionClient('scaled_pos_joint_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    
    start_time = time.perf_counter()

    # Enviroment
    env = swift.Swift()
    env.launch(realtime=True)
    dt = 0.05
    env._send_socket
    # Clone
    robot = rtb.models.UR3()
  
    # Gripper
    gripper_path = "/home/minhtricao/robothon2023/RTB-P Test Files/SomeApplications/CAMGRIPPER.STL"
    gripper = collisionObj.Mesh(filename=gripper_path,pose = SE3(0,0,0),scale=[0.001, 0.001, 0.001],color = [0.5,0.1,0.1,1])
    TGR = SE3.Rx(pi)*SE3(0,-0.105,-0.175) # gripper pose in ee frame

    # Camera
    cam = collisionObj.Cuboid(scale=[0.03,0.06,0.015], pose = SE3(0,0,0), color = [0.5,0.5,0.1,1])
    TCR = SE3(0.085,0,0.09)*SE3.Ry(pi/2) # cam pose in ee frame

    # Coin
    coin = collisionObj.Cylinder(radius = 0.01, length= 0.005, pose = SE3(0, 0, 0), color = [0.3,0.3,0.1,1])
    TCC = SE3(0.23,0,0) * SE3.Rx(pi/2) #coin pose in ee frame
    
    # Set initial position
    q_deg = [57.11, -106.32, 56.84, -44.7, -85.81, 328.40]
    q_start = [x*pi/180 for x in q_deg]
    robot.q = q_start

    # Add to environment
    env.add(robot)
    env.add(cam)
    env.add(gripper)
    env.add(coin)

    pin = [] # List of pins
    TCP = SE3(0.23,0,0)*SE3.Ry(pi/2) # pin pose in ee frame

    cam_move(cam, robot, TCR)
    cam_move(gripper, robot, TGR)
    camera_transform = cam.T
    
    for i in range(len(location_array)):
        global_pos = cam_to_global(location_array[i][0],location_array[i][1], camera_transform)
        color = (0.2,0.2,0.5,1)
        if location_array[i][2] == 0 : color = (0.2,0.5,0.2,1)
        pin.append(collisionObj.Cylinder(radius = 0.005, length= 0.06, pose = SE3(global_pos[0], global_pos[1], global_pos[2]-0.05), color = color))
        env.add(pin[i])
    ## INITIAL SET UP- DONE ------------------------------------------------------------------------------------------------------------------------------------ #
    
    ## CODE RUNNING ROBOT - TASK IN GREEN BLOCK ---------------------------------------------------------------------------------------------------------------- #
    for i in range(len(location_array)):
        
        ## GRIPPER OPEN ---------------------------------- #
        while not rospy.is_shutdown():
            pub.publish(moveGripperToPosition(400, 280))
            rospy.sleep(0.25)
            break
        ## GRIPPER OPEN - DONE ---------------------------------- #
        
        ## 0. HOMING SET UP --------------------------------- #
        # First position of the robot:
        q_deg = [57.11, -106.32, 56.84, -44.7, -85.81, 328.40]
        q_start = [x*pi/180 for x in q_deg]
        robot.q = q_start
        
        cam_move(cam, robot, TCR)
        cam_move(gripper, robot, TGR)
        camera_transform = cam.T

        # Define the point in Pixel frame
        pixel_x_1 = location_array[i][0]
        pixel_y_1 = location_array[i][1]

        global_position_1 = cam_to_global(pixel_x_1,pixel_y_1, camera_transform)

        speed = 1 # Default speed of real robot
        turn = 0 # Default turn of ee
        ## 0. HOMING SET UP - DONE ----------------------------- #

        ## 1. MOVE TO BATTERY POSITION IN GREEN BLOCK ----------------------------------------------------------------------------------- #
        # Move to the position of the battery
        if location_array[i][2] == 0.0: #rotate the ee first
            q_now = list(robot.q)
            q_now[-1] -= 5*pi/6.5
            rot_ee = rotate_ee(q_now, turn = 90)
            move_simulation_robot(robot = robot, path= rot_ee.q, env= env, dt = dt, gripper = gripper, cam = cam, TCR = TCR, TGR = TGR)
            
            arrived = True
            send_action_client(arrived, rot_ee.q, goal, start_time, client, speed = speed)
            
            q_start = rot_ee.q[-1]
            turn = 90

        path_1 = move_to_pin(robot, q_start, global_position_1, turn = turn)
        arrived = False
        move_simulation_robot(robot = robot, path= path_1.q, env= env, dt = dt, gripper = gripper, cam = cam, TCR = TCR, TGR = TGR)

        arrived = True

        # Send the trajectory to the robot via the action client command:
        if location_array[i][2] == 0.0:
            speed = 3

        send_action_client(arrived, path_1.q, goal, start_time, client, speed = speed)

        # Close the gripper:
        while not rospy.is_shutdown():
            pub.publish(moveGripperToPosition(400, 50))
            rospy.sleep(0.25)
            break
        ## 1. MOVE TO BATTERY POSITION IN GREEN BLOCK- DONE ---------------------------------------------------------------------------------- #
            
        ## 2. MOVE UP VERTICALLY---------------------------------------------------------------------------------------------------------------------------------------------#
        rospy.loginfo("[UPDATE]: ROBOT'S MOVING UP")
        
        robot.q = path_1.q[-1]
        path_2 = move_up_down(robot, path_1.q[-1], lift=0.06)
        arrived = False
        #Simulate the robot
        move_simulation_robot(robot = robot, path= path_2.q, env= env, dt = dt, gripper = gripper, cam = cam, pin = pin[i], TCR = TCR, TGR = TGR, TCP = TCP)
        
        arrived = True

        goal.trajectory.points.clear()

        # Send the command to the robot via the action client commander:
        send_action_client(arrived, path_2.q, goal, start_time, client)
        
        # Rotate back if the pin is AAA
        rot = []
        if location_array[i][2] == 0:
            q_now = list(robot.q)
            q_now[-1] += pi/2 + pi/13
            rot = rotate_ee(q_now, turn = -90)
            move_simulation_robot(robot = robot, path = rot.q, env= env, dt = dt, gripper = gripper, cam = cam, pin = pin[i], TCR = TCR, TGR = TGR, TCP = TCP)
            arrived = True
            send_action_client(arrived, rot.q, goal, start_time, client)   
        ## 2. MOVE UP VERTICALLY-DONE --------------------------------------------------------------------------------------------------------------------------------------------#
        
        ## 3. MOVE TO THE HOLE POSITION ------------------------------------------------------------------------------------------------------------------------------------------#
        rospy.loginfo("[UPDATE]: MOVING TO ANOTHER POSITION")
        
        goal.trajectory.points.clear()

        pixel_x_2 = hole[i][0]
        pixel_y_2 = hole[i][1]

        global_position_2 = cam_to_global(pixel_x_2,pixel_y_2, camera_transform)

        q_start = path_2.q[-1]
        
        if location_array[i][2] == 0:
            q_start = rot.q[-1]
        
        robot.q = q_start

        path_3 = move_to_pin(robot, q_start, global_position_2,0.225)    

        arrived = False
        
        move_simulation_robot(robot = robot, path= path_3.q, env= env, dt = dt, gripper = gripper, cam = cam, pin = pin[i], TCR = TCR, TGR = TGR, TCP = TCP)
        
        arrived = True

        # Send command to the robot via action client commander:
        send_action_client(arrived, path_3.q, goal, start_time, client)
        ## 3. MOVE TO THE HOLE POSITION - DONE ------------------------------------------------------------------------------------------------------------------------------------------#
        
        ## 4. MOVE DOWN AND DROP BATTERY -------------------------------------------------------------------------------------------------------------------------------------------------#
        q_start = path_3.q[-1]
        
        # Move down
        if hole[i][0] == 630 and hole[i][1] == 135:
             path_4 = move_up_down(robot, q_start,'down',lift = 0.028)
        else:
            path_4 = move_up_down(robot, q_start,'down',lift = 0.035)

        move_simulation_robot(robot = robot, path= path_4.q, env= env, dt = dt, gripper = gripper, cam = cam, pin = pin[i], TCR = TCR, TGR = TGR, TCP = TCP)
        
        arrived = True

        # Send command to the robot via action client commander
        send_action_client(arrived, path_4.q, goal, start_time, client)

        # Open the gripper
        while not rospy.is_shutdown():
            pub.publish(moveGripperToPosition(400, 180))
            rospy.sleep(0.25)
            break
        ## 4. MOVE DOWN AND DROP BATTERY - DONE -------------------------------------------------------------------------------------------------------------------------------------------------#

        ## 5. HOMING ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------#
        q_current = robot.q
        if abs(q_current[-1] - joint_home_radian[-1]) > pi/2:
            q_current[-1] += pi - pi/4.6

        # Path to go home 
        path_back = rtb.jtraj(q_current, joint_home_radian, 40)

        move_simulation_robot(robot = robot, path = path_back.q, env= env, dt = dt, gripper = gripper, cam = cam, TCR = TCR, TGR = TGR)
        
        arrived = True

        send_action_client(arrived, path_back.q, goal, start_time, client)
        ## 5. HOMING - DONE---------------------------------------------------------------------------------------------------------------------------------------------------------------------------#
    ## CODE RUNNING ROBOT - TASK IN GREEN BLOCK - DONE --------------------------------------------------------------------------------------------------------------- #
    
    ## CODE RUNNING ROBOT - COIN TASK ---------------------------------------------------------------------------------------------------------------------------------#
    ## Order:
    ### 1. Move to coin
    ## Close gripper
    while not rospy.is_shutdown():
        pub.publish(moveGripperToPosition(400,400))
        rospy.sleep(0.25)
        break

    q_coin_position = [33.95, -75.09, 53.02, -67.54, -89.51, 304.66]
    q_coin_position = [math.radians(joint_home_degree) for joint_home_degree in q_coin_position]

    # Create a trajectory to the coin_position:
    path_to_coin_position = rtb.jtraj(joint_home_radian, q_coin_position, 40)

    # Move Robot Down:
    path_move_down2_pick_coin = move_up_down(robot, path_to_coin_position.q[-1], 'down', lift=0.045, q_guess_2 = list(path_to_coin_position.q[-1]))

    # Combine two path:
    # path_2_pick_coin = combine_trajectory(path_to_coin_position, path_move_down2_pick_coin)
    path_2_pick_coin = combine_trajectories([path_to_coin_position, path_move_down2_pick_coin])

    move_simulation_robot(robot = robot, path = path_2_pick_coin, env= env, dt = dt, gripper = gripper, cam = cam, TCR = TCR, TGR = TGR)
    arrived = True
    send_action_client(arrived, path_2_pick_coin, goal, start_time, client)
    
    ## Close gripper
    while not rospy.is_shutdown():
        pub.publish(closeGripper(400))
        rospy.sleep(0.25)
        break
    
    ### 2. Lift coin up
    lift_coin_up = list(reversed(path_move_down2_pick_coin.q))
    move_simulation_robot(robot = robot, path = lift_coin_up, env= env, dt = dt, gripper = gripper, cam = cam, TCR = TCR, TGR = TGR)
    arrived = True
    send_action_client(arrived, lift_coin_up, goal, start_time, client)
    
    ################################# 3. FLIPPING THE BATTERIES START FROM HERE: ########################
    
    #### 3.1 MOVE TO ROBOT TO THE POSITION BEFORE FLIPPING THE BATTERY 1:

    flip_coin_data = []

    # Data Belong to battery 1:
    battery_1_position = [45.52, -68.18, 27.30, -47.27, -87.16, 315.94]
    battery_1_position = [math.radians(joint_home_degree) for joint_home_degree in battery_1_position]

    # Data Belong to battery 2:
    battery_2_position = [56.25, -73.65, 35.65, -53.94, -89.34, 324.05]
    battery_2_position = [math.radians(joint_home_degree) for joint_home_degree in battery_2_position]

    # Store data:
    flip_coin_data.append(battery_1_position)
    flip_coin_data.append(battery_2_position)

    # Rotation data:
    rot_coin = []
    rot_coin.append([-0.002,0,0.0043,14])
    rot_coin.append([0.002,0,0.0043,-14])

                # TEST ADDING BATTERY INFO AND DROPPING INFO
    is_battery_there = '12' # '1': only battery 1,'2': only battery 1, '12': both batteries 

    # Data belong to battery 1:
    battery_1_position = [45.52, -68.18, 27.30, -47.27, -87.16, 315.94]
    battery_1_position = [math.radians(joint_home_degree) for joint_home_degree in battery_1_position]

    # Data belong to battery 2:
    battery_2_position = [56.25, -73.65, 35.65, -53.94, -89.34, 324.05]
    battery_2_position = [math.radians(joint_home_degree) for joint_home_degree in battery_2_position]

    # Dropping data
    #-> For coin
    q_drop = [99.23, -75.10, 45.56, -66.99, -85.81, 351.35]
    q_drop_radian = [math.radians(joint_home_degree) for joint_home_degree in q_drop]
    path_drop = rtb.jtraj(lift_coin_up[-1], q_drop_radian, 30)

    #-> For batteries
    q_droppin_1 = [78.65, -96.40, 64.17, -58.12, -88.22, 252.57]
    q_droppin_1 = [math.radians(joint_home_degree) for joint_home_degree in q_droppin_1]
    q_droppin_2 = [70.67, -97.03, 61.32, -54.39, -88.19, 242.99]
    q_droppin_2 = [math.radians(joint_home_degree) for joint_home_degree in q_droppin_2]    
    
    # ROUTE FOR BATTERY 1:
    #-> From coin position to picking position
    pick1_way1 = [36.05, -63.35, 24.29, -48.96, -86.55, 217.81]
    pick1_way1 = [math.radians(joint_home_degree) for joint_home_degree in pick1_way1]
    pick1_way2 = [26.63, -49.31, 19.13, -27.04, -68.63, 203.57]
    pick1_way2 = [math.radians(joint_home_degree) for joint_home_degree in pick1_way2]
    
    pick1_path1 = rtb.mstraj(viapoints=np.array([path_drop.q[-1], pick1_way1, pick1_way2]),dt=0.01,tacc=0.05,qdmax = pi)

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
    path1_instruction ={'picking': pick1_path1, 'taking': pick1_path2, 'dropping': path_droppin_bat1}

    # ROUTE FOR BATTERY 2:
    #-> From current position to picking position
    pick2_way1 = [94.24, -79.7, 61.95, -74.67, -118.79, 84.73 + 360]
    pick2_way1 = [math.radians(joint_home_degree) for joint_home_degree in pick2_way1]

    pick2_way2 = [82.72, -69.40, 56.97, -72.54, -120.00, 82.96 + 360]
    pick2_way2 = [math.radians(joint_home_degree) for joint_home_degree in pick2_way2]

    q_pick2_start = path_droppin_bat1[-1] # default if is_batery_there == '12' or '1'
    if is_battery_there == '2': # if only battery 2 is put in, taking from the coin drop position
        q_pick2_start = path_drop.q[-1]

    pick2_path1 = rtb.mstraj(viapoints=np.array([q_pick2_start, pick2_way1, pick2_way2]),dt=0.01,tacc=0.05,qdmax = pi)

    #-> From picking position taking out
    drag_pose = SE3(-0.06, 0, 0.005) * robot.fkine(pick2_way2)
    q_sol_1 = solve_for_valid_ik(robot= robot, obj_pose = drag_pose, q_guess = pick2_way2)
    q_sol_1[-1] = pick2_way2[-1]

    down_pose = SE3(0.008, 0, -0.008) * robot.fkine(q_sol_1)
    q_sol_2 = solve_for_valid_ik(robot= robot, obj_pose = down_pose, q_guess = list(q_sol_1))
    q_sol_2[-1] = q_sol_1[-1]

    final_ee_pose = SE3(0.055, 0, 0.06)*robot.fkine(q_sol_2)
    q_sol_3 = solve_for_valid_ik(robot= robot, obj_pose = final_ee_pose, q_guess = list(q_sol_2))
    q_sol_3[-1] = q_sol_2[-1]

    pick2_path2 = rtb.mstraj(viapoints=np.array([pick2_way2, q_sol_1, q_sol_2, q_sol_3]),dt=0.01,tacc=0.05,qdmax = pi) 
    
    #->  To dropping position 2
    path2_droppin1 = rtb.jtraj(pick2_path2[-1], q_droppin_2, 30)
    path2_droppin2 = move_up_down(robot, path1_droppin1.q[-1], 'down', lift = 0.04)

    path_droppin_bat2 = combine_trajectories([path2_droppin1, path2_droppin2])

     #-> Combine route for battery 2
    path2_instruction ={'picking': pick2_path1, 'taking': pick2_path2, 'dropping': path_droppin_bat2}

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
        rot_coin.append([0.002,0,0.0043,-14])
        rot_coin.append([0.002,0,0.0043,-14])
        path_instruction.append(path1_instruction)
        path_instruction.append(path2_instruction)

    print(flip_coin_data)

    for i in range(len(flip_coin_data)):

        battery_position = flip_coin_data[i]

        path_to_battery = rtb.jtraj(lift_coin_up[-1], battery_position, 30)
    
        #Simulate and send command:
        move_simulation_robot(robot = robot, path= path_to_battery.q, env= env, dt = 0.05, gripper = gripper, cam = cam, pin = coin, TCR = TCR, TGR = TGR, TCP = TCC)
        arrived = True
        send_action_client(arrived, path_to_battery.q, goal, start_time, client, speed=1)

        ##1 -> down 2
        path_down_battery = move_up_down(robot, path_to_battery.q[-1], 'down', lift = 0.05)
        move_simulation_robot(robot = robot, path= path_down_battery.q, env= env, dt = 0.05, gripper = gripper, cam = cam, pin = coin, TCR = TCR, TGR = TGR, TCP = TCC)
        arrived = True
        send_action_client(arrived, path_down_battery.q, goal, start_time, client, speed=1)

        #### 3.2 START TO FLIP THE BATTERY 
        q_start = list(path_down_battery.q[-1])

        # Coin's desired position
        coin_end = collisionObj.Cylinder(radius = 0.01, length= 0.005, pose = coin.T, color = [0.0,0.5,0.0,0.5])
        rot_deg = rot_coin[i][3]
        coin_end.T = transl(rot_coin[i][0],0,0.0043) @ coin.T @ troty(rot_deg*pi/180) # apply lifting and rotation
        env.add(coin_end)

        q_coin_end = solve_for_valid_ik(robot=robot, obj_pose= SE3(coin_end.T), relative_pose = TCC.inv(), q_guess = q_start)
        q_coin_end[-1] = q_start[-1]

        # Create a trajectory to flip the battery:
        path_coin = rtb.jtraj(q_start, q_coin_end, 20)
        path_coin_up = rtb.jtraj(path_coin.q[-1], battery_position, 20)
        path_back_home = rtb.jtraj(path_coin_up.q[-1], lift_coin_up[-1],30)

        # combine_path = combine_3_trajectory(path_coin, path_coin_up, path_back_home)
        combine_path = combine_trajectories([path_coin, path_coin_up, path_back_home])

        # Simulate and send command:
        move_simulation_robot(robot = robot, path= combine_path, env= env, dt = 0.05, gripper = gripper, cam = cam, pin = coin, TCR = TCR, TGR = TGR, TCP = TCC)
        arrived = True
        send_action_client(arrived, combine_path, goal, start_time, client, speed=6)
    
    #-> Move to the position to drop the coin:
    q_drop = [99.23, -75.10, 45.56, -66.99, -85.81, 351.35]
    q_drop_radian = [math.radians(joint_home_degree) for joint_home_degree in q_drop]

    path_drop = rtb.jtraj(lift_coin_up[-1], q_drop_radian, 30)
    move_simulation_robot(robot = robot, path= path_drop.q, env= env, dt = 0.05, gripper = gripper, cam = cam, pin = coin, TCR = TCR, TGR = TGR, TCP = TCC)
    arrived = True
    send_action_client(arrived, path_drop.q, goal, start_time, client, speed=2)

    while not rospy.is_shutdown():
        pub.publish(moveGripperToPosition(400, 280))
        rospy.sleep(0.25)
        break

    ########################## 5. BEGIN TO PLACE THE BATTERY IN A CORRECT ORDER ###########################
    for instruction in path_instruction:
        # Move to picking battery
        move_simulation_robot(robot = robot, path= instruction['picking'], env= env, dt = 0.05, gripper = gripper, cam = cam, TCR = TCR, TGR = TGR)
        arrived = True
        send_action_client(arrived, instruction['picking'], goal, start_time, client, speed=1)

        while not rospy.is_shutdown():
            pub.publish(closeGripper(400))
            rospy.sleep(0.5)
            break

        # Move the battery out
        move_simulation_robot(robot = robot, path= instruction['taking'], env= env, dt = 0.05, gripper = gripper, cam = cam, TCR = TCR, TGR = TGR)
        arrived = True
        send_action_client(arrived, instruction['taking'], goal, start_time, client, speed=2)

        # Move to drop battery
        move_simulation_robot(robot = robot, path= instruction['dropping'], env= env, dt = 0.05, gripper = gripper, cam = cam, TCR = TCR, TGR = TGR)
        arrived = True
        send_action_client(arrived, path_droppin_bat1, goal, start_time, client, speed=3)

        while not rospy.is_shutdown():
            pub.publish(moveGripperToPosition(400, 280))
            rospy.sleep(0.25)
            break
        
    running_ = False
    
rospy.spin()