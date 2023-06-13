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
from function import*

#Library Belongs to Robot Toolbox:
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import roboticstoolbox as rtb 
from spatialmath import SE3
from math import pi
from math import degrees
import actionlib
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
    rospy.loginfo("Moving Robot to Home!!")
    moveit_commander.roscpp_initialize(sys.argv)
    joint_home_degree = [57.11, -106.32, 56.84, -44.7, -85.81, 328.40]
    joint_home_radian = [math.radians(joint_home_degree) for joint_home_degree in joint_home_degree]
    arm = setUpRobot(0.5)
    arm.go(joint_home_radian)
    moveIt = True
    mess = Int32()
    mess.data = 1
    pub_1.publish(mess)
    rospy.loginfo("Done Moving to Home!!")
    break

# Subcribe Declaration:
subscriber_2 = rospy.Subscriber('Image_Data', Float32MultiArray, callback_2)

subscriber_3 = rospy.Subscriber('Color_data', Float32MultiArray, callback_3)

pub = rospy.Publisher('OnRobotRGOutput', OnRobotRGOutput, queue_size=1)

while len(location_array) == 0:
    rospy.loginfo("Waiting for vision")
    if(len(location_array) != 0):
        rospy.loginfo("Let go")
        rospy.loginfo('Location array: %s', location_array)
        running_ = True
        break

rospy.loginfo("Wait 5 seconds")
rospy.sleep(5)

hole = []
duplicate = False
position = hole_cordinate()

for i in location_array:
    # if i[2] == 0.0:
    #     for j in position: 
    #         if j[2] == 0.0  and j not in hole:
    #             hole.append(j)
    #         else:
    #             continue
    # elif i[2] == 1.0:
    #     for j in position: 
    #         if j[2] == 1.0  and j not in hole:
    #             hole.append(j)
    #         else:
    #             continue
    for j in position:
        if j[2] == i[2] and j not in hole:
            hole.append(j)
        else:
            continue

        
rospy.loginfo('Holes: %s', hole)
    
while running_ == True:

    goal = set_up_action_client()

    # This allows for the time taken to send the message. If the network is fast this could be reduced
    buffer_seconds = 1

    # This is how many seconds the movement will take
    duration_seconds = 5

    #Call the client
    # client = actionlib.SimpleActionClient('eff_joint_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    client = actionlib.SimpleActionClient('scaled_pos_joint_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    
    start_time = time.perf_counter()

    # Enviroment
    env = swift.Swift()
    env.launch(realtime=True)
    dt = 0.05

    # Clone
    robot = rtb.models.UR3()
  
    # Gripper
    gripper_path = "/home/minhtricao/robothon2023/RTB-P Test Files/SomeApplications/CAMGRIPPER.STL"
    gripper = collisionObj.Mesh(filename=gripper_path,pose = SE3(0,0,0),scale=[0.001, 0.001, 0.001],color = [0.5,0.1,0.1,1])
    TGR = SE3.Rx(pi)*SE3(0,-0.105,-0.175) #relative pose of gripper to ee

    # Camera
    cam = collisionObj.Cuboid(scale=[0.03,0.06,0.015], pose = SE3(0,0,0), color = [0.5,0.5,0.1,1])
    TCR = SE3(0.085,0,0.09)*SE3.Ry(pi/2) #relative pose of cam to ee
    
    # Set initial position
    q_deg = [57.11, -106.32, 56.84, -44.7, -85.81, 328.40]
    q_start = [x*pi/180 for x in q_deg]
    robot.q = q_start

    # Add to environment
    env.add(robot)
    env.add(cam)
    env.add(gripper)

    pin = []
    TCP = SE3(0.23,0,0)*SE3.Ry(pi/2) #relative pose pin to robot

    cam_move(cam, robot, TCR)
    cam_move(gripper, robot, TGR)
    camera_transform = cam.T
    
    for i in range(len(location_array)):
        global_pos = cam_to_global(location_array[i][0],location_array[i][1], camera_transform)
        pin.append(collisionObj.Cylinder(radius = 0.005, length= 0.06, pose = SE3(global_pos[0], global_pos[1], global_pos[2]-0.05), color = (0.2,0.2,0.5,1)))
        # cam_move(pin[i],robot, TCP)
        env.add(pin[i])
    
    for i in range(len(location_array)):
        while not rospy.is_shutdown():
            pub.publish(moveGripperToPosition(400, 280))
            rospy.sleep(3)
            break
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

        # Move to the pin
        path = move_to_pin(robot, q_start, global_position_1, turn = 0)
        arrived = False
        if not arrived:
            for q in path.q:
                robot.q = q
                cam_move(cam,robot,TCR)
                cam_move(gripper,robot,TGR)
                env.step(dt)

        total_path = []

        for q in path.q:
            total_path.append(q)

        arrived = True
        end_time = time.perf_counter()
        execution_time = end_time - start_time

        while arrived == True:

            add_trajectory(total_path, goal, execution_time)

            # Send the goal to the action server
            client.send_goal(goal)

                # Wait for the action to complete (or for the connection to be lost)
            client.wait_for_result()

                # Get the result of the action
            result = client.get_result()

                # Print the result
            print(result)

            arrived = False

            while not rospy.is_shutdown():
                pub.publish(moveGripperToPosition(400, 50))
                rospy.sleep(3)
                break
            break

        # Move up
        rospy.loginfo("LIFTING UP !!!!!!!!")
        
        robot.q = path.q[-1]
        path_lift = move_up_down(robot, path.q[-1], lift=0.06)
        
        arrived = False
        if not arrived:
            for q in path_lift.q:
                robot.q = q
                cam_move(cam,robot,TCR)
                cam_move(gripper,robot,TGR)
                cam_move(pin[i],robot, TCP)
                env.step(dt)

        total_lift = []
        

        for q in path_lift.q:
            total_lift.append(q)

        
        end_time_1 = time.perf_counter()
        
        execution_time_1 = end_time_1 - start_time

        arrived = True

        goal.trajectory.points.clear()

        while arrived == True:

            add_trajectory(total_lift, goal, execution_time_1)

            # Send the goal to the action server
            client.send_goal(goal)

            # Wait for the action to complete (or for the connection to be lost)
            client.wait_for_result()

            # Get the result of the action
            result = client.get_result()

            # Print the result
            print(result)

            arrived = False
        

        # Move to Goal
        rospy.loginfo("MOVING THE ANOTHER POSITION")
        
        goal.trajectory.points.clear()

        pixel_x_2 = hole[i][0]
        pixel_y_2 = hole[i][1]

        global_position_2 = cam_to_global(pixel_x_2,pixel_y_2, camera_transform)

        rot = rotate_ee(path_lift.q[-1], turn = 0)
        q_start = rot.q[-1]

        robot.q = q_start
        path_move = move_to_pin(robot, q_start, global_position_2,0.225)    
        
        arrived = False
        
        if not arrived:
            for q in path_move.q:
                robot.q = q
                cam_move(cam,robot,TCR)
                cam_move(gripper,robot,TGR)
                cam_move(pin[i],robot, TCP)
                env.step(dt)
        
        move = []

        for q in rot.q:
            move.append(q)

        for q in path_move.q:
            move.append(q)
        
        q_start = path_move.q[-1]
        
        # Move down
        path_down = move_up_down(robot, q_start,'down',lift = 0.035)

        if not arrived:
            for q in path_down.q:
                robot.q = q
                cam_move(cam,robot,TCR)
                cam_move(gripper,robot,TGR)
                cam_move(pin[i],robot, TCP)
                env.step(dt)
        
        for q in path_down.q:
            move.append(q)

        end_time_2 = time.perf_counter()
        
        execution_time_2 = end_time_2 - start_time

        arrived = True

        while arrived == True:

            add_trajectory(move, goal,execution_time_2)

                # Send the goal to the action server
            client.send_goal(goal)

                # Wait for the action to complete (or for the connection to be lost)
            client.wait_for_result()

                # Get the result of the action
            result = client.get_result()

                # Print the result
            print(result)
            
            while not rospy.is_shutdown():
                pub.publish(moveGripperToPosition(400, 170))
                rospy.sleep(3)
                break
            
            break
        
        arm.go(joint_home_radian)

    
    running_ = False

rospy.spin()