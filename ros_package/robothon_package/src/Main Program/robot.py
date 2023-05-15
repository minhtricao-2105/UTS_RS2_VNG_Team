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
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
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
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
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
# running_ = True

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

    # # The first one will be the yellow container:
    # if flag == 0:
    #     colour_array_yellow = [msg.data[i:i+2] for i in range(0, len(msg.data), 2)]

    #     rospy.loginfo('Yellow array: %s', colour_array_yellow)

    #     flag += 1

    # # The second one will be the orange container:
    # if flag == 1:
    #     colour_array_orange = [msg.data[i:i+2] for i in range(0, len(msg.data), 2)]
        
    #     rospy.loginfo('Orange array: %s', colour_array_orange)

  
    
rospy.init_node('robot_node')

# Subcribe Declaration:
subscriber_2 = rospy.Subscriber('Image_Data', Float32MultiArray, callback_2)

subscriber_3 = rospy.Subscriber('Color_data', Float32MultiArray, callback_3)

pub = rospy.Publisher('OnRobotRGOutput', OnRobotRGOutput, queue_size=1)
# subscriber_2.unsubscribe()

# subscriber_3.unsubscribe()

############## BELONG TO THE ROBOT OVER HERE ###########################

while len(location_array) == 0:
    rospy.loginfo("Waiting for vision")
    if(len(location_array) != 0):
        rospy.loginfo("Let go")
        rospy.loginfo('Location array: %s', location_array)
        # input("Done The Vision Part, Remove the Cable to Continue .... \n")
        running_ = True
        break

rospy.loginfo("Wait 5 seconds")
rospy.sleep(5)

while running_ == True:

    # Publisher Declaration:
    # pub = rospy.Publisher('OnRobotRGOutput', OnRobotRGOutput, queue_size=1)
    
    # Create a JointTrajectory message
    joint_traj = JointTrajectory()

    # Fill in the header
    joint_traj.header.frame_id = "base_link"

    # Set the joint names
    joint_traj.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

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

    # This allows for the time taken to send the message. If the network is fast this could be reduced
    buffer_seconds = 1

    # This is how many seconds the movement will take
    duration_seconds = 5

    #Call the client
    # client = actionlib.SimpleActionClient('eff_joint_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    client = actionlib.SimpleActionClient('scaled_pos_joint_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    
    start_time = time.perf_counter()
    # create environment 
    start_time = time.perf_counter()

    # First position of the robot:
    q_deg = [57.11, -106.32, 56.84, -44.7, -85.81, 328.40]
    
    # Enviroment
    env = swift.Swift()
    env.launch(realtime=True)
    dt = 0.05

    # Clone
    robot = rtb.models.UR3()
    q_start = [x*pi/180 for x in q_deg]
    robot.q = q_start
    env.add(robot)

    cam = collisionObj.Cuboid(scale=[0.03,0.06,0.015], pose = SE3(0,0,0), color = [0.5,0.5,0.1,1])

    TCR = SE3(0.085,0,0.09)
    cam_move(cam, robot, TCR*SE3.Ry(pi/2))

    camera_transform = cam.T

    # Define the point in Pixel frame
    pixel_x_1 = location_array[0][0]
    pixel_y_1 = location_array[0][1]

    global_position_1 = cam_to_global(pixel_x_1,pixel_y_1, camera_transform)

    # q_sample = [63.33, -105.04, 89.33, -75.14, -87.53, 335.72]
    # q_sample = [x*pi/180 for x in q_sample]
    # ee_orientation = SE3.Rt(robot.fkine(q_sample).R)

    # offset_z = 0.17
    # obj_pose=SE3(global_position_1[0], global_position_1[1], global_position_1[2]+ offset_z)*ee_orientation

    # q_guess = [1.3279389,  -1.41725404,  0.17017859, -0.62366733, -1.53202614, -0.20099896] 
    # q_goal = solve_for_valid_ik(robot=robot, obj_pose=obj_pose, q_guess = q_guess, elbow_up_request = True, shoulder_left_request=True)
    # q_goal[-1] += 2*pi
    # print(q_goal)

    # path = rtb.jtraj(robot.q, q_goal,50)

    # arrived = False
    # if not arrived:
    #     for q in path.q:
    #         robot.q = q
    #         cam_move(cam,robot,TCR)
    #         env.step(dt)

    path = move_to_pin(robot, q_start, global_position_1)
    arrived = False
    if not arrived:
        for q in path.q:
            robot.q = q
            cam_move(cam,robot,TCR)
            env.step(dt)

    total_path = []

    for q in path.q:
        total_path.append(q)

    arrived = True
    end_time = time.perf_counter()
    execution_time = end_time - start_time

    while arrived == True:
        # print(total_path)
        for i in range(len(total_path)):

            point = JointTrajectoryPoint()

            point.positions = total_path[i]

            # Calculate the time stamp based on the duration from the current time
        
            point.time_from_start = rospy.Duration.from_sec((i+1)*(duration_seconds/len(total_path))) + rospy.Duration.from_sec(execution_time + 1)

            goal.trajectory.points.append(point)

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
            rospy.loginfo("I am here")
            break
        break

    rospy.loginfo("LIFTING UP !!!!!!!!")

    robot.q = path.q[-1]
    path_lift = lift_up(robot, path.q[-1])
    
    arrived = False
    if not arrived:
        for q in path_lift.q:
            robot.q = q
            cam_move(cam,robot,TCR)
            env.step(dt)

    total_lift = []

    for q in path_lift.q:
        total_lift.append(q)
    
    print(len(total_lift))

    end_time_1 = time.perf_counter()
    
    execution_time_1 = end_time_1 - start_time

    arrived = True

    goal.trajectory.points.clear()

    while arrived == True:
        # print(total_path)
        for i in range(len(total_lift)):

            point = JointTrajectoryPoint()

            point.positions = total_lift[i]

            # Calculate the time stamp based on the duration from the current time
        
            point.time_from_start = rospy.Duration.from_sec((i+1)*(duration_seconds/len(total_lift))) + rospy.Duration.from_sec(execution_time_1 + 1)

            goal.trajectory.points.append(point)

            # Send the goal to the action server
        client.send_goal(goal)

            # Wait for the action to complete (or for the connection to be lost)
        client.wait_for_result()

            # Get the result of the action
        result = client.get_result()

            # Print the result
        print(result)

        arrived = False
    
    running_ = False

rospy.spin()