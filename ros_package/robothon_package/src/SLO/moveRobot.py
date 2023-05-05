#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from math import pi
import time
import math


def moveByJointTarget():
    #Goal 1:
    joint_1_dgree = [-65.33, -91.79, -56.05, -104.97, 96.33, 18.41]
    joint_goal1 = [math.radians(joint_1_dgree) for joint_1_dgree in joint_1_dgree]


    #Goal 2:
    joint_2_dgree = [-70.38, -96.02, -63.19, -109.98, 91.19, 14.77]
    joint_goal2 = [math.radians(joint_2_dgree) for joint_2_dgree in joint_2_dgree]

    joint_goals = [joint_goal1, joint_goal2]

    return joint_goals

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

def moveByJointTarget1():

    #Goal 3:
    joint_3_dgree = [-70.43, -107.65, -27.48, -134.06, 91.12, 14.73]
    joint_goal3 = [math.radians(joint_3_dgree) for joint_3_dgree in joint_3_dgree]

     #Goal 4:
    joint_4_dgree = [-55.68, -103.23, -47.45, -118.17, 90.59, 27.61]
    joint_goal4 = [math.radians(joint_4_dgree) for joint_4_dgree in joint_4_dgree]

        #Goal 3:
    joint_5_dgree = [-55.68, -100.32, -57.9, -110.63, 90.61, 27.62]
    joint_goal5 = [math.radians(joint_5_dgree) for joint_5_dgree in joint_5_dgree]

    joint_goals = [joint_goal3, joint_goal4, joint_goal5]

    return joint_goals

def moveByJointTarget2():

    #Goal 3:
    joint_3_dgree = [-56.28, -106.72, -36.52, -125.59, 90.69, 27.00]
    joint_goal3 = [math.radians(joint_3_dgree) for joint_3_dgree in joint_3_dgree]


    joint_goals = [joint_goal3]

    return joint_goals

def moveByJointTarget3():

    #Goal 3:
    joint_3_dgree = [-101.64, -114.06, -46.51, -125.82, 148.34, 230.82]
    joint_goal3 = [math.radians(joint_3_dgree) for joint_3_dgree in joint_3_dgree]

     #Goal 4:
    joint_4_dgree = [-316.28, -106.55, -86.68, -96.65, 89.06, 187.19]
    joint_goal4 = [math.radians(joint_4_dgree) for joint_4_dgree in joint_4_dgree]

    # joint_5_dgree = [-107.43, -98.81, 18.88, -127.89, 113.19, 236.04]
    # joint_goal5 = [math.radians(joint_5_dgree) for joint_5_dgree in joint_5_dgree]


    joint_goals = [joint_goal3, joint_goal4]

    return joint_goals

def moveByJointTarget4():

    #Goal 3:
    joint_5_dgree = [-286.25, -100.62, 59.68, -255.19, 57.69, 253.30]
    joint_goal5 = [math.radians(joint_5_dgree) for joint_5_dgree in joint_5_dgree]

     #Goal 4:
    joint_4_dgree = [-286.25, -70.42, 61.45, -261.11, 98.73, 254.42]
    joint_goal4 = [math.radians(joint_4_dgree) for joint_4_dgree in joint_4_dgree]

    joint_goals = [joint_goal5, joint_goal4]

    return joint_goals

def moveByJointTarget5():

    #Goal 3:
    joint_5_dgree = [-319.01, -58.29, 11.48, -275.15, 84.65, 273.13]
    joint_goal5 = [math.radians(joint_5_dgree) for joint_5_dgree in joint_5_dgree]

     #Goal 4:
    joint_4_dgree = [34.79, -93.34, 105.99, -101.77, -94.11, 0.29]
    joint_goal4 = [math.radians(joint_4_dgree) for joint_4_dgree in joint_4_dgree]

    joint_goals = [joint_goal5, joint_goal4]

    return joint_goals