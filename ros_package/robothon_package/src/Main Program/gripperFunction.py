#! /usr/bin/python3

##  @file
#   @brief This file contains the necessary functions for controlling the Onrobot RG2 gripper.
#   
#   @author Minh Tri Cao
#   @date last revised May 12, 2023

import sys
import copy
import rospy
import geometry_msgs.msg
from std_msgs.msg import String
from onrobot_rg_control.msg import OnRobotRGOutput
import math
import numpy as np

##  @brief Function to open the robot's gripper to maximum width
#
#   This function takes in a force in Newtons and returns a command to open the robot's gripper to maximum width, with the given force applied to the fingers.
#   @param force The force to be applied to the fingers in Newtons.
#   @return The command to open the gripper.
##
def openGripper(force):
    command_open = OnRobotRGOutput()
    command_open.rGFR = force
    command_open.rGWD = 1100
    command_open.rCTR = 16
    return command_open

##  @brief Function to close the robot's gripper
#
#   The closeGripper function takes in a force parameter in Newtons and creates a OnRobotRGOutput command with the appropriate values to close the gripper.
#   @param force The force to be applied by the gripper in Newtons.
#   @return An OnRobotRGOutput command to close the gripper with the specified force.
##
def closeGripper(force):
    command_close = OnRobotRGOutput()
    command_close.rGFR = force
    command_close.rGWD = 0
    command_close.rCTR = 16
    return command_close

##  @brief Moves the gripper to a specific position with a given force
#   
#   @param force the force in Newton.  
#   @param position the position to move the gripper to.
#   @return OnRobotRGOutput the command to move the gripper to the specified position with the given force.
##
def moveGripperToPosition(force, position):
    command_move = OnRobotRGOutput()
    command_move.rGFR = force
    command_move.rGWD = max(0, position)
    command_move.rCTR = 16
    return command_move
