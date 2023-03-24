#! /usr/bin/python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from onrobot_rg_control.msg import OnRobotRGOutput
import math
import numpy as np

# Open the gripper to maximum 
# @para force Newton
def openGripper(force):
    command_open = OnRobotRGOutput()
    command_open.rGFR = force
    command_open.rGWD = 1100
    command_open.rCTR = 16

def closeGripper(force):
    command_close = OnRobotRGOutput()
    command_close.rGFR = force
    command_close.rGWD = 0
    command_close.rCTR = 16

def moveGripperToPosition(force, position):
    command_move = OnRobotRGOutput()
    command_move.rGFR = force
    command_move.rGWD = max(0, position)
    command_move.rCTR = 16
    
