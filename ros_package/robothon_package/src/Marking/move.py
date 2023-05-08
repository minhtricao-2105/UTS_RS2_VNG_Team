#!/usr/bin/env python3

#First, include necessary library use for the moveit
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from math import pi
import time
from onrobot_rg_control.msg import OnRobotRGOutput

#Import Library:

from moveRobot import *
from gripperFunction import *

# Initialize the moveit_commander and rospy nodes
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('ur3e_set_joint_position', anonymous=True)
pub = rospy.Publisher('OnRobotRGOutput', OnRobotRGOutput, queue_size=1)

#Setup Robot:
#Set the home of the robot:
joint_home_degree = [-49.86, -115.97, 3.56, -163.74, 90.03, 40.61]
joint_home_radian = [math.radians(joint_home_degree) for joint_home_degree in joint_home_degree]

#Set up Robot:
arm = setUpRobot(0.5)

# At the beginning, let's move the robot to the home position:
arm.go(joint_home_radian)