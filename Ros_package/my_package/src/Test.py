#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from math import pi
import time
import random

# Initialize the moveit_commander and rospy nodes
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('ur3e_set_joint_position', anonymous=True)

# Initialize the MoveIt planning scene, robot commander, and arm group
scene = moveit_commander.PlanningSceneInterface()
robot = moveit_commander.RobotCommander()
arm = moveit_commander.MoveGroupCommander("manipulator")

# Set the reference frame and end effector link
arm.set_pose_reference_frame("base_link")
arm.set_end_effector_link("ee_link")

# Set the maximum velocity scaling factor
arm.set_max_velocity_scaling_factor(0.5)

# Get the current joint values
current_joint_values = arm.get_current_joint_values()

#Set the home of the robot:
joint_home = [1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0]

# Define joint limits for your robot
# These are just examples, replace them with the limits for your specific robot
joint_limits = [[-2.0, 2.0], [-1.5, 1.5], [-1.5, 1.5], [-2.0, 2.0], [-2.0, 2.0], [-2.0, 2.0]]

while True:
    joint_goal = [random.uniform(joint_limits[i][0], joint_limits[i][1]) for i in range(len(joint_limits))]
    print(joint_goal)
    arm.go(joint_goal, wait=True)


moveit_commander.roscpp_shutdown()
