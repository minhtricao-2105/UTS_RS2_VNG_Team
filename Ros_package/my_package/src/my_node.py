#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from math import pi
import time

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


# Set the joint target positions (in radians)
# Set the joint target positions (in radians)
joint_goal1 = [1.57, -1.57, 1.57, 0, 0, 0]
joint_goal2 = [-1.57, -1.57, 1.57, 1.57, 0.6, 0.7]
joint_goal3 = [0, -1.57, 1.57, 0, 0, 0]
joint_goal4 = [1.57, 0, 1.57, 0, 0, 0]
joint_goal5 = [-1.57, -0.5, 1.57, 0, 0, 0]
joint_goals = [joint_goal1, joint_goal2, joint_goal3, joint_goal4, joint_goal5]

# move the robot to the home position
arm.go(joint_home, wait=True)

# loop through each joint goal and move the robot to it
while True:
    for goal in joint_goals:
        arm.go(goal)

# move the robot back to the home position
arm.go(joint_home, wait=True)

    


moveit_commander.roscpp_shutdown()