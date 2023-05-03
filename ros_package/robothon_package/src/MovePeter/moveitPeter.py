#!/usr/bin/env python3
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from math import pi
from spatialmath import SE3
import roboticstoolbox as rtb 
import time
import trajectory_msgs.msg
from geometry_msgs.msg import Pose

# Initialize the moveit_commander and rospy nodes
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('ur3e_set_joint_position', anonymous=True)

# Initialize the MoveIt planning scene, robot commander, and arm group
scene = moveit_commander.PlanningSceneInterface()
robot = moveit_commander.RobotCommander()
arm = moveit_commander.MoveGroupCommander("manipulator")

# Set the reference frame and end effector link
# Instantiate a RobotCommander object
robot = moveit_commander.RobotCommander()

# Instantiate a PlanningSceneInterface object
scene = moveit_commander.PlanningSceneInterface()

# Instantiate a MoveGroupCommander object for the end effector
group_name = "end_effector_group"
move_group = moveit_commander.MoveGroupCommander(group_name)

# Set the target pose
target_pose = Pose()
target_pose.position.x = 0.5 # set x coordinate of end effector
target_pose.position.y = 0.0 # set y coordinate of end effector
target_pose.position.z = 0.2 # set z coordinate of end effector
target_pose.orientation.x = 0.0
target_pose.orientation.y = 0.0
target_pose.orientation.z = 0.0
target_pose.orientation.w = 1.0

# Set the target pose as the goal
move_group.set_pose_target(target_pose)

# Plan a trajectory to move the end effector to the target pose
plan = move_group.go(wait=True)

# shut down the moveit_commander node
moveit_commander.roscpp_shutdown()
