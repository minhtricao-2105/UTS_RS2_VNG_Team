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

#robot clone
robotClone = rtb.models.UR3()
robotClone.q = joint_home

# #destination
T = SE3(0.3, 0.2, 0.2) * SE3.Rx(-pi/2) * SE3.Ry(pi/4)
joint_end = robotClone.ikine_LM(T,q0 = robot.q).q
path = rtb.mtraj(tfunc = rtb.trapezoidal, q0 = joint_home, qf = joint_end, t = 100)

# move the robot to the home position
arm.go(joint_home, wait=True)

for q in path.q:
    arm.go(q,wait = True)    

moveit_commander.roscpp_shutdown()