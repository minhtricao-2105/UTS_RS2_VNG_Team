#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from math import pi
import time
import random
from onrobot_rg_control.msg import OnRobotRGOutput

# Create a ros node
rospy.init_node('OnRobotRGSimpleController', anonymous=True)

pub = rospy.Publisher('OnRobotRGOutput', OnRobotRGOutput, queue_size=1)

#for UR3E
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
arm.set_max_velocity_scaling_factor(0.5)

# Get the current joint values
current_joint_values = arm.get_current_joint_values()

#Set the home of the robot:
joint_home = [1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0]


# Set the joint target positions (in radians)
# Set the joint target positions (in radians)
joint_goal1 = [1.7708, -1.5708, 1.7708, -1.8708, -1.5708, 0]

joint_goals = [joint_goal1] 


# Create a command to open the gripper
command_open = OnRobotRGOutput()
command_open.rGFR= 400
command_open.rGWD = 1100
command_open.rCTR = 16

# Create a command to close the gripper
command_close = OnRobotRGOutput()
command_close.rGFR = 400
command_close.rGWD = 0
command_close.rCTR = 16
# Publish the open command to the gripper topic
pub.publish(command_open)

# Wait for the gripper to open
rospy.sleep(1)

# Publish the close command to the gripper topic
pub.publish(command_close)

# Wait for the gripper to close
rospy.sleep(1)


# Keep publishing the close command to the gripper topic to keep the gripper closed
while not rospy.is_shutdown():
    pub.publish(command_open)
    break

arm.go(joint_home, wait = True)

while not rospy.is_shutdown():
    
    pub.publish(command_close)
    rospy.sleep(0.1)

    break

rospy.is_shutdown()