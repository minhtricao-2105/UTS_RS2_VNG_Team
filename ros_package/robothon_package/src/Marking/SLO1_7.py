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
joint_home_degree = [0,-90,0,-90,0,0]
joint_home_radian = [math.radians(joint_home_degree) for joint_home_degree in joint_home_degree]

#Set up Robot:
arm = setUpRobot(0.5)

# At the beginning, let's move the robot to the home position:
arm.go(joint_home_radian)

# Move Robot to the first position using moveit:
for goal in moveByJointTarget():
    arm.go(goal)
    rospy.sleep(1)

# Close The Gripper:
while not rospy.is_shutdown():
    pub.publish(moveGripperToPosition(280, 80))
    rospy.sleep(3)
    break

# Move the Robot to the second Position using Moveit:
for goal in moveByJointTarget1():
    arm.go(goal)
    rospy.sleep(1)

# Then Open the Gripper:
while not rospy.is_shutdown():
    pub.publish(openGripper(400))
    rospy.sleep(3)
    break

#Move Robot to the third position using Moveit:
for goal in moveByJointTarget2():
    arm.go(goal)
    rospy.sleep(1)


# After, completing first task, move the robot to home position:
arm.go(joint_home_radian)


# # Task 2 of will begin in here:

# #Move Robot to the first position of task 2 using Moveit:
# for goal in moveByJointTarget3():
#     arm.go(goal)
#     rospy.sleep(1)

# # Then Close the gripper:
# while not rospy.is_shutdown():
#     pub.publish(moveGripperToPosition(400, 50))
#     rospy.sleep(3)
#     break

# #Move Robot to the second position of task 2 using Moveit:
# for goal in moveByJointTarget4():
#     arm.go(goal)
#     rospy.sleep(1)

# #Move Robot to the third position of task 2 using Moveit:
# for goal in moveByJointTarget5():
#     arm.go(goal)
#     rospy.sleep(1)

# # Then Open the Gripper:
# while not rospy.is_shutdown():
#     pub.publish(openGripper(400))
#     rospy.sleep(3)
#     break

# # After Finishing 2 tasks move the robot back to home position:
# arm.go(joint_home_radian)
