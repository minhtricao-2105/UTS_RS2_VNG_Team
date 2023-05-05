#!/usr/bin/env python3

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
arm = setUpRobot(0.5)

arm.go(joint_home_radian)

for goal in moveByJointTarget():
    arm.go(goal)
    rospy.sleep(1)

while not rospy.is_shutdown():
    pub.publish(moveGripperToPosition(400, 100))
    rospy.sleep(3)
    break

for goal in moveByJointTarget1():
    arm.go(goal)
    rospy.sleep(1)

while not rospy.is_shutdown():
    pub.publish(openGripper(400))
    rospy.sleep(3)
    break

arm.go(joint_home_radian)

# Task 2:

for goal in moveByJointTarget2():
    arm.go(goal)
    rospy.sleep(1)

while not rospy.is_shutdown():
    pub.publish(moveGripperToPosition(400, 50))
    rospy.sleep(3)
    break

for goal in moveByJointTarget3():
    arm.go(goal)
    rospy.sleep(1)

while not rospy.is_shutdown():
    pub.publish(openGripper(400))
    rospy.sleep(3)
    break

arm.go(joint_home_radian)

#Task 3
for goal in moveByJointTarget4():
    arm.go(goal)
    rospy.sleep(1)

while not rospy.is_shutdown():
    pub.publish(moveGripperToPosition(400, 200))
    rospy.sleep(3)
    break

for goal in moveByJointTarget5():
    arm.go(goal)
    rospy.sleep(1)

while not rospy.is_shutdown():
    pub.publish(openGripper(400))
    rospy.sleep(3)
    break

arm.go(joint_home_radian)
moveit_commander.roscpp_shutdown()


