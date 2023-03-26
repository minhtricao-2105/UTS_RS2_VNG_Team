#!/usr/bin/env python3
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from math import pi
import time
from onrobot_rg_control.msg import OnRobotRGOutput

# Initialize the moveit_commander and rospy nodes
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('ur3e_set_joint_position', anonymous=True)

pub = rospy.Publisher('OnRobotRGOutput', OnRobotRGOutput, queue_size=1)

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
current_pose_values = arm.get_current_pose().pose

print(current_pose_values)
#Set the home of the robot:
joint_home = [1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0]


# Set the joint target positions (in radians)
# Set the joint target positions (in radians)
joint_goal1 = [1.57, -1.57, 1.57, 0, 0, 0]
joint_goal2 = [-1.57, -1.57, 1.57, 1.57, 0.6, 0.7]
joint_goal3 = [0, -1.57, 1.57, 0, 0, 0]
joint_goal4 = [1.57, -1.57, 1.57, 0, 0, 0]
joint_goal5 = [-1.57, -1.57, 1.57, 0, 0, 0]
joint_job = [46.87*pi/180, -78.77*pi/180, 89.39*pi/180, -101.42*pi/180, -89.38*pi/180, 319.51*pi/180]

joint_goals = [joint_job,joint_home, joint_goal1, joint_goal2, joint_goal3, joint_goal4, joint_goal5]

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


# move the robot to the home position
arm.go(joint_home)

done = True
# loop through each joint goal and move the robot to it
while True:
    for goal in joint_goals:
        arm.go(goal)
        rospy.sleep(1)
        if done: 
            pub.publish(command_close)
            done = False

# move the robot back to the home position
arm.go(joint_home, wait=True)

    


moveit_commander.roscpp_shutdown()
