#!/usr/bin/env python

import rospy
import moveit_commander
from moveit_commander.conversions import pose_to_list
from math import pi

# Initialize the ROS node
rospy.init_node('cobot_controller', anonymous=True)

# Initialize the MoveIt! commander
moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "manipulator"
move_group = moveit_commander.MoveGroupCommander(group_name)

# Set the reference frame and end effector link
move_group.set_pose_reference_frame("base_link")
move_group.set_end_effector_link("ee_link")

# Set the joint tolerance and maximum velocity
move_group.set_goal_joint_tolerance(0.001)
move_group.set_max_velocity_scaling_factor(0.1)

# Set the target pose
pose_goal = geometry_msgs.msg.Pose()
pose_goal.position.x = 0.4
pose_goal.position.y = 0.0
pose_goal.position.z = 0.4
pose_goal.orientation.x = 0.0
pose_goal.orientation.y = 1.0
pose_goal.orientation.z = 0.0
pose_goal.orientation.w = 0.0
move_group.set_pose_target(pose_goal)

# Plan and execute the motion
plan = move_group.go(wait=True)
move_group.stop()
move_group.clear_pose_targets()

# Shut down MoveIt! commander and ROS node
moveit_commander.roscpp_shutdown()
moveit_commander.os._exit(0)
