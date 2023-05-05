#!/usr/bin/env python3
import sys
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

# Set joint home
joint_home = [1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0]

# Set the initial and final Cartesian poses
p1 = SE3(0.5, 0, 0.5) 
p2 = SE3(0.5, 0.2, 0.5) * SE3.Rx(-pi/2) * SE3.Ry(pi/4)

# Use the Robotics Toolbox to generate a joint trajectory between the initial and final Cartesian poses
robotClone = rtb.models.UR3()
q1 = robotClone.ikine_LM(p1)
q2 = robotClone.ikine_LM(p2)
# path = rtb.mtraj(tfunc = rtb.trapezoidal, q0 = q1.q, qf = q2.q, t= 50)
path = rtb.jtraj(q0=q1.q,qf = q2.q, t = 50)
# path = rtb.ctraj(p1,p2,50)
path.plot(True)
print(path.q)
# Plan and execute the joint trajectory using MoveIt
for q in path.q:
    arm.go(q)
    print(q)

# Move the robot back to the home position
arm.go(joint_home, wait=True)

moveit_commander.roscpp_shutdown()
