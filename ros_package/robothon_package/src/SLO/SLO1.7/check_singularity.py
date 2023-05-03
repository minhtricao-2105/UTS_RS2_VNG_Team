#!/usr/bin/env python3
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import roboticstoolbox as rtb 
from spatialmath import SE3
from math import pi
from math import degrees
import numpy as np
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
import swift
import time

import sys
from ur3module import *

# Create a ros node:
rospy.init_node('execute_trajectory')

# Create a JointTrajectory message
joint_traj = JointTrajectory()

# Fill in the header
joint_traj.header.frame_id = "base_link"

# Set the joint names
joint_traj.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

# Create a FollowJointTrajectoryGoal message:
goal = FollowJointTrajectoryGoal()

# Set the joint names:
goal.trajectory.joint_names = joint_traj.joint_names

# Set Sequence
goal.trajectory.header.seq = 1

# Set time stamp
goal.trajectory.header.stamp = rospy.Time.now()

# Set Tolerance
goal.goal_time_tolerance = rospy.Duration.from_sec(0.05)

# This allows for the time taken to send the message. If the network is fast this could be reduced
buffer_seconds = 1

# This is how many seconds the movement will take
duration_seconds = 5

# Call the client
# client = actionlib.SimpleActionClient('eff_joint_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
client = actionlib.SimpleActionClient('scaled_pos_joint_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

start_time = time.perf_counter()
# create robot and environment
# start_time = time.perf_counter()
robot = rtb.models.UR3()
q_deg = [31.24, -50.72, -5.51, -140.39, -38.79, 200.26]
robot.q = [x*pi/180 for x in q_deg]

env = swift.Swift()
env.launch(realtime=True)
env.add(robot)

# manipulability threshold
thresh_hold = 0.0097

# pose robot try to reach
final_pose_1 = robot.fkine(robot.q)*SE3.Tx(-0.25)

# RMRC implementation with singularity check
path_1 = []
arrived = False
dt = 0.05
while not arrived:    
    v, arrived = rtb.p_servo(robot.fkine(robot.q), final_pose_1, 0.5)
    robot.qd = np.linalg.pinv(robot.jacobe(robot.q)) @ v
    env.add(collisionObj.Sphere(radius=0.005, pose = robot.fkine(robot.q),color = (0.1,0.5,0.1,1)))
    env.step(dt)

    maniplty = robot.manipulability(robot.q,axes='trans')
    print(">",maniplty,'<\n')
    path_1.append(robot.q)
    if maniplty <= thresh_hold: 
        print("Close to singularity!")
        break

# if the singularity happen, try to finish the path with other method

flag = True

if not arrived:
    q_end = solve_for_valid_ik(robot,obj_pose=final_pose_1)
    path = rtb.jtraj(robot.q, q_end,50).q
    for q in path:
        robot.q = q
        env.add(collisionObj.Sphere(radius=0.005, pose = robot.fkine(robot.q),color = (0.5,0.1,0.1,1)))
        env.step(0.05)
        

total_path = []

for q in path_1:
    total_path.append(q)

for q in path:
    total_path.append(q)
arrived = True
end_time = time.perf_counter()
execution_time = end_time - start_time

while arrived == True:
    # print(total_path)
    print("Im fking done!")
    for i in range(len(total_path)):

        point = JointTrajectoryPoint()

        point.positions = total_path[i]

        # Calculate the time stamp based on the duration from the current time
       
        point.time_from_start = rospy.Duration.from_sec((i+1)*(duration_seconds/len(total_path))) + rospy.Duration.from_sec(execution_time + 1)

        goal.trajectory.points.append(point)

        # Send the goal to the action server
    client.send_goal(goal)

        # Wait for the action to complete (or for the connection to be lost)
    client.wait_for_result()

        # Get the result of the action
    result = client.get_result()

        # Print the result
    print(result)

    break
