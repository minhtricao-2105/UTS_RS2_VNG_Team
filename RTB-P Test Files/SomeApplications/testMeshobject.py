#!/usr/bin/env python3
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from math import degrees
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
import time

from ur3module import *
from pathgenerator import *

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


# create environment 
# env = swift.Swift()
# env.launch(realtime=True)

robot = rtb.models.UR3()
q_ini = [42,-28,61,-123,-87,0]
q_ini = [x*pi/180 for x in q_ini]

q_end = [-60,-28,61,-123,-87,0]
q_end = [x*pi/180 for x in q_end]
# create a goal object 
goal_obj = collisionObj.Sphere(radius = 0.02, pose = robot.fkine(q_end),color = (0.5,0.1,0.1,1))
# env.add(goal_obj)

robot.q = q_ini
# env.add(robot)

path_obj = "/home/minhtricao/robothon2023/RTB-P Test Files/SomeApplications/Robothon_Assembly_Box - Robothon_Assembly_Box.STEP-1 Robothon Box.STEP-1.STL"
box = collisionObj.Mesh(filename=path_obj,pose = SE3(0,0,0),scale=[0.001, 0.001, 0.001], color = (0.2,0.3,0.1,1))

# correct position
box_pose = SE3.Rx(pi/2)*SE3.Ry(pi/2)*SE3(box.T)
box_pose = SE3(0,-0.17,-0.12)*box_pose
box.T = box_pose.A

# env.add(box)

# create collision free path
total_path,success = gen_path(robot=robot,q_goal=q_end,obstacle_list=[box])

# if success: 
#     show_path(robot,path,env)
#     # move robot along path
#     for q in path:
#         robot.q = q
#         env.step(0.05)

flag = False

while flag == False:
    # print(total_path)
    print("Im fking done!")
    for i in range(len(total_path)):

        point = JointTrajectoryPoint()

        point.positions = total_path[i]

        # Calculate the time stamp based on the duration from the current time
       
        point.time_from_start = rospy.Duration.from_sec((i+1)*(duration_seconds/len(total_path)))

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



rospy.is_shutdown() 
# env.hold()