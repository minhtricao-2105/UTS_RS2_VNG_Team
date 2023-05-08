#!/usr/bin/env python3
# Import Library needed for the project:
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
import spatialgeometry.geom as collisionObj
from library import*

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

# DO THE TRAJECTORY PATH HERE:::
joint_positions = []

def get_joint_positions(msg):
    global joint_positions
    joint_positions = msg.position

# Subscribe to the joint states topic to get current joint positions
sub = rospy.Subscriber("/joint_states", JointState, get_joint_positions)


start_time = time.perf_counter()
# Start Time of the simulator robot: it is used to count the rospy.time for the realtime robot
r1 = rtb.models.UR3()
# start_time = time.perf_counter()



# Calling another node to get data for the current position
def get_data():
    rospy.sleep(0.5)
    global joint_positions
    joint_positions = list(joint_positions)
    swap = joint_positions[0]
    joint_positions[0] = joint_positions[2]
    joint_positions[2] = swap
    r1.q = joint_positions

get_data()

print(joint_positions)


degrees_list = [degrees(r) for r in joint_positions]

print(degrees_list)


# Generate the trajectory

q1  = r1.q
q2 = [0, -pi/2, pi/2, 0, pi/2, 0]
traj = rtb.jtraj(q1, q2, 100)


env = swift.Swift()
env.launch(realtime=True)
env.add(r1)

move_robot(r1,traj.q,env)

total_path = []

rectangle_path = solve_joint(q2, r1)

total_path = total_path + rectangle_path

q3 = [-pi/2, -pi/2, pi/2, 0, pi/2, 0]
             
traj3 = rtb.jtraj(total_path[-1],q3,100)

for q in traj3.q:
    total_path.append(q)

rectangle_path_2 = solve_joint(q3,r1)

total_path = total_path + rectangle_path_2

move_robot(r1,rectangle_path,env)

end_time = time.perf_counter()

execution_time = end_time - start_time

print(execution_time)


# Send the command to the robot:
while True:
    # print(total_path)
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