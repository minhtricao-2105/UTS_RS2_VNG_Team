#!/usr/bin/env python3
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import roboticstoolbox as rtb 
from spatialmath import SE3
from math import pi
from math import degrees

import numpy as np

# Create a ros node:
rospy.init_node('execute_trajectory')

# Create a JointTrajectory message
joint_traj = JointTrajectory()

# Fill in the header
joint_traj.header.frame_id = "base_link"

# Set the joint names
joint_traj.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

# joint_traj.joint_names = ['elbow_joint', 'shoulder_lift_joint', 'shoulder_pan_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
#  Function to get current joint positions
joint_positions = []

def get_joint_positions(msg):
    global joint_positions
    joint_positions = msg.position

# Subscribe to the joint states topic to get current joint positions
sub = rospy.Subscriber("/joint_states", JointState, get_joint_positions)

def get_data():
    rospy.sleep(0.5)
    global joint_positions
    joint_positions = list(joint_positions)
    
    # j0 = joint_positions[0]
    # j2 = joint_positions[2]

    # joint_positions[0] = j2
    # joint_positions[2] = j0



get_data()

print(joint_positions)

degrees_list = [degrees(r) for r in joint_positions]
print(degrees_list)

# Generate the trajectory
robot = rtb.models.UR3()
rospy.sleep(1)
q1 = joint_positions
q2 = [-pi,-pi/2,0,-pi/3,-pi/4,0]
final_pose = SE3.Tz(-0.35)*robot.fkine(robot.q) 

# Create a trajectory from current position to q2:
traj = rtb.jtraj(q1, q2, 100)

# Create a trajectory from q2 to final pose:
arrived = False
path = []

while not arrived:
    v, arrived = rtb.p_servo(robot.fkine(robot.q), final_pose, 0.7)
    robot.qd = np.linalg.pinv(robot.jacobe(robot.q)) @ v
    path.append(robot.q)

# Compile two trajetory:
total_path = []

for q in traj.q:
    total_path.append(q)

for q in path:
    total_path.append(q)

# Fill in the JointTrajectory message with the waypoints
start = rospy.Time.now()
print(start.to_sec())
for i in range(len(traj.q)):
    point = JointTrajectoryPoint()
    point.positions = total_path.q[i]
    point.time_from_start =  rospy.Duration.from_sec(i/100)
    print(point.time_from_start)
    joint_traj.points.append(point)

print(joint_traj)

# # Publish the JointTrajectory message to the eff_joint_traj_controller/command topic
pub = rospy.Publisher('/eff_joint_traj_controller/command', JointTrajectory, queue_size=1)

# rate = rospy.Rate(100)
print(degrees_list)

while not rospy.is_shutdown():
    pub.publish(joint_traj)
    rospy.sleep(0.05)
    diff = np.abs(np.array(joint_positions) - np.array(q2))  
    mod_diff = np.linalg.norm(diff) 
    print(mod_diff)
    if(mod_diff < 0.5):
        joint_traj.points = []
        break



rospy.is_shutdown()
