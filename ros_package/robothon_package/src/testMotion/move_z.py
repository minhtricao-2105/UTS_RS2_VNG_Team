#!/usr/bin/env python3
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import roboticstoolbox as rtb 
from spatialmath import SE3
from math import pi
from math import degrees
import numpy as np
import time
import swift
import actionlib
# Create a ros node:
rospy.init_node('execute_trajectory')

# Create a JointTrajectory message
joint_traj = JointTrajectory()

# Fill in the header
joint_traj.header.stamp = rospy.Time.now()
joint_traj.header.frame_id = "base_link"

# Set the joint names
joint_traj.joint_names = ['elbow_joint', 'shoulder_lift_joint', 'shoulder_pan_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

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
    hold = joint_positions[0]
    joint_positions[0] = joint_positions[2]
    joint_positions[2] = hold
    
get_data()

robot = rtb.models.UR3()
env = swift.Swift()
env.launch(realtime=True)

for link in robot.links:
    print(link.name)

q1  = joint_positions
robot.q = q1
q2 = [0,-pi/2, pi/3, 0, 0, 0]
traj = rtb.jtraj(q1, q2, 100)

env.add(robot)

for q in traj.q:
    robot.q = q
    env.step(0.05)

robot.q = q2
arrived = False
final_pose =  SE3.Tx(-0.2)* robot.fkine(q2)

path = []
while not arrived:
    v, arrived = rtb.p_servo(robot.fkine(robot.q), final_pose, 0.7)
    robot.qd = np.linalg.pinv(robot.jacobe(robot.q)) @ v
    env.step(0.05)
    path.append(robot.q)
    
total_path = []
for q in traj.q:
    total_path.append(q)

for q in path:
    total_path.append(q)


for i in range(len(total_path)):
    point = JointTrajectoryPoint()
    point.positions = total_path[i]
    point.time_from_start = rospy.Duration.from_sec(i/100.0)
    joint_traj.points.append(point)

# # Publish the JointTrajectory message to the eff_joint_traj_controller/command topic
pub = rospy.Publisher('/scaled_pos_joint_traj_controller/command', JointTrajectory, queue_size=0)
# [client, goal] = actionlib.SimpleActionClient('/scaled_pos_joint_traj_controller/follow_joint_trajectory')
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    joint_traj.header.stamp = rospy.Time.now()
    pub.publish(joint_traj)
    rate.sleep()
    diff = np.abs(np.array(joint_positions) - np.array(total_path[-1]))  
    mod_diff = np.linalg.norm(diff) 
    # print(mod_diff)
    if(mod_diff < 0.5):
        joint_traj.points = []
        break


rospy.is_shutdown()

