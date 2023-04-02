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
joint_traj.header.stamp = rospy.Time.now()
joint_traj.header.frame_id = "base_link"

# Set the joint names
# joint_traj.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

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
    
    # j0 = joint_positions[0]
    # j2 = joint_positions[2]

    # joint_positions[0] = j2
    # joint_positions[2] = j0



get_data()

print(joint_positions)

degrees_list = [degrees(r) for r in joint_positions]
print(degrees_list)

# Generate the trajectory
p2 = SE3(0.2, 0.3, 0.5) * SE3.Rx(pi/2) * SE3.Ry(pi/4)
robot = rtb.models.UR3()
rospy.sleep(1)
q1 = joint_positions
q2 = [-pi,-pi/2,0,-pi/3,-pi/4,0]
# q2 = robot.ikine_LM(T = p2,q0 = q1).q
traj = rtb.jtraj(q1, q2, 100)
# print(traj)

# Fill in the JointTrajectory message with the waypoints
for i in range(len(traj.q)):
    point = JointTrajectoryPoint()
    point.positions = traj.q[i]
    point.time_from_start = rospy.Duration.from_sec((i/100))
    joint_traj.points.append(point)

# # Publish the JointTrajectory message to the eff_joint_traj_controller/command topic
pub = rospy.Publisher('/scaled_pos_joint_traj_controller/command', JointTrajectory, queue_size=1)

rate = rospy.Rate(100)
print(degrees_list)
while not rospy.is_shutdown():
    rospy.sleep(0.05)
    pub.publish(joint_traj)
    diff = np.abs(np.array(joint_positions) - np.array(q2))  
    mod_diff = np.linalg.norm(diff) 
    print(mod_diff)
    if(mod_diff < 0.5):
        joint_traj.points = []
        break



rospy.is_shutdown()
