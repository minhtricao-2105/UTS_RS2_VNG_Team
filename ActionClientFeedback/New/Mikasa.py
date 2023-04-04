
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
client = actionlib.SimpleActionClient('eff_joint_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

# DO THE TRAJECTORY PATH HERE:::
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

get_data()

print(joint_positions)

# Generate the trajectory
robot = rtb.models.UR3()
env = swift.Swift()
env.launch(realtime=True)

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

start_joint_send = JointTrajectoryPoint()
start_joint_send.positions = joint_positions
start_joint_send.time_from_start = rospy.Duration.from_sec(0)
goal.trajectory.points.append(start_joint_send)

for i in range(len(total_path)):
    point = JointTrajectoryPoint()
    point.positions = total_path[i]
    point.time_from_start = rospy.Duration.from_sec((i+1)*(duration_seconds/len(traj.q)))
    goal.trajectory.points.append(point)

# Send the goal to the action server
client.send_goal(goal)

# Wait for the action to complete (or for the connection to be lost)
client.wait_for_result()

# Get the result of the action
result = client.get_result()

# Print the result
print(result)


