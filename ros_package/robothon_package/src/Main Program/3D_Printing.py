# from function import rtb, collisionObj,SE3, pi, swift, np, cam_move, move_simulation_robot
from spatialmath.base import *
from ur3module import *
import time
from function import *
import rospy
#Library Belongs to Gripper
from onrobot_rg_control.msg import OnRobotRGOutput
from gripperFunction import *

rospy.init_node('test_node')

# Clone
robot = rtb.models.UR3() 
# q_deg = [48.82, -76.87, 53.63, -66.76, -90.25, 316.64] # First position of the robot:
# q_deg = [48.79, -75.39, 48.19, -59.86, -89.85, 316.2]
# q_deg = [47.21, -80.32, 59.11, -69.57, -89.16, 316.65]
# q_deg = [49.09, -75.28, 51.89, -65.13, -90.88, 317.38]
# q_start = [x*pi/180 for x in q_deg]
#robot.q = q_start

# Camera
cam = collisionObj.Cuboid(scale=[0.03,0.06,0.015], pose = SE3(0,0,0), color = [0.5,0.5,0.1,1])
TCR = SE3(0.085,0,0.09)*SE3.Ry(pi/2) #cam pose in ee frame
# cam_move(cam, robot, TCR)

# Gripper
TGR = SE3.Rx(pi)*SE3(0,-0.105,-0.175) #gripper pose in ee frame
gripper_path = "/home/minhtricao/robothon2023/RTB-P Test Files/SomeApplications/CAMGRIPPER.STL"
# gripper_path = "/home/quangngo/Robotics Studio 2/GroupGit/robothon2023/RTB-P Test Files/SomeApplications/CAMGRIPPER.STL"
# gripper_path = "D:/UTS_RS2_VNG_Team/RTB-P Test Files/SomeApplications/CAMGRIPPER.STL"
gripper = collisionObj.Mesh(filename=gripper_path,pose = SE3(0,0,0),scale=[0.001, 0.001, 0.001],color = [0.5,0.1,0.1,1])
# cam_move(gripper,robot,TGR)

# Coin
coin = collisionObj.Cylinder(radius = 0.01, length= 0.005, pose = SE3(0, 0, 0), color = [0.3,0.3,0.1,1])
TCP = SE3(0.23,0,0) * SE3.Rx(pi/2) #coin pose in ee frame
# cam_move(coin, robot, TCP)

# Enviroment
env = swift.Swift()
env.launch(realtime=True)
dt = 0.05

## INITIAL SET UP -------------------------------------------------------------------------------------------------------- #
goal = set_up_action_client()

 # This allows for the time taken to send the message. If the network is fast this could be reduced
buffer_seconds = 1

# This is how many seconds the movement will take
duration_seconds = 5

# Call the client
# client = actionlib.SimpleActionClient('eff_joint_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
client = actionlib.SimpleActionClient('scaled_pos_joint_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
pub = rospy.Publisher('OnRobotRGOutput', OnRobotRGOutput, queue_size=1)
start_time = time.perf_counter()

# HOME
## home -> 1
joint_home_degree = [57.11, -106.32, 56.84, -44.7, -85.81, 328.40]
joint_home_radian = [math.radians(joint_home_degree) for joint_home_degree in joint_home_degree]
arm = setUpRobot(0.5)
arm.go(joint_home_radian)
robot.q = joint_home_radian

q1 = [45.52, -68.18, 27.30, -47.27, -87.16, 315.94]
q1_radian = [math.radians(joint_home_degree) for joint_home_degree in q1]

env.add(robot)
env.add(cam)
env.add(gripper)

