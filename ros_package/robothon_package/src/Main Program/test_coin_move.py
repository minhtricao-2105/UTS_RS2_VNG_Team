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
env.add(coin)

path1 = rtb.jtraj(joint_home_radian, q1_radian, 30)
move_simulation_robot(robot = robot, path= path1.q, env= env, dt = 0.05, gripper = gripper, cam = cam, pin = coin, TCR = TCR, TGR = TGR, TCP = TCP)

arrived = True
send_action_client(arrived, path1.q, goal, start_time, client, speed=1)

##1 -> down 2
path_down1 = move_up_down(robot, path1.q[-1], 'down', lift = 0.05)
move_simulation_robot(robot = robot, path= path_down1.q, env= env, dt = 0.05, gripper = gripper, cam = cam, pin = coin, TCR = TCR, TGR = TGR, TCP = TCP)

arrived = True
send_action_client(arrived, path_down1.q, goal, start_time, client, speed=1)

##3 -> flick coin
# q_start = list(robot.q)
# q_start[-1] += (3*pi/4)
q_start = list(path_down1.q[-1])

# Coin's desired position
coin_end = collisionObj.Cylinder(radius = 0.01, length= 0.005, pose = coin.T, color = [0.0,0.5,0.0,0.5])
rot_deg = 14
coin_end.T = transl(-0.002,0,0.0043) @ coin.T @ troty(rot_deg*pi/180) # apply lifting and rotation
env.add(coin_end)

# input("Press anything\n")
q_coin_end = solve_for_valid_ik(robot=robot, obj_pose= SE3(coin_end.T), relative_pose = TCP.inv(), q_guess = q_start)
q_coin_end[-1] = q_start[-1]
# print(q_coin_end)
path_coin = rtb.jtraj(q_start, q_coin_end, 20)
path_coin_up = rtb.jtraj(path_coin.q[-1], q1_radian, 20)

path_lift_coin = []

for q in path_coin.q:
    path_lift_coin.append(q)
for q in path_coin_up.q:
    path_lift_coin.append(q)

move_simulation_robot(robot = robot, path= path_lift_coin, env= env, dt = 0.05, gripper = gripper, cam = cam, pin = coin, TCR = TCR, TGR = TGR, TCP = TCP)

arrived = True

send_action_client(arrived, path_lift_coin, goal, start_time, client, speed=4)


## Battery 2
### Move from 1 to 2 and move down
q2 = [56.25, -78.38, 50.01, -63.57, -89.34, 327.08]
q2_radian = [math.radians(joint_home_degree) for joint_home_degree in q2]
path_1to2 = rtb.jtraj(path_lift_coin[-1], q2_radian, 20)

path_down2 = move_up_down(robot, path_1to2.q[-1], 'down', lift = 0.032)

path_battery2 = []

for q in path_1to2.q:
    path_battery2.append(q)
for q in path_down2.q:
    path_battery2.append(q)

move_simulation_robot(robot = robot, path= path_battery2, env= env, dt = 0.05, gripper = gripper, cam = cam, pin = coin, TCR = TCR, TGR = TGR, TCP = TCP)
arrived = True
send_action_client(arrived, path_battery2, goal, start_time, client, speed=2)

#### Flick coin 2
rot_deg = -14
coin_end.T = transl(0.002,0,0.0043) @ coin.T @ troty(rot_deg*pi/180) # apply lifting and rotation

q_coin2_end = solve_for_valid_ik(robot=robot, obj_pose= SE3(coin_end.T), relative_pose = TCP.inv(), q_guess = list(path_battery2[-1]))
q_coin2_end[-1] = path_battery2[-1][-1]

path_coin2 = rtb.jtraj(path_battery2[-1], q_coin2_end, 20)
path_coin_up2 = rtb.jtraj(path_coin2.q[-1], q2_radian, 20)

path_lift_coin2 = []

for q in path_coin2.q:
    path_lift_coin2.append(q)
for q in path_coin_up2.q:
    path_lift_coin2.append(q)

move_simulation_robot(robot = robot, path= path_lift_coin2, env= env, dt = 0.05, gripper = gripper, cam = cam, pin = coin, TCR = TCR, TGR = TGR, TCP = TCP)
arrived = True
send_action_client(arrived, path_lift_coin2, goal, start_time, client, speed=4)



##DONE BATTERY 2

#################################

#-> Move to the position to drop the coin:
q_drop = [99.23, -75.10, 45.56, -66.99, -85.81, 351.35]
q_drop_radian = [math.radians(joint_home_degree) for joint_home_degree in q_drop]

path_drop = rtb.jtraj(path_lift_coin2[-1], q_drop_radian, 30)
move_simulation_robot(robot = robot, path= path_drop.q, env= env, dt = 0.05, gripper = gripper, cam = cam, pin = coin, TCR = TCR, TGR = TGR, TCP = TCP)
arrived = True
send_action_client(arrived, path_drop.q, goal, start_time, client, speed=2)

while not rospy.is_shutdown():
    pub.publish(moveGripperToPosition(400, 280))
    rospy.sleep(0.25)
    break
#-> Done dropping the coin:

# #-> Move robot to home after dropping the coin:
# path_home_drop = rtb.jtraj(path_drop.q[-1], joint_home_radian, 30)
# move_simulation_robot(robot = robot, path= path_home_drop.q, env= env, dt = 0.05, gripper = gripper, cam = cam, pin = coin, TCR = TCR, TGR = TGR, TCP = TCP)
# arrived = True
# send_action_client(arrived, path_home_drop.q, goal, start_time, client, speed=2)

# -> Done moving home:

#-> Move to pick 1:
pick1_way1 = [36.05, -63.35, 24.29, -48.96, -86.55, 217.81]
pick1_way1 = [math.radians(joint_home_degree) for joint_home_degree in pick1_way1]

pick1_way2 = [26.63, -49.31, 19.13, -27.04, -68.63, 203.57]
pick1_way2 = [math.radians(joint_home_degree) for joint_home_degree in pick1_way2]

pick1_path1 = rtb.mstraj(viapoints=np.array([path_drop.q[-1], pick1_way1, pick1_way2]),dt=0.01,tacc=0.05,qdmax = pi)

move_simulation_robot(robot = robot, path= pick1_path1.q, env= env, dt = 0.05, gripper = gripper, cam = cam, TCR = TCR, TGR = TGR)
arrived = True
send_action_client(arrived, pick1_path1.q, goal, start_time, client, speed=2)

while not rospy.is_shutdown():
    pub.publish(closeGripper(400))
    rospy.sleep(0.25)
    break

pick1_way3 = [56.74, -86.43, 46.07, -51.44, -88.32, 234.33]
pick1_way3 = [math.radians(joint_home_degree) for joint_home_degree in pick1_way3]

path_up_pickway1 = move_up_down(robot, pick1_path1.q[-1], lift = 0.005) 
path_to_pickway1 = rtb.jtraj(path_up_pickway1.q[-1], pick1_way3, 30)

path_pickway1 = []
for q in path_up_pickway1.q:
    path_pickway1.append(q)

for q in path_to_pickway1.q:
    path_pickway1.append(q)

move_simulation_robot(robot = robot, path= path_pickway1, env= env, dt = 0.05, gripper = gripper, cam = cam, TCR = TCR, TGR = TGR)
arrived = True
send_action_client(arrived, path_pickway1, goal, start_time, client, speed=2)

# Move to drop battery
q_droppin_1 = [78.65, -96.40, 64.17, -58.12, -88.22, 252.57]
q_droppin_1 = [math.radians(joint_home_degree) for joint_home_degree in q_droppin_1]

path_droppin1 = rtb.jtraj(path_pickway1[-1], q_droppin_1, 30)
path_droppin2 = move_up_down(robot, path_droppin1.q[-1], 'down', lift = 0.02)

path_droppin_bat1 = []
for q in path_droppin1.q:
    path_droppin_bat1.append(q)
for q in path_droppin2.q:
    path_droppin_bat1.append(q)

move_simulation_robot(robot = robot, path= path_droppin_bat1, env= env, dt = 0.05, gripper = gripper, cam = cam, TCR = TCR, TGR = TGR)
arrived = True
send_action_client(arrived, path_droppin_bat1, goal, start_time, client, speed=1)

while not rospy.is_shutdown():
    pub.publish(moveGripperToPosition(400, 280))
    rospy.sleep(0.25)
    break

#-> Move to pin 2
pick2_way1 = [94.24, -79.7, 61.95, -74.67, -118.79, 84.73 + 360]
pick2_way1 = [math.radians(joint_home_degree) for joint_home_degree in pick2_way1]

pick2_way2 = [82.72, -69.40, 56.97, -72.54, -120.00, 82.96 + 360]
pick2_way2 = [math.radians(joint_home_degree) for joint_home_degree in pick2_way2]


pick2_path1 = rtb.mstraj(viapoints=np.array([path_droppin_bat1[-1], joint_home_radian, pick2_way1, pick2_way2]),dt=0.01,tacc=0.05,qdmax = pi)

move_simulation_robot(robot = robot, path= pick2_path1.q, env= env, dt = 0.05, gripper = gripper, cam = cam, TCR = TCR, TGR = TGR)
arrived = True
send_action_client(arrived, pick2_path1.q, goal, start_time, client, speed=3)

while not rospy.is_shutdown():
    pub.publish(closeGripper(300))
    rospy.sleep(0.25)
    break

pick2_way3 = [37.44, -81.68, 42.45, -50.46, -89.29, 41.75 + 360]
pick2_way3 = [math.radians(joint_home_degree) for joint_home_degree in pick2_way3]

path_up_pickway2 = move_up_down(robot, pick2_path1.q[-1], lift = 0.014) 
path_to_pickway2 = rtb.jtraj(path_up_pickway2.q[-1], pick2_way3, 30)

path_pickway2 = []
for q in path_up_pickway2.q:
    path_pickway2.append(q)

for q in path_to_pickway2.q:
    path_pickway2.append(q)


move_simulation_robot(robot = robot, path= path_pickway2, env= env, dt = 0.05, gripper = gripper, cam = cam, TCR = TCR, TGR = TGR)
arrived = True
send_action_client(arrived, path_pickway2, goal, start_time, client, speed=5)

# Move to drop battery 2 - REMEMBER TO CHANGE
q_droppin_1 = [78.65, -96.40, 64.17, -58.12, -88.22, 252.57] 
q_droppin_1 = [math.radians(joint_home_degree) for joint_home_degree in q_droppin_1]

path_droppin1 = rtb.jtraj(path_pickway2[-1], q_droppin_1, 30)
path_droppin2 = move_up_down(robot, path_droppin1.q[-1], 'down', lift = 0.02)

path_droppin_bat1 = []
for q in path_droppin1.q:
    path_droppin_bat1.append(q)
for q in path_droppin2.q:
    path_droppin_bat1.append(q)

move_simulation_robot(robot = robot, path= path_droppin_bat1, env= env, dt = 0.05, gripper = gripper, cam = cam, TCR = TCR, TGR = TGR)
arrived = True
send_action_client(arrived, path_droppin_bat1, goal, start_time, client, speed=1)

while not rospy.is_shutdown():
    pub.publish(moveGripperToPosition(400, 280))
    rospy.sleep(0.25)
    break

# env.hold()
rospy.spin()

## TEACH TEST -------------------------------------------------------------------------------------------------------- #
# # Set the end-effector joint limit
# robot.links[7].qlim = [-2*pi, 2*pi]

# # This is our callback function from the sliders in Swift which set
# # the joint angles of our robot to the value of the sliders
# def set_joint(j, value):
#     robot.q[j] = np.deg2rad(float(value))

# # Loop through each link in the robot and if it is a variable joint,
# # add a slider to Swift to control it
# j = 0
# for link in robot.links:
#     if link.isjoint:
#         # We use a lambda as the callback function from Swift
#         # j=j is used to set the value of j rather than the variable j
#         # We use the HTML unicode format for the degree sign in the unit arg
#         env.add(
#             swift.Slider(
#                 lambda x, j=j: set_joint(j, x),
#                 min=np.round(np.rad2deg(link.qlim[0]), 2),
#                 max=np.round(np.rad2deg(link.qlim[1]), 2),
#                 step=1,
#                 value=np.round(np.rad2deg(robot.q[j]), 2),
#                 desc="robot Joint " + str(j),
#                 unit="&#176;",
#             )
#         )

#         j += 1

# while True:
#     # Process the event queue from Swift, this invokes the callback functions
#     # from the sliders if the slider value was changed
#     # env.process_events()

#     # Update the environment with the new robot pose
#     dt = 0
#     move_simulation_robot(robot = robot, path= [robot.q], env= env, dt = dt, gripper = gripper, cam = cam, pin = coin, TCR = TCR, TGR = TGR, TCP = TCP)
#     time.sleep(0.01)
## -------------------------------------------------------------------------------------------------------- #
