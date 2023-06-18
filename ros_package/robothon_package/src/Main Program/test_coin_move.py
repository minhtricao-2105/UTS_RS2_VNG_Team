from function import rtb, collisionObj,SE3, pi, swift, np, cam_move, move_simulation_robot
from spatialmath.base import *
from ur3module import *
import time

# Clone
robot = rtb.models.UR3() 
q_deg = [45.49, -81.63, 63.85, -73.31, -86.16, 319.28 - 360] # First position of the robot:
q_start = [x*pi/180 for x in q_deg]
robot.q = q_start

# Camera
cam = collisionObj.Cuboid(scale=[0.03,0.06,0.015], pose = SE3(0,0,0), color = [0.5,0.5,0.1,1])
TCR = SE3(0.085,0,0.09)*SE3.Ry(pi/2) #cam pose in ee frame
cam_move(cam, robot, TCR)

# Gripper
TGR = SE3.Rx(pi)*SE3(0,-0.105,-0.175) #gripper pose in ee frame
# gripper_path = "/home/minhtricao/robothon2023/RTB-P Test Files/SomeApplications/CAMGRIPPER.STL"
gripper_path = "/home/quangngo/Robotics Studio 2/GroupGit/robothon2023/RTB-P Test Files/SomeApplications/CAMGRIPPER.STL"
# gripper_path = "D:/UTS_RS2_VNG_Team/RTB-P Test Files/SomeApplications/CAMGRIPPER.STL"
gripper = collisionObj.Mesh(filename=gripper_path,pose = SE3(0,0,0),scale=[0.001, 0.001, 0.001],color = [0.5,0.1,0.1,1])
cam_move(gripper,robot,TGR)

# Coin
coin = collisionObj.Cylinder(radius = 0.01, length= 0.005, pose = SE3(0, 0, 0), color = [0.3,0.3,0.1,1])
TCP = SE3(0.23,0,0) * SE3.Rx(pi/2) #coin pose in ee frame
cam_move(coin, robot, TCP)

# Enviroment
env = swift.Swift()
env.launch(realtime=True)
env.add(robot)
env.add(cam)
env.add(gripper)
env.add(coin)
dt = 0.05

# Coin's desired position
coin_end = collisionObj.Cylinder(radius = 0.01, length= 0.005, pose = coin.T, color = [0.0,0.5,0.0,0.5])
rot_deg = 10
coin_end.T = transl(-0.002,0,0.004) @ coin_end.T @ troty(rot_deg*pi/180) # apply lifting and rotation
env.add(coin_end)

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
input("Press anything\n")
q_coin_end = solve_for_valid_ik(robot=robot, obj_pose= SE3(coin_end.T), relative_pose = TCP.inv(), q_guess = q_start)
path_coin = rtb.jtraj(q_start, q_coin_end, 20)
move_simulation_robot(robot = robot, path= path_coin.q, env= env, dt = 0.05, gripper = gripper, cam = cam, pin = coin, TCR = TCR, TGR = TGR, TCP = TCP)

env.hold()