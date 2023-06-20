from function import rtb, collisionObj,SE3, pi, swift, np, cam_move, move_simulation_robot
from spatialmath.base import *
from ur3module import *
# import time
from function import *

pick2_way1 = [94.24, -79.7, 61.95, -74.67, -118.79, 84.73 + 360]
pick2_way1 = [math.radians(joint_home_degree) for joint_home_degree in pick2_way1]

env = swift.Swift()

# Enviroment
env = swift.Swift()
env.launch(realtime=True)
dt = 0.05

robot = rtb.models.UR3()

cam = collisionObj.Cuboid(scale=[0.03,0.06,0.015], pose = SE3(0,0,0), color = [0.5,0.5,0.1,1])
TCR = SE3(0.085,0,0.09)*SE3.Ry(pi/2) #cam pose in ee frame
# cam_move(cam, robot, TCR)

# Gripper
TGR = SE3.Rx(pi)*SE3(0,-0.105,-0.175) #gripper pose in ee frame
# gripper_path = "/home/minhtricao/robothon2023/RTB-P Test Files/SomeApplications/CAMGRIPPER.STL"
gripper_path = "/home/quangngo/Robotics Studio 2/GroupGit/robothon2023/RTB-P Test Files/SomeApplications/CAMGRIPPER.STL"
# gripper_path = "D:/UTS_RS2_VNG_Team/RTB-P Test Files/SomeApplications/CAMGRIPPER.STL"
gripper = collisionObj.Mesh(filename=gripper_path,pose = SE3(0,0,0),scale=[0.001, 0.001, 0.001],color = [0.5,0.1,0.1,1])
# cam_move(gripper,robot,TGR)

robot.q = pick2_way1
env.add(robot)
env.add(cam)
env.add(gripper)

move_simulation_robot(robot = robot, path= [pick2_way1], env= env, dt = 0.05, gripper = gripper, cam = cam, TCR = TCR, TGR = TGR)


# ee_transf = SE3()
rot_cen = SE3(0.02, 0, -0.01) * robot.fkine(pick2_way1)
rot_y = SE3.Ry(10*pi/180)

final_ee_pose = rot_cen.inv() * rot_y * rot_cen*robot.fkine(pick2_way1)
q_sol = solve_for_valid_ik(robot= robot, obj_pose = final_ee_pose, q_guess = pick2_way1)
q_sol[-1] = pick2_way1[-1]

path = rtb.jtraj(pick2_way1, q_sol, 50)
move_simulation_robot(robot = robot, path= path.q, env= env, dt = 0.05, gripper = gripper, cam = cam, TCR = TCR, TGR = TGR)



env.hold()



