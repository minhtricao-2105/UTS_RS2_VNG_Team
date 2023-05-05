import sys
sys.path.append('../Modules')
from ur3module import *

def cam_move(cam,robot,T):
    cam.T = robot.fkine(robot.q)*T

env = swift.Swift()
env.launch(realtime = True)
dt = 0.05

gripper_path = "/home/quangngo/Desktop/RTB-P Test files/SomeApplications/CAMGRIPPER.STL"
gripper = collisionObj.Mesh(filename=gripper_path,pose = SE3(0,0,0),scale=[0.001, 0.001, 0.001],color = [0.5,0.1,0.1,1])

cam = collisionObj.Cuboid(scale=[0.03,0.06,0.015], pose = SE3(0,0,0), color = [0.5,0.5,0.1,1])

q_deg = [57.11, -106.32, 56.84, -44.7, -85.81, 328.40]
robot = rtb.models.UR3()
robot.q = [x*pi/180 for x in q_deg]

#gripper in robot ee frame
TGR = SE3.Rx(pi)*SE3(0,-0.105,-0.175)
TCR = SE3(0.085,0,0.09)

cam_move(gripper,robot, TGR)
cam_move(cam, robot, TCR)

env.add(robot)
env.add(gripper)
env.add(cam)

q_end = [0,-pi/2,0,0,0,0]
path = rtb.jtraj(robot.q, q_end,50)

# for q in path.q:
#     robot.q = q
#     cam_move(gripper,robot, TGR)
#     cam_move(cam,robot,TCR)
#     env.step(dt)
print(cam.T)

env.hold()


