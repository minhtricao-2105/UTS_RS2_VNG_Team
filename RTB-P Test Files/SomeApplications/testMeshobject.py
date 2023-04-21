import sys
sys.path.append('../Modules')
from ur3module import *
from pathgenerator import *

# create environment 
env = swift.Swift()
env.launch(realtime=True)

robot = rtb.models.UR3()
q_ini = [42,-28,61,-123,-87,0]
q_ini = [x*pi/180 for x in q_ini]

q_end = [-60,-28,61,-123,-87,0]
q_end = [x*pi/180 for x in q_end]
# create a goal object 
goal_obj = collisionObj.Sphere(radius = 0.02, pose = robot.fkine(q_end),color = (0.5,0.1,0.1,1))
env.add(goal_obj)

robot.q = q_ini
env.add(robot)

path_obj = "/home/quangngo/Desktop/RTB-P Test files/SomeApplications/Robothon_Assembly_Box - Robothon_Assembly_Box.STEP-1 Robothon Box.STEP-1.STL"
box = collisionObj.Mesh(filename=path_obj,pose = SE3(0,0,0),scale=[0.001, 0.001, 0.001], color = (0.2,0.3,0.1,1))

# correct position
box_pose = SE3.Rx(pi/2)*SE3.Ry(pi/2)*SE3(box.T)
box_pose = SE3(0,-0.17,-0.12)*box_pose
box.T = box_pose.A

env.add(box)

# create collision free path
path,success = gen_path(robot=robot,q_goal=q_end,obstacle_list=[box])

if success: 
    show_path(robot,path,env)
    # move robot along path
    for q in path:
        robot.q = q
        env.step(0.05)

env.hold()