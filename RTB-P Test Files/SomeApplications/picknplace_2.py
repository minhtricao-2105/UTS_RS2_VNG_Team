import roboticstoolbox as rtb
import numpy as np
from spatialmath import SE3
import swift
import random
import spatialgeometry.geom as collisionObj
from math import pi
from time import sleep

# relative_pose is the relative pose between the end-effector and the obj
def move_robot(robot, path, obj = None, relative_pose = None):
    print('Unsafe movement!')
    for q in path:
            robot.q = q
            if obj and relative_pose:
                pick = SE3(obj.T)
                pick = SE3(robot.fkine(q).A)* relative_pose.inv() 
                obj.T = pick.A
            env.step(0.05)
    
## environment, robot and object
env = swift.Swift()
env.launch(realtime= "True")

robot = rtb.models.UR3()
robot.q[1] = -pi/2 
env.add(robot)

ground = collisionObj.Cuboid(scale = [20,20,0.1], pose = SE3(0,0,-0.03), color = (0.13,0.12,0.1,1))
env.add(ground)

obj = collisionObj.Cuboid(scale = [0.2,0.1,0.1], pose = SE3(0.45,0,0.05),color = (0.1,0.5,0.1,1))
print("-object pose at first:\n",obj.T)
env.add(obj)

## end-effector pose for picking
# NOT CORRECT: obj.T * SE3.Ry(pi/2))
# rather do:
pick_pose = SE3(obj.T)
relative_pose = SE3.Tz(0.05) * SE3.Ry(pi/2) # relative pose between the end-effector and the obj
pick_pose = pick_pose * relative_pose

# Right way to update object pose:  obj.T = SE3(...).A
print(pick_pose)
pick_joint_LM = robot.ikine_LM(Tep = pick_pose,q0 = robot.q, joint_limits= True) # Numerical inverse kinematics by default Levenberg-Marquadt optimization
can_solve_LM = pick_joint_LM.success
if can_solve_LM:
    path = rtb.jtraj(robot.q,pick_joint_LM.q,50)
move_robot(robot,path.q)
move_robot(robot,reversed(path.q),obj,relative_pose)

## end-effector pose for placing
place_pose = SE3(-0.3,-0.3,0.05)*relative_pose
place_joint_LM = robot.ikine_LM(Tep = place_pose, q0 = robot.q, joint_limits= True) # Numerical inverse kinematics by default Levenberg-Marquadt optimization
can_solve_LM = place_joint_LM.success
if can_solve_LM:
    path = rtb.jtraj(robot.q,place_joint_LM.q,50)
else: print("CANT DO!")

move_robot(robot,path.q,obj,relative_pose)
move_robot(robot,reversed(path.q))
env.hold()