import roboticstoolbox as rtb
import numpy as np
from spatialmath import SE3
import swift
import random
import spatialgeometry.geom as collisionObj
from math import pi
from time import sleep

## environment, robot and object
env = swift.Swift()
env.launch(realtime= "True")

robot = rtb.models.UR3()
robot.q[1] = -pi/2 
env.add(robot)

ground = collisionObj.Cuboid(scale = [20,20,0.1], color = (0.13,0.12,0.1,0.1))
ground.T[2,3] = -0.1
env.add(ground)

obj = collisionObj.Cuboid(scale = [0.2,0.1,0.1], pose = SE3(0.45,0,0.05),color = (0.1,0.5,0.1,1))
print("-object pose at first:\n",obj.T)
env.add(obj)

## end-effector pose for picking
# NOT CORRECT: obj.T * SE3.Ry(pi/2))
# rather do:
pick_pose = SE3(obj.T)
pick_pose = pick_pose * SE3.Tz(0.05) * SE3.Ry(pi/2)
# Right way to update object pose:  obj.T = SE3(...).A
print(pick_pose)

## different ik solvers:
pick_joint_LM = robot.ikine_LM(Tep = pick_pose,q0 = robot.q, joint_limits= True) # Numerical inverse kinematics by default Levenberg-Marquadt optimization
can_solve_LM = pick_joint_LM.success

pick_joint_LM_chan = robot.ik_lm_chan(Tep = pick_pose,q0 = robot.q) # Numerical inverse kinematics by Levenberg-Marquadt optimization (Chan's Method)
can_solve_LM_chan = pick_joint_LM_chan[1]

pick_joint_LM_sugi = robot.ik_lm_sugihara(Tep = pick_pose,q0 = robot.q) # Numerical inverse kinematics by Levenberg-Marquadt optimization (Sugihara's Method)
can_solve_LM_sugi = pick_joint_LM_sugi[1]

pick_joint_LM_nr = robot.ik_nr(Tep = pick_pose,q0 = robot.q) # Numerical inverse kinematics by Levenberg-Marquadt optimization ((Newton-Raphson Method)
can_solve_LM_nr = pick_joint_LM_nr[1]

pick_joint_LM_gn = robot.ik_gn(Tep = pick_pose,q0 = robot.q) # Numerical inverse kinematics by Levenberg-Marquadt optimization (Gauss-NewtonMethod)
can_solve_LM_gn = pick_joint_LM_gn[1]

pick_joint_LM_wl = robot.ik_lm_wampler(Tep = pick_pose,q0 = robot.q) # Numerical inverse kinematics by Levenberg-Marquadt optimization (Wamplers's Method)
can_solve_LM_wl = pick_joint_LM_wl[1]

    # Show solutions:
if can_solve_LM: 
    print("A solution using LM can be found!")
    print("Cost:",pick_joint_LM.residual,'\n')
    robot.q = pick_joint_LM.q
    env.step(0.05)
    sleep(2)
else: print("Cannot solve using LM.",'\n')

# if can_solve_LM_chan: 
#     print("A solution using LM-chan can be found!")
#     print("Cost:",pick_joint_LM_chan[4],'\n')
#     robot.q = pick_joint_LM_chan[0]
#     env.step(0.05)
#     sleep(2)
# else: print("Cannot solve using LM-chan!",'\n')

# if can_solve_LM_sugi:
#     print("A solution using LM-sugihara can be found!")
#     print("Cost:",pick_joint_LM_sugi[4],'\n')
#     robot.q = pick_joint_LM_sugi[0]
#     env.step(0.05)
#     sleep(2)
# else: print("Cannot solve using LM-sugihara",'\n')

# if can_solve_LM_nr:
#     print("A solution using LM-newtonraphson can be found!")
#     print("Cost:",pick_joint_LM_nr[4],'\n')
#     robot.q = pick_joint_LM_nr[0]
#     env.step(0.05)
#     sleep(2)
# else: print("Cannot solve using LM-newtonraphson!",'\n')

# if can_solve_LM_gn:
#     print("A solution using LM-gaussnewton can be found!")
#     print("Cost:",pick_joint_LM_gn[4],'\n')
#     robot.q = pick_joint_LM_gn[0]
#     env.step(0.05)
#     sleep(2)
# else: print("Cannot solve using LM-gaussnewton!",'\n')

# if can_solve_LM_wl:
#     print("A solution using LM-wampler can be found!")
#     print("Cost:",pick_joint_LM_wl[4],'\n')
#     robot.q = pick_joint_LM_wl[0]
#     env.step(0.05)
#     sleep(2)
# else: print("Cannot solve using LM-wampler!",'\n')

# env.step(0.05)
env.hold()

