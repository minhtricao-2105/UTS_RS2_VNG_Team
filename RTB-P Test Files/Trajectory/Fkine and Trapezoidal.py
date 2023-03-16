import roboticstoolbox as rtb
import numpy as np
from numpy import pi
from swift import Swift
from spatialmath import SE3
import random

# return an list include <class 'spatialmath.pose3d.SE3'> object as transform of each link given joint state
def GetLinkTransform(robot,q):
    T = [] # Tranforms array of link
    for i in range(len(q)):
        T.append(robot.fkine(q,end = robot.links[i].name)) 
    return T

# function to move robot using jtraj
def Move_robot(robot, q_end):
    #path =rtb.trapezoidal(robot.q,q_end,t =50)
    path =rtb.mtraj(tfunc = rtb.trapezoidal,q0 = robot.q, qf = q_end,t=50)
    for q in path.q:
        # check height condition for each link
        height = [GetLinkTransform(robot,q)[i].A[2,3] for i in range(2,6)]
        if min(height) > 0.15:
            robot.q = q
            env.step(0.05)
        else: 
            print('Not finish current path!')
            break

# create a robot from a URDF file
robot = rtb.models.Panda()
robot.q[1] = -pi/2

# create an environment
env = Swift()
env.launch(realtime="True")
env.add(robot)
robot.collided

# run the robot randomly
while True:
    q_rand = np.array([random.randint(-180,180)*pi/180 for _ in range(robot.n)])   
    Move_robot(robot,q_rand)








