import swift
import roboticstoolbox as rtb
import numpy as np
from numpy import pi
from spatialmath import SE3
import random
import spatialgeometry.geom as collisionObj

# return an list include <class 'spatialmath.pose3d.SE3'> object as transform of each link given joint state
def GetLinkTransform(robot,q):
    T = [] # Tranforms array of link
    for i in range(len(q)):
        T.append(robot.fkine(q,end = robot.links[i].name)) 
    return T

# check whether the robot touches the ground
def isTouchGround(robot,q):
    T = GetLinkTransform(robot,q)
    height = [T[i].A[2,3] for i in range(2,6)]
    if min(height) > 0.05:
        return False
    else: 
        return True

# check collision with the obstacle
def isCollision(robot,q,obstacle):
    if robot.iscollided(q,obstacle,True):
        return True
    else:
        return False

# function to move robot using jtraj
count = 1
def Move_robot_raw(robot, q_end):
    print('Unsafe movement!')
    path =rtb.jtraj(robot.q,q_end,t=50)
    for q in path.q:
        robot.q = q
        env.step(0.05)

def Move_robot(robot, q_end):
    global count
    path =rtb.jtraj(robot.q,q_end,t=100)
    for i in range(len(path)):
        # check each link
        if not isTouchGround(robot,path.q[i]) and not isCollision(robot,path.q[i],obstacle):
            print('Move normal!')
            robot.q = path.q[i]
            env.step(0.05)
        elif isTouchGround(robot,path.q[i]) and not isCollision(robot,path.q[i],obstacle):
            count = count + 1
            print(count,'.May touch ground, another path!')
            Move_robot_raw(robot,path.q[i-1])
            break
        elif isCollision(robot,path.q[i],obstacle):
            count = count + 1
            print(count,'.Get collision, another path!')
            Move_robot_raw(robot,path.q[i-1])
            break



env = swift.Swift()
env.launch(realtime= "True")

# Make a robot and a collision object add to Swift
robot = rtb.models.UR3()
robot.q[1] = -np.pi/2 
obstacle = collisionObj.Sphere(radius = 0.08, pose = SE3(0.3,0.1,0.5), color = (0.8,0.5,0.5,1))

env.add(robot)
env.add(obstacle)

# run the robot randomly
while True:
    q_rand = np.array([random.randint(-180,180)*pi/180 for _ in range(robot.n)])   
    Move_robot(robot,q_rand)