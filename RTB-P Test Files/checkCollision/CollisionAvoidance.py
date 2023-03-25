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
    if min(height) > 0.1:
        return False
    else: 
        return True

# check collision with the obstacle
def isCollision(robot,q,obstacle):
    if robot.iscollided(q,obstacle,True):
        return True
    else:
        return False

# function to move robot back
count = 1
def Move_back(robot, qpath):
    print('Unsafe movement!')
    for q in reversed(qpath):
            robot.q = q
            env.step(0.07)
    

def Move_robot(robot, q_end):
    global count
    #path =rtb.jtraj(robot.q,q_end,t=50)
    path = rtb.mtraj(tfunc = rtb.trapezoidal,q0 = robot.q, qf = q_end,t=60)
    for i in range(len(path)):
        # check each link
        touchGround= isTouchGround(robot,path.q[i])
        touchObject = isCollision(robot,path.q[i],obstacle)
        if not touchGround and not touchObject:
            #print('Move normal!')
            robot.q = path.q[i]
            env.step(0.05)
        elif touchGround and not touchObject:
            count = count + 1
            print(count,'.May touch ground, another path!')
            if i > 2: Move_back(robot,path.q[int(0.8*i):i])
            else: break
            break
        elif touchObject:
            count = count + 1
            print(count,'.Get collision, another path!')
            if i > 2: Move_back(robot,path.q[int(0.8*i):i])
            else: break
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

    if isCollision(robot,q_rand,obstacle) or isTouchGround(robot,q_rand): continue
    
    Move_robot(robot,q_rand)

# q_end = robot.qr
# path = rtb.jtraj(robot.q, q_end,50)

# for q in path.q:
#     robot.q = q
#     env.step(0.05)

# Move_back(robot,path.q[35:49])
