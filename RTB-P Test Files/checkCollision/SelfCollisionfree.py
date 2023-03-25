import roboticstoolbox as rtb
import numpy as np
from spatialmath import SE3
import swift
import random
import spatialgeometry.geom as collisionObj
from math import pi


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

# check for self collision at a joint coordinate q, see self-collision text file
def isSelfCollision_UR3(robot,q):
    qHolder = robot.q
    robot.q = q
    for i in reversed(range(8)[1:8]):
        stopCheck = False
        for j in reversed(range(i - 1)[1:i]):
            isCollide = robot.links[i].iscollided(shape = robot.links[j].collision[0], skip =True)
            if isCollide: 
                stopCheck = True
                robot.q = qHolder
                break    
        if stopCheck: break
    robot.q = qHolder
    return stopCheck

#function to move robot back
count = 1
def Move_back(robot, qpath):
    print('Unsafe movement!')
    for q in reversed(qpath):
            robot.q = q
            env.step(0.05)

# move UR3/e robot with self-collsion detect incorporate
def MoveUR3(robot,qEnd):
    global count
    # path = rtb.jtraj(q0 = robot.q, qf = qEnd,t=60)
    path = rtb.mtraj(tfunc=rtb.trapezoidal,q0 = robot.q,qf=qEnd,t =60)
    pathFinished = True
    for i in range(len(path)):
        # check each link
        touchGround= isTouchGround(robot,path.q[i])
        touchSelf = isSelfCollision_UR3(robot,path.q[i])
        
        if not touchGround and not touchSelf:
            #print('Move normal!')
            robot.q = path.q[i]
            env.step(0.05)
        elif touchGround and not touchSelf:
            count = count + 1
            print(count,'.May touch ground, another path!')
            if i > 2: Move_back(robot,path.q[int(0.8*i):i])
            else:
                print("i=",i,"Stuck!") 
                break
            break
        elif touchSelf:
            count = count + 1
            print(count,'.Get self collision, another path!')
            if i > 2: Move_back(robot,path.q[int(0.8*i):i])
            else:
                print("i=",i,"Stuck!") 
                break
            break

    print("Path finished:", pathFinished)
    return pathFinished

#create robot and environment
robot = rtb.models.UR3()
robot.q[1] = -np.pi/2 

env = swift.Swift()
env.launch(realtime=True)
env.add(robot)

# run the robot randomly
while True:
    q_rand = np.array([random.randint(-180,180)*pi/180 for _ in range(robot.n)])   
    selfCo = isSelfCollision_UR3(robot,q_rand)
    groundCo = isTouchGround(robot,q_rand)
    print("Self collision: ",selfCo," and Ground collision:",groundCo)    
    if selfCo or groundCo:
        print("Regenerate final q!")
        continue
    else: 
        MoveUR3(robot,q_rand)




