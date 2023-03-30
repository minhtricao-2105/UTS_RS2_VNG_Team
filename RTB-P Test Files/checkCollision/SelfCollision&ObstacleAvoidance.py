import roboticstoolbox as rtb
import numpy as np
from spatialmath import SE3
import swift
import random
import spatialgeometry.geom as collisionObj
from math import pi

# return an list include <class 'spatialmath.pose3d.SE3'> object as transform of each link given joint state
def get_link_transform(robot,q):
    T = [] # Tranforms array of link
    for i in range(len(q)):
        T.append(robot.fkine(q,end = robot.links[i].name)) 
    return T

# check whether the robot touches the ground
def is_touch_ground(robot,q):
    T = get_link_transform(robot,q)
    height = [T[i].A[2,3] for i in range(2,6)]
    if min(height) > 0.1:
        return False
    else: 
        return True

# check for self collision at a joint coordinate q, see self-collision text file
roboClone = rtb.models.UR3() # make a clone for self collision check
def is_self_collision_UR3(q):
    global roboClone
    roboClone.q = q
    for i in reversed(range(8)[1:8]):
        stopCheck = False
        if i == 2 :break
        for j in reversed(range(i - 1)[1:i-1]):
            isCollide = roboClone.links[i].iscollided(shape = roboClone.links[j].collision[0])
            if isCollide: 
                stopCheck = True
                print("--COLLISION DETECT:",roboClone.links[i].name," will collide with ",roboClone.links[j].name)
                break    
        if stopCheck: break
    return stopCheck

# check collision with the obstacle
def is_collision_obstacle(robot,q,obstacleList):
    isCollide = False
    for obstacle in obstacleList:
        if robot.iscollided(q,obstacle,True):
            isCollide = True
            break
    return isCollide

#########################################
# move UR3/e robot with self-collsion detect incorporate
def move_robot(robot,qEnd):
    print("**TRY TO MOVE ALONG PATH:")
    global count
    global stopLoop
    # path = rtb.jtraj(q0 = robot.q, qf = qEnd,t=60)
    path = rtb.mtraj(tfunc=rtb.trapezoidal,q0 = robot.q,qf=qEnd,t =50)
    pathFinished = True
    for i in range(len(path)):
        # check each link
        touchGround= is_touch_ground(robot,path.q[i])
        touchSelf = is_self_collision_UR3(path.q[i])
        touchObstacle = is_collision_obstacle(robot,path.q[i],obstacleList) 
        if not touchGround and not touchSelf and not touchObstacle:
            #print('Move normal!')
            robot.q = path.q[i]
            env.step(0.05)
        else:
            if touchGround: print(count,'.May touch ground at step', i ,', another path!')
            if touchObstacle:  print(count,'.May touch obstacle at step', i ,', another path!')
            if touchObstacle: print(count,'.Get self collision at step', i ,', another path!')
            
            count = count + 1
            pathFinished = False 
            if i >= 2: move_back(robot,path.q[int(0.8*i):i])
            elif i==1: stopLoop = True
            break
                 
    print("Path finished:", pathFinished)
    return pathFinished

# function to move robot back
count = 1
def move_back(robot, qpath):
    print('Unsafe movement!')
    for q in reversed(qpath):
            robot.q = q
            env.step(0.07)

#############################################
# RUN #
env = swift.Swift()
env.launch(realtime= "True")

# Make a robot and collision objects and add to Swift
robot = rtb.models.UR3()
robot.q[1] = -np.pi/2 

obstacleList = []
obstacleList.append(collisionObj.Sphere(radius = 0.05, pose = SE3(0.2,0.2,0.3), color = (0.1,0.1,0.5,1)))
obstacleList.append(collisionObj.Sphere(radius = 0.05, pose = SE3(-0.2,0.2,0.5), color = (0.1,0.4,0.5,1)))
obstacleList.append(collisionObj.Cuboid(scale = [0.08,0.08,0.08], pose = SE3(0.2,-0.4,0.5), color = (0.3,0.2,0.5,1)))
obstacleList.append(collisionObj.Cuboid(scale = [0.1,0.1,0.1], pose = SE3(0.2,-0.3,0.3), color = (0.5,0.2,0.2,1)))

env.add(robot)

for obstacle in obstacleList:
    env.add(obstacle)

# run the robot randomly
stopLoop = False
while True:
    if stopLoop:
        print("Can't recover. Stop moving!")
        break

    q_rand = np.array([random.randint(-180,180)*pi/180 for _ in range(robot.n)])
    selfCo = is_self_collision_UR3(q_rand)
    groundCo = is_touch_ground(robot,q_rand)
    obCo = is_collision_obstacle(robot,q_rand,obstacleList)
    print("---Self collision: ",selfCo,"; Ground collision:",groundCo,"; Obstacle collision:", obCo)    
    
    if selfCo or groundCo or obCo:
        print("->Nonreachable goal! Regenerate final q!")
        continue
    else: 
        move_robot(robot,q_rand)
    print("----\n\n")

env.hold()