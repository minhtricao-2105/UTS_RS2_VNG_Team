import roboticstoolbox as rtb
import numpy as np
from spatialmath import SE3
import swift
import random
import spatialgeometry.geom as collisionObj
from math import pi
import time
import threading

#### ROBOT MOVE #####################################################
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
roboClone = rtb.models.UR3() #make a clone for self collision check
def isSelfCollision_UR3(q):
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

# check collision with the obstacle list
def isCollisionObstacle(robot,q,obstacleList):
    isCollide = False
    for obstacle in obstacleList:
        if robot.iscollided(q,obstacle,True):
            isCollide = True
            break
    return isCollide

# move UR3/e robot with self-collsion detect incorporate
count = 1
def MoveRobot(robot,qEnd,obstacleList):
    print("**TRY TO MOVE ALONG PATH:")
    global stopLoop
    global count
    # path = rtb.jtraj(q0 = robot.q, qf = qEnd,t=60)
    path = rtb.mtraj(tfunc=rtb.trapezoidal,q0 = robot.q,qf=qEnd,t =50)
    pathFinished = True
    for i in range(len(path)):
        # check each link
        touchGround= isTouchGround(robot,path.q[i])
        touchSelf = isSelfCollision_UR3(path.q[i])
        touchObstacle = isCollisionObstacle(robot,path.q[i],obstacleList) 
        if not touchGround and not touchSelf and not touchObstacle:
            #print('Move normal!')
            robot.q = path.q[i]
            env.step(0.05)
        else:
            if touchGround: print(count,'.May touch ground at step', i ,', another path!')
            if touchObstacle:  print(count,'.May touch obstacle at step', i ,', another path!')
            if touchSelf: print(count,'.Get self collision at step', i ,', another path!')
            
            count = count + 1
            pathFinished = False 
            # if i >= 2: MoveBack(robot,path.q[int(0.8*i):i])
            # elif i==1: stopLoop = True
            if i == 1 : stopLoop = True
            break
                 
    print("Path finished:", pathFinished)
    return pathFinished

# function to move robot back
def MoveBack(robot, qpath):
    print('Unsafe movement!')
    for q in reversed(qpath):
            robot.q = q
            env.step(0.07)


# Randomly move the robot
stopLoop = False
def RunRobotRandom(obstacleList):
    while True:
        if stopLoop:
            print("Can't recover. Stop moving!")
            break
        q_rand = np.array([random.randint(-180,180)*pi/180 for _ in range(robot.n)])
        
        selfCo = isSelfCollision_UR3(q_rand)
        groundCo = isTouchGround(robot,q_rand)
        # obCo = isCollisionObstacle(robot,q_rand,obstacleList)
        
        print("---Self collision: ",selfCo,"; Ground collision:",groundCo) 
        if selfCo or groundCo:
            print("->Goal may touch ground or lead to self-collision! Regenerate final q!")
            continue
        else: 
            MoveRobot(robot,q_rand,obstacleList)
        print("----\n\n")

#### OBJECT MOVE ####################################
# create a point lists for a circle
def Circle():
    # Define the radius of the circle
    radius = 0.3

    # Define the distance from the z-axis
    distanceZ = 0.3

    # Define the number of points to generate
    num_points = 300

    # Generate an array of angles from 0 to 2*pi with a step of 2*pi/num_points
    angles = np.linspace(0, 2*np.pi, num_points, endpoint=False)

    x = radius * np.cos(angles) 
    y = radius * np.sin(angles)
    z = distanceZ * np.ones(num_points)

    # Create a list of [x, y, z] points
    points = np.stack([x, y, z], axis=1)

    # return the point list
    return points

# draw that circle
def CircleDisplay():
    points = Circle()  
    for p in points:
        dot = collisionObj.Sphere(radius=0.003,pose = SE3(p),color = (0.5,0.2,0.2,1))
        env.add(dot)

# move the input obstacle along the circular trajectory
def CircleMove(obstacle):
    trajectory = Circle()
    while True:
        for p in trajectory:
            obstacle.T = SE3(p)
            env.step(0.05)

#### RUN #######################################################################################
osbtacleList = []
obstacle = collisionObj.Cuboid(scale = [0.1,0.1,0.1], pose = SE3(0.2,0.3,0.6), color = (0.2,0.2,0.5,1))
osbtacleList.append(obstacle)

robot = rtb.models.UR3() 
robot.q[1] = -pi/2 

env = swift.Swift()
env.launch(realtime= "True")
env.add(obstacle)
env.add(robot)

CircleDisplay()

t1 = threading.Thread(target = RunRobotRandom, args = (osbtacleList,))
t2 = threading.Thread(target = CircleMove, args = (obstacle,))

t1.start()
t2.start()

t1.join()
t2.join()

print("All threads finished!")