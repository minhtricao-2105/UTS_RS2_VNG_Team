import roboticstoolbox as rtb
import numpy as np
from spatialmath import SE3
import swift
import random
import spatialgeometry.geom as collisionObj
from math import pi


robot = rtb.models.UR3()
robot.q = [0, -pi/2, pi/3, 0, 0, 0]

env = swift.Swift()
env.launch(realtime=True)
env.add(robot)

final_pose = SE3.Tz(-0.35)*robot.fkine(robot.q)

arrived = False
dt = 0.05

while not arrived:
    v, arrived = rtb.p_servo(robot.fkine(robot.q), final_pose, 1)
    robot.qd = np.linalg.pinv(robot.jacobe(robot.q)) @ v
    env.add(collisionObj.Sphere(radius=0.005, pose = robot.fkine(robot.q),color = (0.1,0.5,0.1,1)))
    env.step(dt)

env.hold()
