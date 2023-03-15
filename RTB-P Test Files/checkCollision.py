import roboticstoolbox as rtb
import numpy as np
import spatialmath as sptm
from spatialmath import SE3
import swift

robot = rtb.models.Panda()
obstacle = rtb.Box([1, 1, 1], SE3(1, 0, 0))

env = swift.Swift()
env.launch(realtime="True")
env.add(robot)
env.add(obstacle)

iscollision = robot.iscollided(obstacle)
print(iscollision)

iscollision = robot.links[0].iscollided(obstacle)
print(iscollision)