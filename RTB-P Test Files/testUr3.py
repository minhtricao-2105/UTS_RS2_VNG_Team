import roboticstoolbox as rtb
import numpy as np
from spatialmath import SE3

robot = rtb.models.DH.UR3()
print(robot)

T1 = SE3(0.1, 0.1, 0.1) * SE3.Rz(np.pi/4)

q1 = robot.ikine_LM(T1)
 
print(q1)
print(robot.fkine(q1.q))

print(dir(np))
#robot.plot(q1.q, backend = "Swift")


