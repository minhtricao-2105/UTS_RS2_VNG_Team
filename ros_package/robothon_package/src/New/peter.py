from math import pi
from spatialmath import SE3
import roboticstoolbox as rtb 
import time

def move():
    # Set the initial and final Cartesian poses
    p1 = SE3(0.5, 0, 0.5) 
    p2 = SE3(0.5, 0.2, 0.5) * SE3.Rx(-pi/2) * SE3.Ry(pi/4)

    # Use the Robotics Toolbox to generate a joint trajectory between the initial and final Cartesian poses
    robotClone = rtb.models.UR3()
    q1 = robotClone.ikine_LM(p1)
    q2 = robotClone.ikine_LM(p2)

    # create a path
    path = rtb.jtraj(q0=q1.q,qf = q2.q, t = 50)
    
    return path