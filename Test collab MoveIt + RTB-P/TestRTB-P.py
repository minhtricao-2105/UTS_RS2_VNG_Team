from math import pi
from spatialmath import SE3
import roboticstoolbox as rtb 
import swift


robotClone = rtb.models.UR3()
joint_home = [1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0]
robotClone.q = joint_home

T = SE3(0.3, 0.2, 0.2) * SE3.Rx(-pi/2) * SE3.Ry(pi/4)
joint_end = robotClone.ikine_LM(T,q0 = robotClone.q).q

path = rtb.mtraj(tfunc = rtb.trapezoidal, q0 = joint_home, qf = joint_end, t = 100)


env = swift.Swift()
env.launch(realtime=True)
env.add(robotClone)
        
for q in path.q:
    robotClone.q = q
    env.step(0.05)



env.hold()