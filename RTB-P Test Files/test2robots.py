import roboticstoolbox as rtb
import swift
from spatialmath import SE3
import numpy as np
from numpy import pi
import threading

# move robot function
def move_robot(r, path):
    for q in path.q:
        r.q = q
        env.step(0.05)

initial_joints = np.array([0,-pi/2,pi/4,0,0,0])

# import robots and set position
r1 = rtb.models.UR3()
r2 = rtb.models.UR10()

r1.q = initial_joints
r2.q = initial_joints
r2.base = SE3(0,2,0)

# set up swift environment
env = swift.Swift()
env.launch(realtime=True)
env.add(r1)
env.add(r2)

# end pose
r1_start_T = r1.fkine(initial_joints)
r1_final_T = r1_start_T*SE3.Tx(0.1)*SE3.Ry(pi/4)

r2_start_T = r2.fkine(initial_joints)
r2_final_T = SE3.Tx(0.5)*r2_start_T

# end joints
r1_final_q = r1.ikine_LM(r1_final_T, q0=initial_joints)
r2_final_q = r2.ikine_LM(r2_final_T, q0=initial_joints)

# trajectory jtraj
path_1 = rtb.jtraj(r1.q, r1_final_q.q, 50)
path_2 = rtb.jtraj(r2.q, r2_final_q.q, 50)

# run 2 threads
t1 = threading.Thread(target=move_robot, args=(r1, path_1))
t2 = threading.Thread(target=move_robot, args=(r2, path_2))

t1.start()
t2.start()

t1.join()
t2.join()

env.hold()