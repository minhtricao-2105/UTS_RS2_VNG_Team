import roboticstoolbox as rtb
import swift
from spatialmath import SE3
import numpy as np
from numpy import pi
import threading

# convert poses to joints path
def poses_to_joints(r,path):
    q_list = np.empty((len(path),6))
    q_list = np.empty((len(path),6))
    for i in range(len(path)):
        if i == 0:
            q_list[i] = r.ikine_LM(path[i],q0 = r.q).q
        else:
            q_list[i] = r.ikine_LM(path[i], q0=q_list[i-1]).q

        if (100*i/len(path))%10 == 0: 
            print(100*i/len(path),"%!")
        
    return q_list

# move robot function
def move_robot(r, q_list):    
    for q in q_list:
        r.q = q
        env.step(0.05)

initial_joints = np.array([0,-pi/2,pi/4,0,0,0])

# import robots and set position
r1 = rtb.models.UR3()
r2 = rtb.models.UR3()

r1.q = initial_joints
r2.q = initial_joints
r2.base = SE3(0,1,0)

# set up swift environment
env = swift.Swift()
env.launch(realtime=True)
env.add(r1)
env.add(r2)

# end pose
r1_start_T = r1.fkine(initial_joints)
r1_final_T = r1_start_T*SE3.Tx(0.2)

r2_start_T = r2.fkine(initial_joints)
r2_final_T = r2_start_T*SE3.Tx(0.2)

# trajectory ctraj
path_1 = rtb.ctraj(r1_start_T, r1_final_T, 50)
path_2 = rtb.ctraj(r2_start_T, r2_final_T, 50)

# joint trajectory
q_list1 = poses_to_joints(r1,path_1)
q_list2 = poses_to_joints(r2,path_2)

# run 2 threads
t1 = threading.Thread(target=move_robot, args=(r1, q_list1))
t2 = threading.Thread(target=move_robot, args=(r2, q_list2))

t1.start()
t2.start()

t1.join()
t2.join()

print('done')

env.hold()