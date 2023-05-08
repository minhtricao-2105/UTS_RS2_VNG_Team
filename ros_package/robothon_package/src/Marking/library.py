import roboticstoolbox as rtb
import swift
from spatialmath import SE3
import numpy as np
from numpy import pi

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
def move_robot(r, q_list, env):    
    for q in q_list:
        r.q = q
        env.step(0.05)

def solve_joint(q,r1):

    r1_start_T = r1.fkine(q)
    r1_final_1 = r1_start_T*SE3.Tz(-0.1)
    r1_final_2 = r1_final_1*SE3.Ty(-0.1)
    r1_final_3 = r1_final_2*SE3.Tz(0.1)
    r1_final_4 = r1_final_3*SE3.Ty(0.1)


    # trajectory ctraj
    path_1 = rtb.ctraj(r1_start_T, r1_final_1, 100)
    path_2 = rtb.ctraj(r1_final_1, r1_final_2, 100)
    path_3 = rtb.ctraj(r1_final_2, r1_final_3, 100)
    path_4 = rtb.ctraj(r1_final_3, r1_final_4, 100)


    # joint trajectory
    q_list1 = poses_to_joints(r1,path_1)
    q_list2 = poses_to_joints(r1,path_2)
    q_list3 = poses_to_joints(r1,path_3)
    q_list4 = poses_to_joints(r1,path_4)

    rectangle_path = []

    for q in q_list1:
        rectangle_path.append(q)

    for q in q_list2:
        rectangle_path.append(q)

    for q in q_list3:
        rectangle_path.append(q)

    for q in q_list4:
        rectangle_path.append(q)
    
    return rectangle_path
