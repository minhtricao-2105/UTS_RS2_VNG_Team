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
def move_robot(r, q_list):    
    for q in q_list:
        r.q = q
        env.step(0.05)

def solve_joint(r1,q):
    # end pose
    r1_start_T = r1.fkine(q)
    r1_final_1 = r1_start_T*SE3.Tz(-0.1)
    r1_final_2 = r1_final_1*SE3.Ty(-0.1)
    r1_final_3 = r1_final_2*SE3.Tz(0.1)
    r1_final_4 = r1_final_3*SE3.Ty(0.1)

    # trajectory ctraj
    path_1 = rtb.ctraj(r1_start_T, r1_final_1, 50)
    path_2 = rtb.ctraj(r1_final_1, r1_final_2, 50)
    path_3 = rtb.ctraj(r1_final_2, r1_final_3, 50)
    path_4 = rtb.ctraj(r1_final_3, r1_final_4, 50)

    # joint trajectory
    q_list1 = poses_to_joints(r1,path_1)
    q_list2 = poses_to_joints(r1,path_2)
    q_list3 = poses_to_joints(r1,path_3)
    q_list4 = poses_to_joints(r1,path_4)

    q_rec =[]
    for q in q_list1:
        q_rec.append(q)

    for q in q_list2:
        q_rec.append(q)

    for q in q_list3:
        q_rec.append(q)

    for q in q_list4:
        q_rec.append(q)
    
    return q_rec


# import robots and set position
r1 = rtb.models.UR3()
q_ini = [0, -pi/2, pi/2, 0, pi/2, 0]
r1.q = q_ini

# set up swift environment
env = swift.Swift()
env.launch(realtime=True)
env.add(r1)

total_path = []

for i in range(4):
    if i != 0:
        q_ini[0] = q_ini[0] + pi/2
        rotate = rtb.jtraj(r1.q,q_ini,50).q
        move_robot(r1, rotate)
    
    q_rec = solve_joint(r1,q_ini)
    
    move_robot(r1,q_rec)
    print('done!')


# q_new = [pi/2, -pi/2, pi/2, 0, pi/2, 0]
# move_robot(r1, rtb.jtraj(r1.q, q_new, 30).q)

# move_robot(r1,q_rec)
env.hold()
