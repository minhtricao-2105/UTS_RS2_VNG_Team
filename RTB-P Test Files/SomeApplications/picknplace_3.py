import roboticstoolbox as rtb
import numpy as np
from spatialmath import SE3
import swift
import random
import spatialgeometry.geom as collisionObj
from math import pi

# return an list include <class 'spatialmath.pose3d.SE3'> object as transform of each link given joint state
def get_link_transform(robot,q):
    T = [] # Tranforms array of link
    for i in range(len(q)):
        T.append(robot.fkine(q,end = robot.links[i].name)) 
    return T

# check whether the robot touches the ground
def is_touch_ground(robot,q):
    T = get_link_transform(robot,q)
    height = [T[i].A[2,3] for i in range(2,6)]
    if min(height) > T[1].A[2,3] + 0.1:
        return False
    else: 
        return True

# Move robot with option of object moving along with
# rev_pose is the relative pose between the end-effector and the obj
def move_robot(robot, path, obj = None, relative_pose = None):
    # print('Unsafe movement!')
    for q in path:
            robot.q = q
            env.add(collisionObj.Sphere(radius=0.005, pose = robot.fkine(robot.q),color = (0.5,0.1,0.1,1)))
            if obj and relative_pose:
                pick = SE3(obj.T)
                pick = SE3(robot.fkine(q).A)* relative_pose.inv() 
                obj.T = pick.A
            env.step(0.05)

robo_clone = rtb.models.UR3() # make a clone for self collision check
def is_self_collision_UR3(q):
    global robo_clone
    robo_clone.q = q
    for i in reversed(range(8)[1:8]):
        stop_check = False
        if i == 2 :break
        for j in reversed(range(i - 1)[1:i-1]):
            is_collide = robo_clone.links[i].iscollided(shape = robo_clone.links[j].collision[0])
            if is_collide: 
                stop_check = True
                print("--COLLISION DETECT:",robo_clone.links[i].name," will collide with ",robo_clone.links[j].name)
                break    
        if stop_check: break
    return stop_check

def solve_for_valid_ik(robot,obj_pose,relative_pose):
    valid_solution = False 
    it_solve_possible = 0 # iterator for solution-exist cases
    it_solve_not_possible = 0 # iterator for non-solution-exist cases

    while not valid_solution:
        # random guess for ik_solver
        joint_guess_random = np.array([random.randint(-180,180)*pi/180 for _ in range(robot.n)])

        # Numerical inverse kinematics by default Levenberg-Marquadt optimization
        pick_joint_config = robot.ikine_LM(Tep = obj_pose*relative_pose,q0 = joint_guess_random, joint_limits= True)       

        if pick_joint_config.success:
            if it_solve_possible > 50 : break 
            it_solve_possible += 1 
            
            # if a solution exist, check whether it leads to self-collision or ground touching
            ground_touch = is_touch_ground(robot,pick_joint_config.q)
            self_collision = is_self_collision_UR3(pick_joint_config.q)
            if ground_touch or self_collision:
                continue
            else:
                print("Valid solution found!")
                valid_solution = True
        else:
            if it_solve_not_possible > 50 : break 
            it_solve_not_possible += 1
    
    if valid_solution: return pick_joint_config.q
    else:
        print('Cannot find valid solution!')
        return []

########## RUN ###################
## environment, robot and object
env = swift.Swift()
env.launch(realtime= "True")

robot = rtb.models.UR3()
robot.q[1] = -pi/2
home_joint = robot.q 
env.add(robot)

ground = collisionObj.Cuboid(scale = [20,20,0.1], pose = SE3(0,0,-0.03), color = (0.13,0.12,0.1,1))
env.add(ground)

obj = collisionObj.Cuboid(scale = [0.2,0.1,0.1], pose = SE3(0.45,0,0.05),color = (0.1,0.5,0.1,1))
env.add(obj)

# PICK & PLACE
## object current pose and relative pose to robot ee
pick_pose = SE3(obj.T)
relative_pose = SE3.Tz(0.05) * SE3.Ry(pi/2) # relative pose of the ee with respect to the obj

# solve for picking joint config
pick_joint = solve_for_valid_ik(robot=robot,obj_pose=SE3(obj.T),relative_pose = relative_pose)

## object place pose
place_pose = SE3(-0.3,-0.3,0.05)

# solve for placing joint config
place_joint = solve_for_valid_ik(robot=robot,obj_pose=place_pose,relative_pose = relative_pose)

## do the task
if pick_joint.all() and place_joint.all():
    # go to pick position
    pick_path = rtb.mtraj(rtb.trapezoidal,robot.q,pick_joint,50)
    move_robot(robot=robot, path = pick_path.q)

    # move to place position with the object
    place_path = rtb.mstraj(viapoints=np.array([pick_joint,home_joint,place_joint]),dt=0.02,tacc=0.2,qdmax = np.pi)
    move_robot(robot = robot, path = place_path.q, obj = obj, relative_pose= relative_pose)

    # return to home:
    home_path = rtb.mtraj(rtb.trapezoidal,robot.q,home_joint,50)
    move_robot(robot=robot, path = home_path.q)

    print("Task done!")
else:
    print("Can't do the task!")

env.hold()

