# import sys
# sys.path.append('/home/quangngo/Desktop/RTB-P Test files/Modules/ur3module.py')
# from ur3module import rtb, solve_for_valid_ik, collisionObj, move_robot_with_object,swift, np
from ur3module import *
from spatialmath import SE3
from math import pi

env = swift.Swift()
env.launch(realtime=True)

robot = rtb.models.UR3()
robot.q[1] = -pi/2
home_joint = robot.q

obj = collisionObj.Cuboid(scale = [0.2,0.1,0.1], pose = SE3(0.45,0,0.05),color = (0.1,0.5,0.1,1))
env.add(obj)
env.add(robot)

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
    move_robot_with_object(robot=robot, path = pick_path.q,env = env)

    # move to place position with the object
    place_path = rtb.mstraj(viapoints=np.array([pick_joint,home_joint,place_joint]),dt=0.02,tacc=0.2,qdmax = np.pi)
    move_robot_with_object(robot = robot, path = place_path.q, obj = obj, relative_pose= relative_pose, env = env)

    # return to home:
    home_path = rtb.mtraj(rtb.trapezoidal,robot.q,home_joint,50)
    move_robot_with_object(robot=robot, path = home_path.q, env = env)

    print("Task done!")
else:
    print("Can't do the task!")

# env.hold()

print(robot._grippers)


