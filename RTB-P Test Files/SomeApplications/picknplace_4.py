import sys
sys.path.append('../Modules')
from ur3module import rtb,collisionObj,swift,SE3,solve_for_valid_ik, move_robot_insurance, np, pi
# from ur3module import *
# from spatialmath import SE3
# from math import pi
env = swift.Swift()
env.launch(realtime=True)

robot = rtb.models.UR3()
robot.q[1] = -pi/2
home_joint = robot.q

obj = collisionObj.Cuboid(scale = [0.2,0.1,0.1], pose = SE3(0.45,0,0.05),color = (0.1,0.5,0.1,1))
env.add(obj)
env.add(robot)

# PICK & PLACE
# PICK
## object current pose and relative pose to robot ee3
obj_pose = SE3(obj.T) # object pose
relative_pose = SE3.Tz(0.05) * SE3.Ry(pi/2) # relative pose of the ee with respect to the obj when it touch the object
relative_pose_up_air = SE3.Tz(0.1) * relative_pose

# solve for picking joint config above object
pick_joint_up_air = solve_for_valid_ik(robot=robot,obj_pose=obj_pose,relative_pose = relative_pose_up_air, elbow_up_request = True)

# move to that joint
if pick_joint_up_air.all():
    move_robot_insurance(robot = robot, q_end = pick_joint_up_air,env =  env, show_path = False)

# RMRC to vertically reach the object
pick_pose = SE3.Tz(-0.2)*robot.fkine(robot.q)
arrived = False
count_step = 0
while not arrived:
    v, arrived = rtb.p_servo(robot.fkine(robot.q), pick_pose, 0.5)
    robot.qd = np.linalg.pinv(robot.jacobe(robot.q)) @ v
    # env.add(collisionObj.Sphere(radius=0.005, pose = robot.fkine(robot.q),color = (0.1,0.1,0.5,1)))
    env.step(0.05)
    count_step+=1
    print(count_step)
    if arrived: print("DONE!")


# ## object place pose
# place_pose = SE3(-0.3,-0.3,0.05)

# # solve for placing joint config
# place_joint = solve_for_valid_ik(robot=robot,obj_pose=place_pose,relative_pose = relative_pose)

env.hold()