import sys
sys.path.append('../Modules')
from ur3module import *

# create environment and robot
env = swift.Swift()
env.launch(realtime=True)

robot = rtb.models.UR3()
robot.q[1] = -pi/2
home_joint = robot.q
env.add(robot)

# create a goal object 
goal_obj = collisionObj.Sphere(radius = 0.02, pose = SE3(0.35,0.35,0.05),color = (0.1,0.5,0.1,1))
env.add(goal_obj)

# solve for inversed kinematic to reach the goal
q_goal = solve_for_valid_ik(robot=robot,obj_pose=SE3(goal_obj.T),relative_pose = SE3.Rz(0.05)*SE3.Ry(pi/2), elbow_up_request = True)

# obstacles:
obstacle_list = []
obstacle_list.append(collisionObj.Sphere(radius = 0.07, pose = SE3(0.2,0.1,0.4),color = (0.1,0.1,0.5,1)))
# obstacle_list.append(collisionObj.Sphere(radius = 0.07, pose = SE3(0.2,0.43,0.4),color = (0.1,0.1,0.5,1)))
for obstacle in obstacle_list:
   env.add(obstacle)

# generate path to goal
if not np.array_equal(q_goal,robot.q):
   path = rtb.mtraj(tfunc = rtb.trapezoidal, q0 = robot.q, qf = q_goal, t = 50)
   path_valid,all_valid = get_valid_path(robot, path, obstacle_list)
   show_path(robot,path_valid,env)

for q in path_valid:
   robot.q = q
   env.step(0.05)

# move_robot_insurance(robot,q_goal,env,obstacle_list,show_path=False)
env.hold()
