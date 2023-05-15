import sys
sys.path.append('../Modules')
from ur3module import *
import pathgenerator

def cam_move(cam,robot,T):
    cam.T = robot.fkine(robot.q)*T

def cam_to_global(pixel_x, pixel_y):
    # Compute the normalized coordinates of the point
    normalized_x = (pixel_x - pixel_width / 2) / focal_length
    normalized_y = (pixel_height / 2 - pixel_y) / focal_length

    # Compute the distance from the camera to the global point
    distance = 0.18  # Distance from the camera to the global point in meters

    # Compute the position of the point in the camera frame
    camera_position = np.array([distance * normalized_x, distance * normalized_y, distance, 1])

    # Map the point from the camera frame to the global frame
    global_position = np.matmul(camera_transform, camera_position)

    return global_position

env = swift.Swift()
env.launch(realtime = True)
dt = 0.05

q_deg = [57.11, -106.32, 56.84, -44.7, -85.81, -(360-328.40)]
robot = rtb.models.UR3()
robot.q = [x*pi/180 for x in q_deg]

gripper_path = "/home/quangngo/Desktop/RTB-P Test files/SomeApplications/CAMGRIPPER.STL"
gripper = collisionObj.Mesh(filename=gripper_path,pose = SE3(0,0,0),scale=[0.001, 0.001, 0.001],color = [0.5,0.1,0.1,1])

cam = collisionObj.Cuboid(scale=[0.03,0.06,0.015], pose = SE3(0,0,0), color = [0.5,0.5,0.1,1])

#gripper in robot ee frame
TGR = SE3.Rx(pi)*SE3(0,-0.105,-0.175)

#cam in robot ee frame
TCR = SE3(0.085,0,0.09)*SE3.Ry(pi/2)

cam_move(gripper,robot, TGR)
cam_move(cam, robot, TCR)

env.add(robot)
env.add(gripper)
env.add(cam)    

# Define the camera parameters
pixel_width = 640  # Width of the image in pixels
pixel_height = 480  # Height of the image in pixels
focal_length = 848  # Focal length of the camera in pixels

camera_transform = cam.T

# Define the point in Pixel frame
pixel_x_1 = 270 #265  # X-coordinate of the point in pixels
pixel_y_1 = 137 #135  # Y-coordinate of the point in pixels

global_position_1 = cam_to_global(pixel_x_1,pixel_y_1)

# Print the result
print("Pixel frame coordinates: ({}, {})".format(pixel_x_1, pixel_y_1))
print("Global frame coordinates: ({}, {}, {})".format(global_position_1[0], global_position_1[1], global_position_1[2]))

# Add the goal to environment
goal_obj = collisionObj.Sphere(radius = 0.02, pose = SE3(global_position_1[0], global_position_1[1], global_position_1[2]),color = (0.5,0.1,0.1,1))
env.add(goal_obj)

# Solve IK for end-effector:
# print([x*180/pi for x in robot.q])
q_sample = [63.33, -105.04, 89.33, -75.14, -87.53, 335.72-360]
q_sample = [x*pi/180 for x in q_sample]
print("Sample:" ,q_sample)
ee_orientation = SE3.Rt(robot.fkine(q_sample).R)

offset_z = 0.17
obj_pose=SE3(global_position_1[0], global_position_1[1], global_position_1[2]+ offset_z)*ee_orientation

q_guess = [1.3279389,  -1.41725404,  0.17017859, -0.62366733, -1.53202614, -0.20099896] 
q_goal = solve_for_valid_ik(robot=robot, obj_pose=obj_pose, q_guess = q_guess, elbow_up_request = True, shoulder_left_request=False)
# q_goal[-1] = q_goal[-1] + 2*pi
# print("Goal:",q_goal)
path = rtb.jtraj(robot.q, q_goal,50)

for q in path.q:
    robot.q = q
    cam_move(gripper,robot, TGR)
    cam_move(cam,robot,TCR)
    env.step(dt)

lift = 0.08
q_guess_2 = [ 1.32582823, -1.48473735,  1.1266249,  -1.23659264, -1.53197561,  6.08007491]
q_lifts = []
step_num = 4
step_lift = lift/(step_num-1)

#grip
q_curr = robot.q
#generate 4 poses for lifting up
for i in range(step_num):
    if i == 0 : 
        q_lifts.append(q_curr)
        continue
    pose_lift = robot.fkine(q_curr)*SE3(-(0+step_lift*i),0,0)
    q_lift = solve_for_valid_ik(robot=robot, obj_pose=pose_lift, q_guess = q_guess_2, elbow_up_request = False, shoulder_left_request= False)
    # q_lift[-1] = q_lift[-1] + pi
    q_lifts.append(q_lift)

path_lift = rtb.mstraj(viapoints=np.array([q for q in q_lifts]),dt=0.01,tacc=0.05,qdmax = np.pi)
pathgenerator.show_path(robot,path_lift.q,env)
print(len(path_lift))

for q in path_lift.q:
    robot.q = q
    cam_move(gripper,robot, TGR)
    cam_move(cam,robot,TCR)
    env.step(dt)


env.hold()



