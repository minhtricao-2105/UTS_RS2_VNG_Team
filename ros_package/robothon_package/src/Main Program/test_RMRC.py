from function import*
from time import sleep

# Clone
robot = rtb.models.UR3() 
q_deg = [57.11, -106.32, 56.84, -44.7, -85.81, 328.40] # First position of the robot:
q_start = [x*pi/180 for x in q_deg]
robot.q = q_start

# Camera
cam = collisionObj.Cuboid(scale=[0.03,0.06,0.015], pose = SE3(0,0,0), color = [0.5,0.5,0.1,1])
TCR = SE3(0.085,0,0.09)*SE3.Ry(pi/2) #relative pose with ee
cam_move(cam, robot, TCR)
camera_transform = cam.T #camera transform at capturing position

TGR = SE3.Rx(pi)*SE3(0,-0.105,-0.175)
gripper_path = "/home/quangngo/Desktop/RTB-P Test files/SomeApplications/CAMGRIPPER.STL"
gripper = collisionObj.Mesh(filename=gripper_path,pose = SE3(0,0,0),scale=[0.001, 0.001, 0.001],color = [0.5,0.1,0.1,1])
cam_move(gripper, robot, TGR)

# Enviroment
env = swift.Swift()
env.launch(realtime=True)
dt = 0.05
env.add(robot)
env.add(cam)
env.add(gripper)

# Define the point in Pixel frame
pixel_x_1 = 600
pixel_y_1 = 430
global_position_1 = cam_to_global(pixel_x_1,pixel_y_1, camera_transform) #global position
goal_obj_1 = collisionObj.Sphere(radius = 0.008, pose = SE3(global_position_1[0], global_position_1[1], global_position_1[2]),color = (0.5,0.1,0.1,1))
env.add(goal_obj_1)
print(global_position_1)

# Move to gripping point
path = move_to_pin(robot, q_start, global_position_1, turn = False)
print(path.q)

pose_list = [robot.fkine(q) for q in path.q]
# pose_list = []
# for i in range(len(path.q)):
#     pose = robot.fkine(path.q[i])
#     pose_list.append(pose)

for pose in pose_list:
    p = collisionObj.Sphere(radius = 0.005, pose = pose * SE3(0.2,0,0),color = (0.5,0.1,0.1,1))
    # print(pose.A[0:3,3])
    env.add(p)

i = 0


for pose in pose_list:
    print("Step:", i)
    arrived = False
    while np.linalg.norm(robot.fkine(robot.q).A[0:3,3] - pose.A[0:3,3]) > 0.01:
    # print("HERE")
    # while not arrived:
        v, arrived = rtb.p_servo(robot.fkine(robot.q), pose, 1, threshold=0.01)
        vj = np.linalg.pinv(robot.jacobe(robot.q)) @ v # This is sending the velocity
        vj = vj.tolist()
        joint_step = [dt*x for x in vj]
        robot.q = [x+y for x,y in zip(robot.q, joint_step)]
        print("HERE")
        cam_move(cam, robot, TCR)
        cam_move(gripper, robot, TGR)
        env.add(collisionObj.Sphere(radius=0.005, pose = robot.fkine(robot.q) * SE3(0.2, 0,0),color = (0.1,0.5,0.1,1)))
        env.step(dt)
    i+=1

print("FINAL ERROR: ", np.linalg.norm(robot.fkine(robot.q).A[0:3,3] - pose_list[-1].A[0:3,3]))

env.hold()