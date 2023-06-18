from function import rtb, collisionObj, SE3, cam_move, pi, swift

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

# Gripper
TGR = SE3.Rx(pi)*SE3(0,-0.105,-0.175)
# gripper_path = "/home/minhtricao/robothon2023/RTB-P Test Files/SomeApplications/CAMGRIPPER.STL"
# gripper_path = "/home/quangngo/Robotics Studio 2/GroupGit/robothon2023/RTB-P Test Files/SomeApplications/CAMGRIPPER.STL"
# gripper_path = "D:/UTS_RS2_VNG_Team/RTB-P Test Files/SomeApplications/CAMGRIPPER.STL"
# gripper = collisionObj.Mesh(filename=gripper_path,pose = SE3(0,0,0),scale=[0.001, 0.001, 0.001],color = [0.5,0.1,0.1,1])

# Enviroment
env = swift.Swift()
env.launch(realtime=True)
dt = 0.05
env.add(robot)
# env.add(cam)
# env.add(gripper)

env.hold()