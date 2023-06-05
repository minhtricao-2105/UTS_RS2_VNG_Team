from function import*

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
gripper_path = "/home/quangngo/Desktop/RTB-P Test files/SomeApplications/CAMGRIPPER.STL"
gripper = collisionObj.Mesh(filename=gripper_path,pose = SE3(0,0,0),scale=[0.001, 0.001, 0.001],color = [0.5,0.1,0.1,1])

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
# goal_obj_1 = collisionObj.Sphere(radius = 0.008, pose = SE3(global_position_1[0], global_position_1[1], global_position_1[2]),color = (0.5,0.1,0.1,1))
pin = collisionObj.Cylinder(radius = 0.005, length= 0.06, pose = SE3(global_position_1[0], global_position_1[1], global_position_1[2]-0.05),color = (0.2,0.5,0.1,1))
TCP = SE3(0.23,0,0)*SE3.Ry(pi/2) #relative pose pin to robot
env.add(pin)

# Move to gripping point
path = move_to_pin(robot, q_start, global_position_1, turn = False)
for q in path.q:
    robot.q = q
    cam_move(cam,robot,TCR)
    cam_move(gripper,robot,TGR)
    env.step(dt)

# Lifting up
robot.q = path.q[-1]
path_lift = move_up_down(robot, path.q[-1])
for q in path_lift.q:
    robot.q = q
    cam_move(cam,robot,TCR)
    cam_move(gripper,robot,TGR)
    cam_move(pin,robot,TCP)
    env.step(dt)
print("DONE LIFT!")

# Define new points in Pixel frame
pixel_x_2 = 600
pixel_y_2 = 360
global_position_2 = cam_to_global(pixel_x_2,pixel_y_2, camera_transform) #global position
goal_obj_2 = collisionObj.Sphere(radius = 0.002, pose = SE3(global_position_2[0], global_position_2[1], global_position_2[2]),color = (0.5,0.1,0.1,1))
env.add(goal_obj_2)

# Move to dropping point
robot.q = path_lift.q[-1]
path_2 = move_to_pin(robot, path_lift.q[-1], global_position_2, offset_z = 0.225,turn= True)
for q in path_2.q:
    robot.q = q
    cam_move(cam,robot,TCR)
    cam_move(gripper,robot,TGR)
    cam_move(pin,robot,TCP)
    env.step(dt)

# Moving down
robot.q = path_2.q[-1]
path_lift_2 = move_up_down(robot, path_2.q[-1],dir = 'down')
for q in path_lift_2.q:
    robot.q = q
    cam_move(cam,robot,TCR)
    cam_move(gripper,robot,TGR)
    cam_move(pin,robot,TCP)
    env.step(dt)

env.hold()