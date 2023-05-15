# from ur3module import *
from function import*

# Clone
robot = rtb.models.UR3() 
q_deg = [57.11, -106.32, 56.84, -44.7, -85.81, 328.40] # First position of the robot:
q_start = [x*pi/180 for x in q_deg]
robot.q = q_start


# Camera
cam = collisionObj.Cuboid(scale=[0.03,0.06,0.015], pose = SE3(0,0,0), color = [0.5,0.5,0.1,1])
TCR = SE3(0.085,0,0.09) #relative pose with ee
cam_move(cam, robot, TCR*SE3.Ry(pi/2))
camera_transform = cam.T #camera transform at capturing position

# Enviroment
env = swift.Swift()
env.launch(realtime=True)
dt = 0.05
env.add(robot)
# env.add(cam)

# Define the point in Pixel frame
pixel_x_1 = 600
pixel_y_1 = 430
global_position_1 = cam_to_global(pixel_x_1,pixel_y_1, camera_transform) #global position
goal_obj_1 = collisionObj.Sphere(radius = 0.02, pose = SE3(global_position_1[0], global_position_1[1], global_position_1[2]),color = (0.5,0.1,0.1,1))
env.add(goal_obj_1)

# Move to gripping point
path = move_to_pin(robot, q_start, global_position_1)
for q in path.q:
    robot.q = q
    cam_move(cam,robot,TCR)
    env.step(dt)

# Lifting up
robot.q = path.q[-1]
path_lift = move_up_down(robot, path.q[-1])
for q in path_lift.q:
    robot.q = q
    cam_move(cam,robot,TCR)
    env.step(dt)
print("DONE LIFT!")

# Define new points in Pixel frame
pixel_x_2 = 500
pixel_y_2 = 430
global_position_2 = cam_to_global(pixel_x_2,pixel_y_2, camera_transform) #global position
goal_obj_2 = collisionObj.Sphere(radius = 0.02, pose = SE3(global_position_2[0], global_position_2[1], global_position_2[2]),color = (0.1,0.5,0.1,1))
env.add(goal_obj_2)

# Move to dropping point
robot.q = path_lift.q[-1]
path_2 = move_to_pin(robot, path_lift.q[-1], global_position_2, offset_z = 0.225)
for q in path_2.q:
    robot.q = q
    cam_move(cam,robot,TCR)
    env.step(dt)

# Moving down
robot.q = path_2.q[-1]
path_lift_2 = move_up_down(robot, path_2.q[-1],dir = 'down')
for q in path_lift_2.q:
    robot.q = q
    cam_move(cam,robot,TCR)
    env.step(dt)

env.hold()