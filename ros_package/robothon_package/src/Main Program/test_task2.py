from function import rtb, collisionObj,SE3, pi, swift, np, cam_move, move_simulation_robot
from spatialmath.base import *
from ur3module import *
# import time
from function import *


def cam_to_global(pixel_x, pixel_y, camera_transform):
    # Define the camera parameters
    pixel_width = 640  # Width of the image in pixels
    pixel_height = 480  # Height of the image in pixels
    focal_length = 848  # Focal length of the camera in pixels
    
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

# Enviroment
env = swift.Swift()
env.launch(realtime=True)
dt = 0.05
env._send_socket
# Clone
robot = rtb.models.UR3()

# Box 
box_path = "/home/quangngo/Robotics Studio 2/GroupGit/robothon2023/RTB-P Test Files/SomeApplications/BOX_FINAL.STL"
box = collisionObj.Mesh(filename= box_path,pose = SE3(0,0,0),scale=[0.001, 0.001, 0.001], color = [0.4,0.4,0.4,0.5])
box.T = trotx(pi/2) @ troty(pi)
box.T = transl(0.207,-0.0084,-0.140) @ box.T  

# Gripper
gripper_path = "/home/quangngo/Robotics Studio 2/GroupGit/robothon2023/RTB-P Test Files/SomeApplications/CAMGRIPPER.STL"
gripper = collisionObj.Mesh(filename=gripper_path,pose = SE3(0,0,0),scale=[0.001, 0.001, 0.001],color = [0.5,0.1,0.1,1])
TGR = SE3.Rx(pi)*SE3(0,-0.105,-0.175) # gripper pose in ee frame

# Camera
cam = collisionObj.Cuboid(scale=[0.03,0.06,0.015], pose = SE3(0,0,0), color = [0.5,0.5,0.1,1])
TCR = SE3(0.085,0,0.09)*SE3.Ry(pi/2) # cam pose in ee frame

# Coin
coin = collisionObj.Cylinder(radius = 0.007, length= 0.002, pose = SE3(0, 0, 0), color = [0.3,0.3,0.1,1])
TCC = SE3(0.23,0,0) * SE3.Rx(pi/2) #coin pose in ee frame
coin.T = np.array([[-0.01042, -0.9999, 0.01244, 0.2231],
                   [0.003293, 0.0124, 0.9999, 0.2865],
                   [-0.9999, 0.01046, 0.003163, 0.34],
                   [0, 0, 0, 1]]) @ TCC.A

# Pin p1, p2 task 2
pin1 = collisionObj.Cylinder(radius = 0.007, length= 0.048, pose = SE3(0.13796 - 0.048/2,0.3229,0.1535-0.008)*SE3.Ry(pi/2), color = (0.2,0.2,0.5,1))
pin2 = collisionObj.Cylinder(radius = 0.007, length= 0.048, pose = SE3(0.13796 - 0.048/2,0.3409,0.1535-0.008)*SE3.Ry(pi/2), color = (0.2,0.2,0.5,1))
TCP = SE3(0.23,0,0)*SE3.Ry(pi/2) # pin pose in ee frame

# Set initial position
joint_home_degree = [57.11, -106.32, 56.84, -44.7, -85.81, 328.40]
joint_home_radian = [math.radians(joint_home_degree) for joint_home_degree in joint_home_degree]
robot.q = joint_home_radian

# Add to environment
env.add(robot)
env.add(box)
# env.add(cam)
env.add(gripper)
env.add(coin)
# env.add(pin1)
# env.add(pin2)

# Test hole:
hole = [660,825]
cam_move(cam, robot, TCR)
camera_transform = cam.T
global_pin2 = cam_to_global(660,825, camera_transform)
pin_test = collisionObj.Cylinder(radius = 0.005, length= 0.06, pose = SE3(global_pin2[0], global_pin2[1], global_pin2[2]-0.05), color = [0.2,0.5,0.2,1])
env.add(pin_test)
# q to pick coin
q_coin_position = [33.95, -75.09, 53.02, -67.54, -89.51, 304.66]
q_coin_position = [math.radians(joint_home_degree) for joint_home_degree in q_coin_position]

# Create a trajectory to the coin_position:
path_to_coin_position = rtb.jtraj(joint_home_radian, q_coin_position, 40)

# Move Robot Down:
path_move_down2_pick_coin = move_up_down(robot, path_to_coin_position.q[-1], 'down', lift=0.045, q_guess_2 = list(path_to_coin_position.q[-1]))

# Combine two path:
path_2_pick_coin = combine_trajectories([path_to_coin_position, path_move_down2_pick_coin])

move_simulation_robot(robot = robot, path = path_2_pick_coin, env= env, dt = dt, gripper = gripper, cam = cam, TCR = TCR, TGR = TGR)
arrived = True
#send_action_client(arrived, path_2_pick_coin, goal, start_time, client)
print(robot.fkine(path_move_down2_pick_coin.q[-1]))


### 2. Lift coin up
lift_coin_up = list(reversed(path_move_down2_pick_coin.q))
move_simulation_robot(robot = robot, path = lift_coin_up, env= env, dt = dt, gripper = gripper, cam = cam, pin = coin, TCR = TCR, TGR = TGR, TCP = TCC)
arrived = True

################################# 3. FLIPPING THE BATTERIES START FROM HERE: ########################

#### 3.1 MOVE TO ROBOT TO THE POSITION BEFORE FLIPPING THE BATTERY 1:

            # TEST ADDING BATTERY INFO AND DROPPING INFO
is_battery_there = '2' # '1': only battery 1,'2': only battery 2, '12': both batteries 

if is_battery_there == '1':
    env.add(pin1)
if is_battery_there == '2':
    env.add(pin2)
if is_battery_there == '12':
    env.add(pin1)
    env.add(pin2)


flick_instruction, path_instruction = get_task2_param(robot, joint_home_radian, lift_coin_up[-1], TCC, is_battery_there)

count = 0
for instruction in flick_instruction:
    # Move coin to the battery
    move_simulation_robot(robot = robot, path = instruction['reach'], env= env, dt = 0.05, 
                          gripper = gripper, cam = cam, pin = coin, TCR = TCR, TGR = TGR, TCP = TCC)
    arrived = True
    #send_action_client(arrived, instruction['reach'], goal, start_time, client, speed=1)

    move_simulation_robot(robot = robot, path= instruction['flick'], env= env, dt = 0.05, 
                          gripper = gripper, cam = cam, pin = coin, TCR = TCR, TGR = TGR, TCP = TCC)
    
    pin1_cen = [pin1.T[0,3] - 0.048/2, pin1.T[1,3], pin1.T[2,3]]
    pin2_cen = [pin2.T[0,3] + 0.048/2, pin2.T[1,3], pin2.T[2,3]]
    
    if is_battery_there == '1':
        pin1.T = transl(pin1_cen) @ troty(-10*pi/180) @ linalg.inv(transl(pin1_cen)) @ pin1.T 
    
    if is_battery_there == '2':
        pin2.T = transl(pin2_cen) @ troty(10*pi/180) @ linalg.inv(transl(pin2_cen)) @ pin2.T

    if is_battery_there == '12':
        if count == 0: 
            pin1.T = transl(pin1_cen) @ troty(-15*pi/180) @ linalg.inv(transl(pin1_cen)) @ pin1.T
        if count == 1:
            pin2.T = transl(pin2_cen) @ troty(15*pi/180) @ linalg.inv(transl(pin2_cen)) @ pin2.T

    arrived = True
    #send_action_client(arrived, instruction['flick'], goal, start_time, client, speed=6)
    count += 1

#-> Move to the position to drop the coin
q_drop = [99.23, -75.10, 45.56, -66.99, -85.81, 351.35]
q_drop_radian = [math.radians(joint_home_degree) for joint_home_degree in q_drop]
path_drop = rtb.jtraj(flick_instruction[-1]['flick'][-1], q_drop_radian, 30)
move_simulation_robot(robot = robot, path= path_drop.q, env= env, dt = 0.05, 
                      gripper = gripper, cam = cam, pin = coin, TCR = TCR, TGR = TGR, TCP = TCC)
arrived = True


########################## 5. BEGIN TO PLACE THE BATTERY IN A CORRECT ORDER ###########################
count = 0
for instruction in path_instruction:
     
    # Move to picking battery
    move_simulation_robot(robot = robot, path= instruction['picking'], env= env, dt = 0.05, 
                          gripper = gripper, cam = cam, TCR = TCR, TGR = TGR)
    arrived = True

    if count == 0 and is_battery_there == '1':
        battery = pin1
    if count == 0 and is_battery_there == '2':
        battery = pin2
    if is_battery_there == '12':
        if count == 0: 
            battery = pin1
        if count == 1:
            battery = pin2

    # Move the battery out
    move_simulation_robot(robot = robot, path= instruction['taking'], env= env, dt = 0.02, 
                          gripper = gripper, cam = cam, pin = battery, TCR = TCR, TGR = TGR, TCP = TCP)
    arrived = True

    # Move to drop battery
    move_simulation_robot(robot = robot, path= instruction['dropping'], env= env, dt = 0.02, 
                          gripper = gripper, cam = cam, pin = battery, TCR = TCR, TGR = TGR, TCP = TCP)
    arrived = True

    count += 1

env.hold()