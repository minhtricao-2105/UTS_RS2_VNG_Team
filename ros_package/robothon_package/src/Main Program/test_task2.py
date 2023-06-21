from function import rtb, collisionObj,SE3, pi, swift, np, cam_move, move_simulation_robot
from spatialmath.base import *
from ur3module import *
# import time
from function import *

# Enviroment
env = swift.Swift()
env.launch(realtime=True)
dt = 0.05
env._send_socket
# Clone
robot = rtb.models.UR3()

# Gripper
gripper_path = "/home/quangngo/Robotics Studio 2/GroupGit/robothon2023/RTB-P Test Files/SomeApplications/CAMGRIPPER.STL"
gripper = collisionObj.Mesh(filename=gripper_path,pose = SE3(0,0,0),scale=[0.001, 0.001, 0.001],color = [0.5,0.1,0.1,1])
TGR = SE3.Rx(pi)*SE3(0,-0.105,-0.175) # gripper pose in ee frame

# Camera
cam = collisionObj.Cuboid(scale=[0.03,0.06,0.015], pose = SE3(0,0,0), color = [0.5,0.5,0.1,1])
TCR = SE3(0.085,0,0.09)*SE3.Ry(pi/2) # cam pose in ee frame

# Coin
coin = collisionObj.Cylinder(radius = 0.01, length= 0.005, pose = SE3(0, 0, 0), color = [0.3,0.3,0.1,1])
TCC = SE3(0.23,0,0) * SE3.Rx(pi/2) #coin pose in ee frame
coin.T = np.array([[-0.01042, -0.9999, 0.01244, 0.2231],
                   [0.003293, 0.0124, 0.9999, 0.2865],
                   [-0.9999, 0.01046, 0.003163, 0.34],
                   [0, 0, 0, 1]]) @ TCC.A

# Set initial position
joint_home_degree = [57.11, -106.32, 56.84, -44.7, -85.81, 328.40]
joint_home_radian = [math.radians(joint_home_degree) for joint_home_degree in joint_home_degree]
robot.q = joint_home_radian

# Add to environment
env.add(robot)
env.add(cam)
env.add(gripper)
env.add(coin)

# while not rospy.is_shutdown():
#     pub.publish(moveGripperToPosition(400,400))
#     rospy.sleep(0.25)
#     break

q_coin_position = [33.95, -75.09, 53.02, -67.54, -89.51, 304.66]
q_coin_position = [math.radians(joint_home_degree) for joint_home_degree in q_coin_position]

# Create a trajectory to the coin_position:
path_to_coin_position = rtb.jtraj(joint_home_radian, q_coin_position, 40)

# Move Robot Down:
path_move_down2_pick_coin = move_up_down(robot, path_to_coin_position.q[-1], 'down', lift=0.045, q_guess_2 = list(path_to_coin_position.q[-1]))

# Combine two path:
# path_2_pick_coin = combine_trajectory(path_to_coin_position, path_move_down2_pick_coin)
path_2_pick_coin = combine_trajectories([path_to_coin_position, path_move_down2_pick_coin])

move_simulation_robot(robot = robot, path = path_2_pick_coin, env= env, dt = dt, gripper = gripper, cam = cam, TCR = TCR, TGR = TGR)
arrived = True
#send_action_client(arrived, path_2_pick_coin, goal, start_time, client)
print(robot.fkine(path_move_down2_pick_coin.q[-1]))

# ## Close gripper
# while not rospy.is_shutdown():
#     pub.publish(closeGripper(400))
#     rospy.sleep(0.25)
#     break

### 2. Lift coin up
lift_coin_up = list(reversed(path_move_down2_pick_coin.q))
move_simulation_robot(robot = robot, path = lift_coin_up, env= env, dt = dt, gripper = gripper, cam = cam, pin = coin, TCR = TCR, TGR = TGR, TCP = TCC)
arrived = True
#send_action_client(arrived, lift_coin_up, goal, start_time, client)

################################# 3. FLIPPING THE BATTERIES START FROM HERE: ########################

#### 3.1 MOVE TO ROBOT TO THE POSITION BEFORE FLIPPING THE BATTERY 1:

            # TEST ADDING BATTERY INFO AND DROPPING INFO
is_battery_there = '12' # '1': only battery 1,'2': only battery 2, '12': both batteries 

# Data belong to battery 1:
battery_1_position = [45.52, -68.18, 27.30, -47.27, -87.16, 315.94]
battery_1_position = [math.radians(joint_home_degree) for joint_home_degree in battery_1_position]

# Data belong to battery 2:
battery_2_position = [56.25, -73.65, 35.65, -53.94, -89.34, 324.05]
battery_2_position = [math.radians(joint_home_degree) for joint_home_degree in battery_2_position]

# Dropping data
#-> For coin
q_drop = [99.23, -75.10, 45.56, -66.99, -85.81, 351.35]
q_drop_radian = [math.radians(joint_home_degree) for joint_home_degree in q_drop]
path_drop = rtb.jtraj(lift_coin_up[-1], q_drop_radian, 30)

#-> For batteries
q_droppin_1 = [78.65, -96.40, 64.17, -58.12, -88.22, 252.57]
q_droppin_1 = [math.radians(joint_home_degree) for joint_home_degree in q_droppin_1]
q_droppin_2 = [70.67, -97.03, 61.32, -54.39, -88.19, 242.99]
q_droppin_2 = [math.radians(joint_home_degree) for joint_home_degree in q_droppin_2]    

# ROUTE FOR BATTERY 1:
#-> From coin position to picking position
pick1_way1 = [36.05, -63.35, 24.29, -48.96, -86.55, 217.81]
pick1_way1 = [math.radians(joint_home_degree) for joint_home_degree in pick1_way1]
pick1_way2 = [26.63, -49.31, 19.13, -27.04, -68.63, 203.57]
pick1_way2 = [math.radians(joint_home_degree) for joint_home_degree in pick1_way2]

pick1_path1 = rtb.mstraj(viapoints=np.array([path_drop.q[-1], pick1_way1, pick1_way2]),dt=0.01,tacc=0.05,qdmax = pi)

#-> From picking position taking out
pick1_way3 = [56.74, -86.43, 46.07, -51.44, -88.32, 234.33]
pick1_way3 = [math.radians(joint_home_degree) for joint_home_degree in pick1_way3]

path_up_pickway1 = move_up_down(robot, pick1_path1.q[-1], lift = 0.005) 
path_to_pickway1 = rtb.jtraj(path_up_pickway1.q[-1], pick1_way3, 30)

pick1_path2 = combine_trajectories([path_up_pickway1, path_to_pickway1])

#-> To dropping position 1
path1_droppin1 = rtb.jtraj(pick1_path2[-1], q_droppin_1, 30)
path1_droppin2 = move_up_down(robot, path1_droppin1.q[-1], 'down', lift = 0.02)

path_droppin_bat1 = combine_trajectories([path1_droppin1, path1_droppin2])

#-> Combine route for battery 1
path1_instruction ={'picking': pick1_path1.q, 'taking': pick1_path2, 'dropping': path_droppin_bat1}

# ROUTE FOR BATTERY 2:
#-> From current position to picking position
pick2_way1 = [94.24, -79.7, 61.95, -74.67, -118.79, 84.73 + 360]
pick2_way1 = [math.radians(joint_home_degree) for joint_home_degree in pick2_way1]

pick2_way2 = [82.72, -69.40, 56.97, -72.54, -120.00, 82.96 + 360]
pick2_way2 = [math.radians(joint_home_degree) for joint_home_degree in pick2_way2]

q_pick2_start = path_droppin_bat1[-1] # default if is_batery_there == '12' or '1'
if is_battery_there == '2': # if only battery 2 is put in, taking from the coin drop position
    q_pick2_start = path_drop.q[-1]

pick2_path1 = rtb.mstraj(viapoints=np.array([q_pick2_start, joint_home_radian, pick2_way1, pick2_way2]),dt=0.01,tacc=0.05,qdmax = pi)

#-> From picking position taking out
drag_pose = SE3(-0.06, 0, 0.005) * robot.fkine(pick2_way2)
q_sol_1 = solve_for_valid_ik(robot= robot, obj_pose = drag_pose, q_guess = pick2_way2)
q_sol_1[-1] = pick2_way2[-1]

down_pose = SE3(0.008, 0, -0.008) * robot.fkine(q_sol_1)
q_sol_2 = solve_for_valid_ik(robot= robot, obj_pose = down_pose, q_guess = list(q_sol_1))
q_sol_2[-1] = q_sol_1[-1]

final_ee_pose = SE3(0.055, 0, 0.06)*robot.fkine(q_sol_2)
q_sol_3 = solve_for_valid_ik(robot= robot, obj_pose = final_ee_pose, q_guess = list(q_sol_2))
q_sol_3[-1] = q_sol_2[-1]

pick2_path2 = rtb.mstraj(viapoints=np.array([pick2_way2, q_sol_1, q_sol_2, q_sol_3]),dt=0.01,tacc=0.05,qdmax = pi) 

#->  To dropping position 2
path2_droppin1 = rtb.jtraj(pick2_path2.q[-1], q_droppin_2, 30)
path2_droppin2 = move_up_down(robot, path2_droppin1.q[-1], 'down', lift = 0.04)

path_droppin_bat2 = combine_trajectories([path2_droppin1, path2_droppin2])

    #-> Combine route for battery 2
path2_instruction ={'picking': pick2_path1.q, 'taking': pick2_path2.q, 'dropping': path_droppin_bat2}

# LISTS FOR HOLDING DATA
flip_coin_data = []
rot_coin = []
path_instruction = []

if is_battery_there == '1': # if only battery 1 is put in 
    flip_coin_data.append(battery_1_position)
    rot_coin.append([-0.002,0,0.0043,14])
    path_instruction.append(path1_instruction)

if is_battery_there == '2': # if only battery 2 is put in
    flip_coin_data.append(battery_2_position)
    rot_coin.append([0.002,0,0.0043,-14])
    path_instruction.append(path2_instruction)

if is_battery_there == '12': # if both batteries are put in
    flip_coin_data.append(battery_1_position)
    flip_coin_data.append(battery_2_position)
    rot_coin.append([0.002,0,0.0043,-14])
    rot_coin.append([0.002,0,0.0043,-14])
    path_instruction.append(path1_instruction)
    path_instruction.append(path2_instruction)

print(flip_coin_data)

for i in range(len(flip_coin_data)):

    battery_position = flip_coin_data[i]

    path_to_battery = rtb.jtraj(lift_coin_up[-1], battery_position, 30)

    #Simulate and send command:
    move_simulation_robot(robot = robot, path= path_to_battery.q, env= env, dt = 0.05, gripper = gripper, cam = cam, pin = coin, TCR = TCR, TGR = TGR, TCP = TCC)
    arrived = True
    #send_action_client(arrived, path_to_battery.q, goal, start_time, client, speed=1)

    ##1 -> down 2
    path_down_battery = move_up_down(robot, path_to_battery.q[-1], 'down', lift = 0.05)
    move_simulation_robot(robot = robot, path= path_down_battery.q, env= env, dt = 0.05, gripper = gripper, cam = cam, pin = coin, TCR = TCR, TGR = TGR, TCP = TCC)
    arrived = True
    #send_action_client(arrived, path_down_battery.q, goal, start_time, client, speed=1)

    #### 3.2 START TO FLIP THE BATTERY 
    q_start = list(path_down_battery.q[-1])

    # Coin's desired position
    coin_end = collisionObj.Cylinder(radius = 0.01, length= 0.005, pose = coin.T, color = [0.0,0.5,0.0,0.5])
    rot_deg = rot_coin[i][3]
    coin_end.T = transl(rot_coin[i][0],0,0.0043) @ coin.T @ troty(rot_deg*pi/180) # apply lifting and rotation
    env.add(coin_end)

    q_coin_end = solve_for_valid_ik(robot=robot, obj_pose= SE3(coin_end.T), relative_pose = TCC.inv(), q_guess = q_start)
    q_coin_end[-1] = q_start[-1]

    # Create a trajectory to flip the battery:
    path_coin = rtb.jtraj(q_start, q_coin_end, 20)
    path_coin_up = rtb.jtraj(path_coin.q[-1], battery_position, 20)
    path_back_home = rtb.jtraj(path_coin_up.q[-1], lift_coin_up[-1],30)

    # combine_path = combine_3_trajectory(path_coin, path_coin_up, path_back_home)
    combine_path = combine_trajectories([path_coin, path_coin_up, path_back_home])

    # Simulate and send command:
    move_simulation_robot(robot = robot, path= combine_path, env= env, dt = 0.05, gripper = gripper, cam = cam, pin = coin, TCR = TCR, TGR = TGR, TCP = TCC)
    arrived = True
    #send_action_client(arrived, combine_path, goal, start_time, client, speed=6)

#-> Move to the position to drop the coin:
q_drop = [99.23, -75.10, 45.56, -66.99, -85.81, 351.35]
q_drop_radian = [math.radians(joint_home_degree) for joint_home_degree in q_drop]

path_drop = rtb.jtraj(lift_coin_up[-1], q_drop_radian, 30)
move_simulation_robot(robot = robot, path= path_drop.q, env= env, dt = 0.05, gripper = gripper, cam = cam, pin = coin, TCR = TCR, TGR = TGR, TCP = TCC)
arrived = True
#send_action_client(arrived, path_drop.q, goal, start_time, client, speed=2)

# while not rospy.is_shutdown():
#     pub.publish(moveGripperToPosition(400, 280))
#     rospy.sleep(0.25)
#     break

########################## 5. BEGIN TO PLACE THE BATTERY IN A CORRECT ORDER ###########################
for instruction in path_instruction:
    # Move to picking battery
    move_simulation_robot(robot = robot, path= instruction['picking'], env= env, dt = 0.05, gripper = gripper, cam = cam, TCR = TCR, TGR = TGR)
    arrived = True
    #send_action_client(arrived, instruction['picking'], goal, start_time, client, speed=1)

    # while not rospy.is_shutdown():
    #     pub.publish(closeGripper(400))
    #     rospy.sleep(0.5)
    #     break

    # Move the battery out
    move_simulation_robot(robot = robot, path= instruction['taking'], env= env, dt = 0.05, gripper = gripper, cam = cam, TCR = TCR, TGR = TGR)
    arrived = True
    #send_action_client(arrived, instruction['taking'], goal, start_time, client, speed=2)

    # Move to drop battery
    move_simulation_robot(robot = robot, path= instruction['dropping'], env= env, dt = 0.05, gripper = gripper, cam = cam, TCR = TCR, TGR = TGR)
    arrived = True
    #send_action_client(arrived, path_droppin_bat1, goal, start_time, client, speed=3)

    # while not rospy.is_shutdown():
    #     pub.publish(moveGripperToPosition(400, 280))
    #     rospy.sleep(0.25)
    #     break
    