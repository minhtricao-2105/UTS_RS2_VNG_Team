import sys
sys.path.append('../Modules')
from ur3module import *

def cam_move(cam,robot,T):
    cam.T = robot.fkine(robot.q)*T

env = swift.Swift()
env.launch(realtime = True)
dt = 0.05

q_deg = [57.11, -106.32, 56.84, -44.7, -85.81, 328.40]
robot = rtb.models.UR3()
robot.q = [x*pi/180 for x in q_deg]

gripper_path = "/home/quangngo/Desktop/RTB-P Test files/SomeApplications/CAMGRIPPER.STL"
gripper = collisionObj.Mesh(filename=gripper_path,pose = SE3(0,0,0),scale=[0.001, 0.001, 0.001],color = [0.5,0.1,0.1,1])

cam = collisionObj.Cuboid(scale=[0.03,0.06,0.015], pose = SE3(0,0,0), color = [0.5,0.5,0.1,1])

#gripper in robot ee frame
TGR = SE3.Rx(pi)*SE3(0,-0.105,-0.175)

#cam in robot ee frame
TCR = SE3(0.085,0,0.09)

cam_move(gripper,robot, TGR)
cam_move(cam, robot, TCR*SE3.Ry(pi/2))

env.add(robot)
env.add(gripper)
env.add(cam)

q_end = [0,-pi/2,0,0,0,0]
path = rtb.jtraj(robot.q, q_end,50)

# for q in path.q:
#     robot.q = q
#     cam_move(gripper,robot, TGR)
#     cam_move(cam,robot,TCR)
#     env.step(dt)
    
print("CAM:\n",cam.T)

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


# Define the camera parameters
pixel_width = 640  # Width of the image in pixels
pixel_height = 480  # Height of the image in pixels
focal_length = 848  # Focal length of the camera in pixels

camera_transform = cam.T

# Define the point in Pixel frame
pixel_x_1 = 265 #265  # X-coordinate of the point in pixels
pixel_y_1 = 135 #135  # Y-coordinate of the point in pixels

pixel_x_2 = 265  # X-coordinate of the point in pixels
pixel_y_2 = 340  # Y-coordinate of the point in pixels

global_position_1 = cam_to_global(pixel_x_1,pixel_y_1)
global_position_2 = cam_to_global(pixel_x_2,pixel_y_2)


# Print the result
print("Pixel frame coordinates: ({}, {})".format(pixel_x_1, pixel_y_1))
print("Global frame coordinates: ({}, {}, {})".format(global_position_1[0], global_position_1[1], global_position_1[2]))

print("\n")
print("Pixel frame coordinates: ({}, {})".format(pixel_x_2, pixel_y_2))
print("Global frame coordinates: ({}, {}, {})".format(global_position_2[0], global_position_2[1], global_position_2[2]))

print("X different: ", 100* abs(global_position_1[0] - global_position_2[0]), "cm")
print("Y different: ", 100* abs(global_position_1[1] - global_position_2[1]), "cm")

env.hold()



