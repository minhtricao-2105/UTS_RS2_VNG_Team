from robodk import robolink    # RoboDK API
from robodk import robomath    # Robot toolbox
import numpy as np
import random

def getZ_links(robot,joint):
    z_link = [None]*6
    H_base = robot.Parent().Pose()

    # Initialize a list to store the transformations of each link
    H_list = [H_base]

    # Iterate through each joint and calculate the transformation from the previous link to the current link
    for i in range(len(joint)):
        # Get the transformation matrix from the previous link to the current link
        H_prev = H_list[-1]
        print(H_prev)
        H_i = robot.SolveFK(H_prev, joint[i], i+1)
        z_link.append(H_i[2,3])

    return z

# Connect to the RoboDK API
RDK = robolink.Robolink()

# Get the robot item by name
robot = RDK.Item('UR3e', robolink.ITEM_TYPE_ROBOT)
# Set the simulation speed to 100% (realtime)
RDK.SimulationSpeed()
joints = robot.Joints()
joint_list = np.empty((5,6))
joint_list[0] = np.ravel(joints).tolist()
joint_list[1] = [0,-90 ,0 ,0 ,0 ,0]
joint_list[2] = [10,-90 ,30 ,0 ,30,0]
joint_list[3] = [20,-45 ,30 ,45 ,10 ,40]
joint_list[4] = [30,-45 ,45 ,0 ,0 ,30]

print(getZ_links(robot,list(joint_list[0])))

# Run through 5 joint states
for q in joint_list:
    robot.MoveJ(list(q))
    robomath.pause(1)

# Run randomly
while True:
    random_joint = [random.randint(-90, 90) for _ in range(6)]
    ee_pose = robot.SolveFK(random_joint)
    z = ee_pose[2,3]
    if z > 50:
        robot.MoveJ(random_joint)
        robomath.pause(0.5)
    else:
        continue




