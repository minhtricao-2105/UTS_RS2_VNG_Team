from robodk import robolink    # RoboDK API
from robodk import robomath    # Robot toolbox
import numpy as np
import random

# Connect to the RoboDK API
RDK = robolink.Robolink()

# Get the robot item by name
robot = RDK.Item('UR3e', robolink.ITEM_TYPE_ROBOT)
# Set the simulation speed to 100% (realtime)
RDK.SimulationSpeed()

#pose 0:
home_rad = [0, -np.pi/2, 0, 0, 0, 0]
home = [x * 180/np.pi for x in home_rad]
robot.MoveJ(home)

#pose 1:
j1 = [20 ,-100, 30, 80, 20, 0]
robot.MoveJ(j1)

#pose 2:
j2 = [40 ,-120, 0, 30, 30, 180]
robot.MoveJ(j2)

#pose 3:
j3 = [40, -54.87, -58.05, 22.93, 30, 180]
robot.MoveL(j3)

#pose 4:
j4 = [71.45, -13.38, -91.06, 56.55, 42.37, 129.28]
robot.MoveL(j4)

# #pose 5:
j5_1 = [62.14, -27.18, -80.86, 51.18, 36.66, 140.88]
j5_2 = [63.22, -37.91, -76.33, 58.97, -37.26, 139.38]
robot.MoveC(j5_1, j5_2)

print("DONE!")
