import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3
import matplotlib.pyplot as plt

# create a random SE(3) transformation matrix
T = SE3.Rand()

# create a robotic arm with 2 links of length 1
robot = rtb.models.DH.Puma560()

# set the robot to the random SE(3) transformation matrix
robot.base = T

# create a joint configuration
q = np.random.rand(robot.n)

# plot the robot in the new configuration
robot.plot(q)

# show the plot
robot.teach()



