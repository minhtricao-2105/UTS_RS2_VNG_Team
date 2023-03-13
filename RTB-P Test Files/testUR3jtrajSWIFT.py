import swift
import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3

# create a UR3 robot
robot = rtb.models.UR3()

#create Swift environment and add robot
env = swift.Swift()
env.launch(realtime = "True")
env.add(robot)


# define the end pose
T2 = SE3(0.3, 0.2, 0.3) * SE3.Rz(np.pi/4)

print(type(robot.qr))

# compute the inverse kinematics for the end-effector poses
q2 = robot.ikine_LM(T2,q0 = robot.qr) 


# use jtraj to create a trajectory
path = rtb.jtraj(robot.qr, q2.q, 50)

# simulate the robot moving along the trajectory
dt = 0.05

for q in path.q:
	robot.q = q
	#robot.plot(q,backend = 'Swift')
	env.step(dt)

env.hold()
