from urx import Robot
robot = Robot("192.168.1.10") # Replace with the IP address of your robot

robot.set_tcp((0, 0, 0.1, 0, 0, 0))
robot.set_speed(0.1)

from roboticstoolbox import *
from spatialmath import SE3


T = SE3(0.3, 0.2, 0.1) * SE3.Rx(-pi/2) * SE3.Ry(pi/4)
q = robot.get_inverse_kin(T)
robot.movej(q, 0.1, 0.1)
