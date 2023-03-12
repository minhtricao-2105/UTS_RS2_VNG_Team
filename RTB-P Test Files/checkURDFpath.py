import roboticstoolbox as rtb

# create a UR3 robot
robot = rtb.models.URDF.UR3()
print(robot.urdf_filepath)
