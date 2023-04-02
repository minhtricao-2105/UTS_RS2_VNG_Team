import numpy as np
from spatialmath import SE3

# create the input 4x4 numpy array
T = np.array([[1., 0., 0., 0.45],
              [0., 1., 0., 0.  ],
              [0., 0., 1., 0.05],
              [0., 0., 0., 1.  ]])

# create an SE3 object from the numpy array
T_se3 = SE3(T)

# create an SE3 object representing the rotation around the local y-axis
T_local_rotated = T_se3 * SE3.Ry(np.pi/2)

# extract the 4x4 numpy array from the rotated SE3 object
T_rotated = T_local_rotated.A

print(T_rotated)
