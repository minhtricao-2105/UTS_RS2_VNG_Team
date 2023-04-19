from spatialmath import SE3
import numpy as np

T1 = SE3(0.0, 0.5, 0.6)
T2 = SE3(0.2, 0.3, 0.4)

T = T1 - T2
print(np.linalg.norm(T[:3,3]))