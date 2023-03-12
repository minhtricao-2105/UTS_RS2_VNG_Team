import roboticstoolbox as rtb
from spatialmath import SE3
import numpy as np
from roboticstoolbox.backends.PyPlot import PyPlot

#create Puma560
p560 = rtb.models.DH.Puma560()

ur3 = rtb.models.UR3()
ur3.base = SE3.Tx(1)

#create Vpython environment
env = PyPlot()
env.launch(realtime=True)
env.add(p560)
env.add(ur3)

qt = rtb.tools.trajectory.jtraj(p560.qz, p560.qr, 50)

for q in qt.q:
    p560.q = q
    ur3.q = q - np.pi
    env.step(0.05)

env.hold()