import roboticstoolbox as rtb
from roboticstoolbox.backends.VPython import VPython

#create Puma560
p560 = rtb.models.DH.Puma560()

#create Vpython environment
env = VPython()
env.launch(realtime=True)
env.add(p560)

qt = rtb.tools.trajectory.jtraj(p560.qz, p560.qr, 50)

for q in qt.q:
    p560.q = q
    env.step(0.05)
