import roboticstoolbox as rtb
import time
panda = rtb.models.URDF.Panda()

print(panda)

T = panda.fkine(panda.qz, end='panda_hand')

print(T)

panda.plot(panda.qr, backend = 'pyplot')

time.sleep(10)

