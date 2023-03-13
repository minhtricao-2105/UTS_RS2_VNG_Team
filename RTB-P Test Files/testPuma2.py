import roboticstoolbox as rtb

puma = rtb.models.DH.Puma560()

traj = rtb.jtraj(puma.qz, puma.qr, 100)

puma.plot(traj.q)

T = puma.fkine([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])  # forward kinematics

q2 = puma.ikine_LM(T,[0.0,0.3,0.6,0.2,0.0,0.0])

print(q2)
