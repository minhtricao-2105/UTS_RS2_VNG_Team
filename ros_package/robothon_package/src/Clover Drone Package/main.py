from setup import*

# Take off 2m

take_off(1)

navigate_local_wait(3,0,0,'nan',0.2, 0.05)

navigate_local_wait(0,-3,0,'nan',0.2, 0.05)

navigate_local_wait(-3,0,0,'nan',0.2, 0.05)

navigate_local_wait(0,3,0,'nan',0.2, 0.05)

navigate_local_wait(4,-4,0,'nan',0.2, 0.05)

navigate_local_wait(-4,4,0,'nan',0.2, 0.05)

land()