import rospy
from clover import srv
from std_srvs.srv import Trigger
import math

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)

# TODO
# Description:
def get_odo_global():
    get_odometry = get_telemetry(frame_id='map')
    print(get_odometry)
    return get_odometry

# TODO
# Description:
def navigate_local_wait(x,y,z,yaw,speed, tolerance):
    navigate(x = x, y = y, z = z, yaw = float(yaw), frame_id='body', auto_arm = True)
    while not rospy.is_shutdown():
        get_odometry = get_telemetry(frame_id='navigate_target')
        print(get_odometry)
        if math.sqrt(get_odometry.x ** 2 + get_odometry.y ** 2 + get_odometry.z ** 2 < tolerance):
            break
        rospy.sleep(0.2)

# TODO
# Description:
def take_off(height):
    navigate(x = 0, y = 0, z = height, yaw = float('nan'), speed = 0.3, frame_id = 'body', auto_arm = True)
    while not rospy.is_shutdown():
        get_odometry = get_telemetry(frame_id='navigate_target')
        if math.sqrt(get_odometry.x ** 2 + get_odometry.y ** 2 + get_odometry.z ** 2 < 0.05):
            break
        rospy.sleep(0.2)

# TODO
# Description:
def landing():
    get_odo = get_telemetry(frame_id='map')
    navigate(x = 0, y = 0, z = -get_odo.z, yaw = float('nan'), speed = 0.3, frame_id = 'body', auto_arm = True)
    while not rospy.is_shutdown():
        get_odometry = get_telemetry(frame_id='navigate_target')
        if math.sqrt(get_odometry.x ** 2 + get_odometry.y ** 2 + get_odometry.z ** 2 < 0.05):
            break
        rospy.sleep(0.2)

# TODO
# Description:
def wait_arrival(tolerance):
    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)

# TODO
# Fly Circle:
def circle_motion(radius, speed):
    start = get_telemetry()
    start_stamp = rospy.get_rostime()
    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        angle = (rospy.get_rostime() - start_stamp).to_sec() * speed
        x = start.x + math.sin(angle) * radius
        y = start.y + math.cos(angle) * radius
        set_position(x=x, y=y, z=start.z)

        r.sleep()