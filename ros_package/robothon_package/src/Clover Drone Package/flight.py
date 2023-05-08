import rospy
from clover import srv
from std_srvs.srv import Trigger
from setup import*
import math

# Create a ROS Node for simulator:
rospy.init_node('flight')

# Setup parameter for clover:
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)

RADIUS = 0.6 # m
SPEED = 0.3 # rad / s
start = get_telemetry()
start_stamp = rospy.get_rostime()
r = rospy.Rate(10)

navigate(x = 0, y = 0, z = 1.5, yaw = float('nan'), speed = 0.1, frame_id = 'body', auto_arm = True)

rospy.sleep(10)

while not rospy.is_shutdown():
    angle = (rospy.get_rostime() - start_stamp).to_sec() * SPEED
    x = start.x + math.sin(angle) * RADIUS
    y = start.y + math.cos(angle) * RADIUS
    set_position(x=x, y=y, z=start.z)
    r.sleep() 

land()
