import rospy
from clover import srv
from std_srvs.srv import Trigger

from typing import NamedTuple
import math

import tf2_ros
import tf2_geometry_msgs

from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import BatteryState
from mavros_msgs.msg import RCIn

class PointLocal(NamedTuple):
    x: float
    y: float
    z: float

def get_distance_local(p1: PointLocal, p2: PointLocal) -> float:
    return math.sqrt((p2.x - p1.x)**2 + (p2.y - p1.y)**2 + (p2.z - p1.z)**2)

def get_distance_global(lat1, lon1, lat2, lon2):
    return math.hypot(lat1 - lat2, lon1 - lon2) * 1.113195e5

def transform_position():
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # Create PoseStamped object (or get it from a topic):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = rospy.get_rostime()
    pose.pose.position.x = 1
    pose.pose.position.y = 2
    pose.pose.position.z = 3
    pose.pose.orientation.w = 1

    frame_id = 'base_link'
    transform_timeout = rospy.Duration(0.2)
    new_pose = tf_buffer.transform(pose, frame_id, transform_timeout)

    return 0


