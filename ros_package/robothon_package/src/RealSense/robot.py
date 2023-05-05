#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
import numpy as np

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)

def callback_2(msg):
    # Extract the array from the message data field
    my_array = [msg.data[i:i+3] for i in range(0, len(msg.data), 3)]
    rospy.loginfo('Received array: %s', my_array)

    # Convert my_array to numpy array
    final = np.array(my_array)


rospy.init_node('subscriber_node')

subscriber_1 = rospy.Subscriber('Computer_Vision', String, callback)

subscriber_2 = rospy.Subscriber('Image_Data', Float32MultiArray, callback_2)

rospy.spin()