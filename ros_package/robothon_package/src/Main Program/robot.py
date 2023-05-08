#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
import numpy as np
from function import*

# Array to store the position of the yellow container (AAA Batteries) from the Vision Nodes
colour_array_yellow = []

# Array to store the position of the orange container (AA Batteries) from the Vision Nodes
colour_array_orange = []

# Array to store the position of the batteries from the Vision nodes
location_array = []

# Array to store the transfer from pixel to local frame of camera:
transfer = []

def callback_2(msg):
    
    #Call out the global variable to store the data from the CV nodes:
    global location_array

    #Create a global variable to transfer pixel to local frame of the camera
    global transfer  

    # Extract the array from the message data field
    location_array = [msg.data[i:i+3] for i in range(0, len(msg.data), 3)]

    # # Call the transfer value
    transfer = transfer_local(np.array(location_array))

    # Try to print out the location of the batteries from the computer vision node:
    rospy.loginfo('Location array: %s',transfer)

def callback_3(msg):
    
    #Call out the global variable to store the data from the CV nodes:
    global colour_array_orange
    global colour_array_yellow

    # At the begining, flag will equal to 0
    flag = 0

    # The first one will be the yellow container:
    if flag == 0:
        colour_array_yellow = [msg.data[i:i+2] for i in range(0, len(msg.data), 2)]

        rospy.loginfo('Yellow array: %s', colour_array_yellow)

        flag += 1

    # The second one will be the orange container:
    if flag == 1:
        colour_array_orange = [msg.data[i:i+2] for i in range(0, len(msg.data), 2)]
        
        rospy.loginfo('Orange array: %s', colour_array_orange)

  
    
rospy.init_node('robot_node')

subscriber_2 = rospy.Subscriber('Image_Data', Float32MultiArray, callback_2)

subscriber_3 = rospy.Subscriber('Color_data', Float32MultiArray, callback_3)




rospy.spin()