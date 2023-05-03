from setup import*

# Create a ROS Node for simulator:
rospy.init_node('flight')

take_off(1)

circle_motion(1, 0.4)

land()