#! /usr/bin/python
import rospy
from ur_msgs import ToolDataMsg

def talker():

	rospy.init_node('openGripper')
	pub=rospy.Publisher('chatter', String, queue_size=10)
	
	
	rate=rospy.Rate(10)
	
	while not rospy.is_shutdown():
		hello_str="hello world %s" % rospy.get_time()
		rospy.loginfo(hello_str)
		pub.publish(hell_str)
		rate.sleep()

if __name__=='__main__':
  try:
    talker()
  except rospy.ROSInterruptException:
    pass
