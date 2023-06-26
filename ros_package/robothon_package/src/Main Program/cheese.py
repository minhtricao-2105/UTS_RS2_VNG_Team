import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2 as cv

bridge = CvBridge()
flag = 1
image = None

def image_callback(data):
    global image
    global flag

    # At the beginning, flag is true, so take the first figure:
    if flag == 1:
        # Convert ROS image message to OpenCV image
        image = bridge.imgmsg_to_cv2(data, "bgr8")
        
        cv.imwrite("captured_image.jpg", image)
        flag += 1

rospy.init_node("Realsense")

subscriber_1 = rospy.Subscriber("/camera/color/image_raw", Image, image_callback)

rospy.spin()
