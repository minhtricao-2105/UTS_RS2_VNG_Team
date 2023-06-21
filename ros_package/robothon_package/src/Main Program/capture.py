import pyautogui
import rospy
from sensor_msgs.msg import Image as SensorImage
from cv_bridge import CvBridge 
import numpy as np

rospy.init_node("capture_node")

image_pub = rospy.Publisher("screenshot", SensorImage, queue_size=1)

bridge = CvBridge()

rate = rospy.Rate(100)

while not rospy.is_shutdown():
    
    screenshot = pyautogui.screenshot()

    pil_image = screenshot.convert('RGB')

    np_image = np.array(pil_image)

    ros_image = bridge.cv2_to_imgmsg(np_image, "rgb8")

    image_pub.publish(ros_image)

    rate.sleep()