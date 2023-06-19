import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv


rospy.init_node("node")

    # Create a publisher to publish the image topic
image_publisher = rospy.Publisher("/image_topic", Image, queue_size=10)

    # Create a CvBridge instance to convert cv2 images to ROS Image messages
bridge = CvBridge()

    # Load the image from file or capture it using cv2
image = cv.imread("media/shiraishi.jpg")

    # Convert the cv2 image to ROS Image message
ros_image = bridge.cv2_to_imgmsg(image, encoding="bgr8")

    # Publish the image
rate = rospy.Rate(10)  # Publish at a rate of 10 Hz

while not rospy.is_shutdown():
    image_publisher.publish(ros_image)
    rate.sleep()

