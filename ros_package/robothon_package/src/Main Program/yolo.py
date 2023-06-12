import cv2

# Load the YOLO model
net = cv2.dnn.readNetFromDarknet("yolov3.cfg", "yolov3.weights")
