import cv2

# initialize video capture
cap = cv2.VideoCapture(0)

# initialize window for object selection
cv2.namedWindow("Object Selection")
bbox = cv2.selectROI("Object Selection", frame)

# initialize object tracker
tracker = cv2.TrackerMeanShift_create()
tracker.init(frame, bbox)

while True:
    # read a frame from the video stream
    ret, frame = cap.read()
    if not ret:
        break

    # update the object tracker
    success, bbox = tracker.update(frame)

    if success:
        # draw bounding box around tracked object
        x, y, w, h = [int(v) for v in bbox]
        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2, 1)
    else:
        cv2.putText(frame, "Object lost", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)

    # display the frame
    cv2.imshow("Object Tracking", frame)

    # exit if 'q' key is pressed
    if cv2.waitKey(1) == ord('q'):
        break

# release resources
cap.release()
cv2.destroyAllWindows()
