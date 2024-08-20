import cv2

# Open the video capture with the GStreamer pipeline
cap = cv2.VideoCapture("nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)1280, height=(int)720, format=(string)NV12, framerate=(fraction)30/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink", cv2.CAP_GSTREAMER)

if not cap.isOpened():
    print("Failed to open camera")
else:
    ret, frame = cap.read()
    if ret:
        cv2.imshow('frame', frame)
        cv2.waitKey(0)
    else:
        print("Failed to capture image")
    cap.release()

cv2.destroyAllWindows()
