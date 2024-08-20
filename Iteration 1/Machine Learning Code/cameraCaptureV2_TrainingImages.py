import cv2
import os

# Create the directory for storing images if it doesn't already exist
save_folder = "Training Images PiCamera"
if not os.path.exists(save_folder):
    os.makedirs(save_folder)

gst_pipeline="nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)1080, height=(int)720, format=(string)NV12, framerate=(fraction)30/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! appsink"
cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
capture_count = 99 # Change back to 1 if starting from beginning

while True:
    ret, img = cap.read()

    if not ret:
        print("Failed to capture image")
        break

    cv2.imshow('frame', img)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('t'):
        # Save the captured image in the designated folder
        image_path = os.path.join(save_folder, f'Image{capture_count}.jpg')
        cv2.imwrite(image_path, img)
        print(f"Image saved as {image_path}")
        capture_count += 1
    elif key == ord('q'):
        # Exit the loop if 'q' is pressed
        break

cap.release()
cv2.destroyAllWindows()
