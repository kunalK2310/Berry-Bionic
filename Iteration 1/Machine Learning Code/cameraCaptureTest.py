##Code provided by Chat-GPT4
import picamera
import time

# Create an instance of the PiCamera class
camera = picamera.PiCamera()

try:
    # Optional: Set camera resolution
    # camera.resolution = (1024, 768)

    # Start the camera preview (useful for adjusting camera positioning)
    camera.start_preview()

    # Wait for 2 seconds to allow the camera sensor to adjust to lighting conditions
    time.sleep(5)

    # Capture an image and save it to a file
    camera.capture('/home/pi/Desktop/ML_teaching/TrainingImages/Set3/image4.jpg')

    print("Image captured and saved successfully.")
finally:
    # Stop camera preview
    camera.stop_preview()

    # Close the camera
    camera.close()
