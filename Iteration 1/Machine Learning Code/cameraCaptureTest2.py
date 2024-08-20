import picamera
import time

# Create an instance of the PiCamera class
##Code provided by Chat-GPT4

camera = picamera.PiCamera()

try:
    # Optional: Set camera resolution
    # camera.resolution = (1024, 768)

    # Start the camera preview (useful for adjusting camera positioning)
    camera.start_preview()

    # Wait for 2 seconds to allow the camera sensor to adjust to lighting conditions
    time.sleep(2)

    # Loop through i=1 to 10
    for i in range(1, 11):  # Starts from 1, goes up to and includes 10
        # Capture an image and save it to a file with loop counter in the filename
        filename = f'/home/pi/Desktop/ML_teaching/TrainingImages/Set5/image{i}.jpg'
        camera.capture(filename)

        print(f"Image {i} captured and saved successfully as {filename}.")

        # Wait for 10 seconds before capturing the next image, unless it's the last one
        if i < 10:  # Avoid waiting after the last image is captured
            time.sleep(10)

finally:
    # Stop camera preview
    camera.stop_preview()

    # Close the camera
    camera.close()
