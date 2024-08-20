import serial
import cv2
import detection_utils as du 
import os
import time
import numpy as np
import time

# Configuration Constants
MODEL_FILENAME = 'model.tflite'
LABELS_FILENAME = 'labels.txt'
TXT_FILENAME = '/media/pi/PNSTAW/plant_data.txt'
PLANT_IDS = [f"{letter}{number:02d}" for letter in 'AB' for number in range(1, 13)]

def main():
    video_capture = du.initialize_video_capture()
    #time.sleep(5) # Give time for camera to setup
    od_model = du.loadLabelsandModel(LABELS_FILENAME, MODEL_FILENAME)
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)  # Adjust Arduino baud rate to 115200 as well
    
    while True:
        # Check IR_beam status to see if robot stopped
        if du.read_ir_beam_status():
            print("Inside Main Logic")
            frame = du.capture_image(video_capture)
            augmented_image = du.process_image(frame)
            print("Image Processed")
            predictions = du.detect_objects(augmented_image, od_model)
            image_with_info, healthy_count, unhealthy_count, flower_count, angles = du.calculate_movements_and_display(predictions, augmented_image)
            results = [healthy_count, unhealthy_count, flower_count]
            print(results)
            print(angles)
            
            for angle in angles.values():
                ser.write(f"angle:{angle}\n".encode())
                time.sleep(1)  # Delay to allow Arduino to process the command
            
            ser.write(b"reset\n")  # Reset base after operations
            
            cv2.imshow("Detection Output", image_with_info)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    video_capture.release()
    cv2.destroyAllWindows()
    ser.close()

if __name__ == '__main__':
    main()
