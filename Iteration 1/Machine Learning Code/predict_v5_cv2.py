import sys
import tflite_runtime.interpreter as tf
import numpy as np
from PIL import Image
from object_detection import ObjectDetection
import csv
import cv2
import helper
import os  # Import os for directory operations

MODEL_FILENAME = 'model.tflite'
LABELS_FILENAME = 'labels.txt'
CSV_FILENAME = '/media/pi/PNSTAW/plant_data.csv' 

class TFLiteObjectDetection(ObjectDetection):
    """Object Detection class for TensorFlow Lite"""
    def __init__(self, model_filename, labels):
        super(TFLiteObjectDetection, self).__init__(labels)
        self.interpreter = tf.Interpreter(model_path=model_filename)
        self.interpreter.allocate_tensors()
        self.input_index = self.interpreter.get_input_details()[0]['index']
        self.output_index = self.interpreter.get_output_details()[0]['index']

    def predict(self, preprocessed_image):
        inputs = np.array(preprocessed_image, dtype=np.float32)[np.newaxis, :, :, (2, 1, 0)]  # RGB -> BGR and add 1 dimension.
        self.interpreter.resize_tensor_input(self.input_index, inputs.shape)
        self.interpreter.allocate_tensors()
        self.interpreter.set_tensor(self.input_index, inputs)
        self.interpreter.invoke()
        return self.interpreter.get_tensor(self.output_index)[0]

# Load labels
with open(LABELS_FILENAME, 'r') as f:
    labels = [l.strip() for l in f.readlines()]

od_model = TFLiteObjectDetection(MODEL_FILENAME, labels)

video_object = cv2.VideoCapture(0)

# Initialize CSV
csv_headers = ['PlantNumber', 'NumberHealthy', 'NumberUnhealthy', 'NumberFlowers']
with open(CSV_FILENAME, 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(csv_headers)

plant_number = 1
captures_per_plant = 5
results_per_plant = []

while True:
    ret, frame = video_object.read()
    cv2.imshow('frame', cv2.resize(frame, (640, 480)))  # Resizing the window

    if cv2.waitKey(1) & 0xFF == ord('t') and len(results_per_plant) < captures_per_plant:
        image = helper.update_orientation(frame)
        image = helper.resize_down_to_1600_max_dim(image)
        h, w = image.shape[:2]
        min_dim = min(w,h)
        max_square_image = helper.crop_center(image, min_dim, min_dim)
        augmented_image = helper.resize_to_512_square(max_square_image)
        predictions = od_model.predict_image(Image.fromarray(augmented_image))
        
        font = cv2.FONT_HERSHEY_SIMPLEX

        healthy_count = 0
        unhealthy_count = 0
        flower_count = 0

        for pred in predictions:
            if pred['probability'] >= 0.6:
                # Update count based on prediction
                if pred['tagName'].lower() == 'healthy':
                    healthy_count += 1
                elif pred['tagName'].lower() == 'unhealthy':
                    unhealthy_count += 1
                elif pred['tagName'].lower() == 'flower':
                    flower_count += 1
                
                ## Draw boudning boxes and display text
                topleft = (int(pred['boundingBox']['left'] * augmented_image.shape[0]), int(pred['boundingBox']['top'] * augmented_image.shape[1]))
                bottomright = (int(topleft[0] + pred['boundingBox']['width'] * augmented_image.shape[0]), int(topleft[1] + pred['boundingBox']['height'] * augmented_image.shape[0]))
#                 print(topleft)
#                 print(bottomright)
                
                # text to put
                text=f"{pred['tagName']} | {round(pred['probability'] * 100, 2)}%" 
                
                # draw rectangle and text on img
                cv2.rectangle(augmented_image, topleft, bottomright, (255, 0 ,0), 2)
                cv2.putText(augmented_image, text, topleft, font, 0.5, (255, 0, 0), 1, cv2.LINE_AA)

        results_per_plant.append((healthy_count, unhealthy_count, flower_count))

        if len(results_per_plant) == captures_per_plant:  # After capturing 5 images for the current plant
            avg_results = [round(sum(x) / len(results_per_plant)) for x in zip(*results_per_plant)]
            with open(CSV_FILENAME, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([f"Plant {plant_number:02d}"] + avg_results)

            # Reset for next plant
            results_per_plant.clear()
            plant_number += 1

        # Create folder for each plant and save image
        plant_folder = f"Plant_{plant_number:02d}"
        if not os.path.exists(plant_folder):
            os.makedirs(plant_folder)
        image_path = os.path.join(plant_folder, f'Image_{len(results_per_plant)}.jpg')
        cv2.imwrite(image_path, augmented_image)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

video_object.release()
cv2.destroyAllWindows()