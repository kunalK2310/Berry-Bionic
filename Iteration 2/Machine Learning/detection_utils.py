import cv2
from PIL import Image
import numpy as np
import helper
from object_detection import ObjectDetection
import tflite_runtime.interpreter as tf

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
    
def initialize_video_capture():
    
    # Make changes here to use with python3.8
    gst_pipeline="nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)1080, height=(int)720, format=(string)NV12, framerate=(fraction)30/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! appsink"
    cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
    return cap

def capture_image(video_capture):
    ret, frame = video_capture.read()
    if not ret:
        raise Exception("Failed to capture image")
    return frame

def process_image(frame):
     # Process the image and make predictions as before
    image = helper.update_orientation(frame)
    image = helper.resize_down_to_1600_max_dim(image)
    h, w = image.shape[:2]
    min_dim = min(w,h)
    max_square_image = helper.crop_center(image, min_dim, min_dim)
    augmented_image = helper.resize_to_512_square(max_square_image)
    return augmented_image


def loadLabelsandModel(filename,MODEL_FILENAME):
    # Load labels
    with open(filename, 'r') as f:
        labels = [line.strip() for line in f.readlines()]
    # Load model
    od_model = TFLiteObjectDetection(MODEL_FILENAME, labels)
    return od_model

def detect_objects(augmented_image, od_model):
    return od_model.predict_image(Image.fromarray(augmented_image))

def calculate_angle(origin, point):
    vector_origin_to_point = np.array([point[0] - origin[0], point[1] - origin[1]])
    vector_origin_to_vertical = np.array([0, -1])
    dot_product = np.dot(vector_origin_to_point, vector_origin_to_vertical)
    magnitude = np.linalg.norm(vector_origin_to_point)
    angle_rad = np.arccos(dot_product / magnitude)
    angle_deg = np.degrees(angle_rad)
    if point[0] < origin[0]:
        angle_deg = -angle_deg
    return angle_deg

def draw_line_and_angle(image, origin, point, angle, label):
    color = (0, 255, 0) if label == "Unhealthy" else (0, 0, 255)
    cv2.line(image, origin, point, color, 2)
    angle_text = f"{label}: {angle:.2f}°"
    cv2.putText(image, angle_text, point, cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
    
def calculate_movements_and_display(predictions, image):
    healthy_count = 0
    unhealthy_count = 0
    flower_count = 0
    stem_location = None
    flower_locations = []
    unhealthy_locations = []
    
    # Parse predictions
    for pred in predictions:
        if pred['probability'] < 0.5:
            continue

        centerX = int(pred['boundingBox']['left'] * image.shape[1] + pred['boundingBox']['width'] * image.shape[1] / 2)
        centerY = int(pred['boundingBox']['top'] * image.shape[0] + pred['boundingBox']['height'] * image.shape[0] / 2)
        center = (centerX, centerY)

        if pred['tagName'].lower() == 'stem':
            stem_location = center
        elif pred['tagName'].lower() == 'healthy':
            healthy_count += 1
        elif pred['tagName'].lower() == 'unhealthy':
            unhealthy_locations.append(center)
            unhealthy_count += 1
        elif pred['tagName'].lower() == 'flower':
            flower_locations.append(center)
            flower_count += 1

    # Draw and calculate angles if stem is identified
    angles = {}
    if stem_location:
        # Draw vertical reference line from the stem
        cv2.line(image, stem_location, (stem_location[0], 0), (255, 0, 0), 2)

        for i, location in enumerate(unhealthy_locations):
            angle = calculate_angle(stem_location, location)
            angles[f'unhealthy_{i}'] = angle
            draw_line_and_angle(image, stem_location, location, angle, "Unhealthy")

        if flower_locations:
            angle = calculate_angle(stem_location, flower_locations[0])
            angles['flower'] = angle
            draw_line_and_angle(image, stem_location, flower_locations[0], angle, "Flower")

    return image, healthy_count, unhealthy_count, flower_count, angles


def log_results(filename, results):
    # Log results to a CSV file
    import csv
    with open(filename, 'a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(results)

def read_ir_beam_status():
    # Placeholder function to simulate an IR beam breaker input
    # Replace this with actual GPIO input reading if using Raspberry Pi etc.
    import random
    return random.choice([True, False])  # Simulate random stop events

def send_data_to_arduino(serial_conn, angles):
    # Format and send data to Arduino via serial
    for key, angle in angles.items():
        command = f"{key}:{angle}\n"
        serial_conn.write(command.encode())
