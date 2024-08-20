import numpy as np
from PIL import Image
import picamera
import picamera.array
import cv2
from object_detection import ObjectDetection
import helper
import tflite_runtime.interpreter as tflite
from time import sleep

MODEL_FILENAME = 'model.tflite'
LABELS_FILENAME = 'labels.txt'

class TFLiteObjectDetection(ObjectDetection):
    """Object Detection class for TensorFlow Lite."""
    def __init__(self, model_filename, labels):
        super(TFLiteObjectDetection, self).__init__(labels)
        self.interpreter = tflite.Interpreter(model_path=model_filename)
        self.interpreter.allocate_tensors()
        self.input_index = self.interpreter.get_input_details()[0]['index']
        self.output_index = self.interpreter.get_output_details()[0]['index']

    def predict(self, preprocessed_image):
        inputs = np.array(preprocessed_image, dtype=np.float32)[np.newaxis, :, :, :]  # Assume RGB format and add 1 dimension.
        self.interpreter.resize_tensor_input(self.input_index, inputs.shape)
        self.interpreter.allocate_tensors()
        self.interpreter.set_tensor(self.input_index, inputs)
        self.interpreter.invoke()
        return self.interpreter.get_tensor(self.output_index)[0]

# Load labels
with open(LABELS_FILENAME, 'r') as f:
    labels = [l.strip() for l in f.readlines()]

od_model = TFLiteObjectDetection(MODEL_FILENAME, labels)

def capture_and_detect():
    with picamera.PiCamera() as camera:
        camera.resolution = (640, 480)  # Adjust as needed
        with picamera.array.PiRGBArray(camera) as stream:
            while True:
                camera.capture(stream, format='bgr')
                frame = stream.array  # Frame is in BGR format suitable for OpenCV
                
                image = helper.update_orientation(frame)
                image = helper.resize_down_to_1600_max_dim(image)
                h, w = image.shape[:2]
                min_dim = min(w,h)
                max_square_image = helper.crop_center(image, min_dim, min_dim)
                augmented_image = helper.resize_to_512_square(max_square_image)
                
                predictions = od_model.predict_image(Image.fromarray(augmented_image))
                
                for pred in predictions:
                    if pred['probability'] >= 0.5:
                        # Draw bounding box and label on the frame
                        topleft = (int(pred['boundingBox']['left'] * augmented_image.shape[1]), int(pred['boundingBox']['top'] * augmented_image.shape[0]))
                        bottomright = (int(topleft[0] + pred['boundingBox']['width'] * augmented_image.shape[1]), int(topleft[1] + pred['boundingBox']['height'] * augmented_image.shape[0]))
                        cv2.rectangle(frame, topleft, bottomright, (255, 0, 0), 2)
                        label = f"{pred['tagName']} ({round(pred['probability'] * 100, 2)}%)"
                        cv2.putText(frame, label, (topleft[0], topleft[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

                # Display the frame
                cv2.imshow('Object Detection', frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to quit
                    break

                # Clear the stream in preparation for the next frame
                stream.truncate(0)

capture_and_detect()

# Make sure to release resources
cv2.destroyAllWindows()
