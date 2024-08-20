import sys
from threading import Timer
import numpy as np
import tflite_runtime.interpreter as tf
from PIL import Image
from object_detection import ObjectDetection
import picamera
import picamera.array
import helper
import time

MODEL_FILENAME = 'model.tflite'
LABELS_FILENAME = 'labels.txt'


class TFLiteObjectDetection(ObjectDetection):
    """Object Detection class for TensorFlow Lite"""
    def __init__(self, model_filename, labels):
        super(TFLiteObjectDetection, self).__init__(labels)
        self.interpreter = tf.Interpreter(model_path=model_filename)
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


def main():
    # Load labels
    with open(LABELS_FILENAME, 'r') as f:
        labels = [l.strip() for l in f.readlines()]

    od_model = TFLiteObjectDetection(MODEL_FILENAME, labels)
    
    found = False
    t_wait = 2  # time interval between detections (s)
    t_show = 1000  # time image shows on the screen (ms)

    with picamera.PiCamera() as camera:
        camera.resolution = (640, 480)  # Adjust as needed
        camera.start_preview()
        time.sleep(2)  # Camera warm-up time

        with picamera.array.PiRGBArray(camera) as stream:
            while True:
                camera.capture(stream, format='bgr', use_video_port=True)
                frame = stream.array  # Frame is in BGR format suitable for OpenCV

                if not found:
                    image = helper.update_orientation(frame)
                    image = helper.resize_down_to_1600_max_dim(image)
                    h, w = image.shape[:2]
                    min_dim = min(w, h)
                    max_square_image = helper.crop_center(image, min_dim, min_dim)
                    augmented_image = helper.resize_to_512_square(max_square_image)

                    predictions = od_model.predict_image(Image.fromarray(augmented_image))

                    for pred in predictions:
                        if pred['probability'] >= 0.5:
                            found = True
                            # Process predictions...
                            # Display or process the frame...
                            break  # Exit the for loop once an object is found

                if found:
                    time.sleep(t_show / 1000)  # Wait for specified time after finding an object
                    found = False  # Reset found flag

                # Clear the stream for the next frame
                stream.truncate(0)

                # Break the loop with a specific condition, e.g., a button press or a certain time elapsed
                # if some_condition:
                #     break

if __name__ == '__main__':
    main()
