import time
import board
import busio
import Jetson.GPIO as GPIO
import adafruit_vl6180x
from adafruit_tca9548a import TCA9548A
from adafruit_vl6180x import VL6180X
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
import csv
import os
import time
import cv2
import numpy as np
from PIL import Image
import tflite_runtime.interpreter as tf


# Global dictionary to hold sensor references
sensors = {}

def initialize_sensors():
    global sensors
    # Create I2C bus
    i2c_bus = busio.I2C(board.SCL, board.SDA)

    # Initialize the TCA9548A I2C multiplexer
    tca = TCA9548A(i2c_bus)

    # Mapping sensors to channels
    sensor_channels = {
        'LeftWallFront': 0,
        'LeftWallRear': 6,
        'LeftPlant1': 7,
        'LeftPlant2': 1,
        'RightWallFront': 2,
        'RightWallRear': 5,
        'RightPlant1': 4,
        'RightPlant2': 3
    }

    # Create sensor instances for all channels and store them in the sensors dictionary
    for name, channel in sensor_channels.items():
        sensors[name] = VL6180X(tca[channel])

    print("Sensors initialized")

def check_sensors():
    # Assuming sensors have been initialized and stored in the global dictionary
    global sensors
    
    # Read the distance from left and right wall front sensors
    left_distance = sensors['LeftWallFront'].range
    right_distance = sensors['RightWallFront'].range

    # Determine which wall to track based on sensor distances
    wall_track = 'left' if left_distance < right_distance else 'right'
    
    print(f"Checking which wall to track: {wall_track}")
    return wall_track

# Motor control pins
motor1_step = 17   # BCM pin 17 for Motor 1 Step
motor1_dir = 4     # BCM pin 4 for Motor 1 Direction
motor2_step = 27   # BCM pin 27 for Motor 2 Step
motor2_dir = 22    # BCM pin 22 for Motor 2 Direction

# Setup GPIO only once to avoid reconfiguration errors
GPIO.setmode(GPIO.BCM)  # Broadcom pin-numbering scheme
GPIO.setup([motor1_step, motor1_dir, motor2_step, motor2_dir], GPIO.OUT)

def step_motors(steps, clockwise=True, step_delay=0.0015):
    """
    Function to perform steps simultaneously on both motors.
    - steps: number of steps to perform
    - clockwise: True for clockwise, False for counterclockwise
    - step_delay: delay between each step in seconds
    """
    # Set direction for both motors
    GPIO.output(motor1_dir, GPIO.HIGH if clockwise else GPIO.LOW)
    GPIO.output(motor2_dir, GPIO.HIGH if clockwise else GPIO.LOW)

    # Perform steps
    for _ in range(steps):
        GPIO.output([motor1_step, motor2_step], GPIO.HIGH)
        time.sleep(step_delay)
        GPIO.output([motor1_step, motor2_step], GPIO.LOW)
        time.sleep(step_delay)

def move_platform(direction, steps, step_delay=0.0015):
    """
    Moves the platform up or down based on the direction specified.
    - direction: 'up' or 'down'
    - steps: number of steps to move
    - step_delay: delay between each step
    """
    if direction == 'up':
        print("Moving platform UP")
        step_motors(steps, clockwise=True, step_delay=step_delay)
    elif direction == 'down':
        print("Moving platform DOWN")
        step_motors(steps, clockwise=False, step_delay=step_delay)
    else:
        raise ValueError("Direction must be 'up' or 'down'")

# Setup the I2C bus and PCA9685
i2c_bus = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c_bus)
pca.frequency = 50

# Initialize the servo on channel 9
servo_arm = servo.Servo(pca.channels[9], min_pulse=500, max_pulse=2500)

def gradual_move_to_angle(target_angle, step_delay=0.03):
    """Gradually move the servo to the target angle."""
    current_angle = int(servo_arm.angle) if servo_arm.angle is not None else 90  # Start from 90 if not set
    step = 1 if target_angle > current_angle else -1
    
    for angle in range(current_angle, target_angle + step, step):
        servo_arm.angle = angle
        time.sleep(step_delay)

def adjust_arm_position(wall_track):
    """Adjust arm based on the wall tracking direction."""
    target_angle = 30 if wall_track == 'right' else 150
    print(f"Gradually moving arm to {target_angle} degrees for {wall_track} wall tracking.")
    gradual_move_to_angle(target_angle)

def disable_servo():
    """Disable the servo to prevent it from holding torque."""
    servo_arm.angle = None
    print("Servo disabled to prevent damage.")

def rotate_arm(direction):
    print(f"Rotating arm towards {direction}")

# Constants
MODEL_FILENAME = 'model.tflite'
LABELS_FILENAME = 'labels.txt'
CSV_FILENAME = '/home/kunal/Desktop/ML_teaching/plant_data.csv'

# Load TensorFlow Lite model and prepare labels
def load_model_and_labels():
    with open(LABELS_FILENAME, 'r') as f:
        labels = [line.strip() for line in f.readlines()]
    interpreter = tf.Interpreter(model_path=MODEL_FILENAME)
    interpreter.allocate_tensors()
    return interpreter, labels

# Predict with the TensorFlow Lite model
def predict_with_model(interpreter, image, input_index, output_index):
    inputs = np.array(image, dtype=np.float32)[np.newaxis, :, :, :]
    interpreter.set_tensor(input_index, inputs)
    interpreter.invoke()
    return interpreter.get_tensor(output_index)[0]

# Capture and process images from the camera
def capture_and_process_images(video_source, interpreter, input_index, output_index, captures_per_plant):
    video_object = cv2.VideoCapture(video_source)
    results = []
    for _ in range(captures_per_plant):
        ret, frame = video_object.read()
        if not ret:
            break
        processed_image = preprocess_image(frame)
        predictions = predict_with_model(interpreter, processed_image, input_index, output_index)
        results.append(analyze_predictions(predictions))
    video_object.release()
    return results

# Initialize CSV for output
def initialize_csv():
    with open(CSV_FILENAME, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['PlantNumber', 'NumberHealthy', 'NumberUnhealthy', 'NumberFlowers'])

# Append results to CSV
def append_to_csv(plant_number, results):
    with open(CSV_FILENAME, 'a', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow([plant_number] + results)

def identify_angles():
    print("Identifying angles for unhealthy parts and flowers")
    return [45, 90]  # Placeholder angles

def rotate_turntable_to_angle(angle):
    print(f"Rotating turntable to {angle} degrees")


def perform_trimming_action():
    print("Performing trimming action")


def take_photo():
    print("Taking photo after action")

def move_to_next_plant():
    print("Moving to the next plant")

def detect_plant():
    print("Plant detected")
    return True  # Assume a plant is always detected for demo

def is_last_plant():
    print("Checking if it is the last plant")
    return False  # Always returns False for continuous loop in demo
