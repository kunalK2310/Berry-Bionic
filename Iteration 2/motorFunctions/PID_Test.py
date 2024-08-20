import time
import board
import busio
from adafruit_pca9685 import PCA9685
import Jetson.GPIO as GPIO

# Setup PCA9685 and GPIO
i2c_bus = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c_bus)
pca.frequency = 100

GPIO.setmode(GPIO.BCM)
motor_pins = {
    'AIN1_left': 24,
    'AIN2_left': 25,
    'AIN1_right': 18,
    'AIN2_right': 23,
}
GPIO.setup(list(motor_pins.values()), GPIO.OUT)

# PWM channels for motor pairs
pwm_left_front = pca.channels[0]
pwm_left_rear = pca.channels[1]
pwm_right_front = pca.channels[3]
pwm_right_rear = pca.channels[2]

def set_motor_speed(pwm, speed):
    pwm.duty_cycle = int((speed / 100) * 0xffff)

def set_motor_directions(left_forward, right_forward):
    GPIO.output(motor_pins['AIN1_left'], GPIO.LOW if left_forward else GPIO.HIGH)
    GPIO.output(motor_pins['AIN2_left'], GPIO.HIGH if left_forward else GPIO.LOW)
    GPIO.output(motor_pins['AIN1_right'], GPIO.LOW if right_forward else GPIO.HIGH)
    GPIO.output(motor_pins['AIN2_right'], GPIO.HIGH if right_forward else GPIO.LOW)

# PID constants
Kp, Ki, Kd = 0.1, 0.01, 0.05
integral = 0
last_error = 0
desired_distance = 20  # mm

def pid_control(distance_front, distance_rear):
    global integral, last_error
    error = distance_front - distance_rear
    integral += error
    derivative = error - last_error

    output = Kp * error + Ki * integral + Kd * derivative
    last_error = error
    return output

try:
    while True:
        # Simulated sensor readings
        distance_front = read_distance(front_sensor)
        distance_rear = read_distance(rear_sensor)
        correction = pid_control(distance_front, distance_rear)

        # Calculate individual motor speeds
        left_motor_speed = max(0, min(100, 70 + correction))
        right_motor_speed = max(0, min(100, 70 - correction))

        # Set motor directions based on PID output
        set_motor_directions(left_forward=True, right_forward=True)
        set_motor_speed(pwm_left_front, left_motor_speed)
        set_motor_speed(pwm_left_rear, left_motor_speed)
        set_motor_speed(pwm_right_front, right_motor_speed)
        set_motor_speed(pwm_right_rear, right_motor_speed)

        time.sleep(0.1)

finally:
    # Stop all motors
    set_motor_speed(pwm_left_front, 0)
    set_motor_speed(pwm_left_rear, 0)
    set_motor_speed(pwm_right_front, 0)
    set_motor_speed(pwm_right_rear, 0)
    
    # Cleanup
    pca.deinit()
    GPIO.cleanup()
    print("Cleanup successful.")
