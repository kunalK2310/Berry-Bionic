import time
import board
import busio
from adafruit_pca9685 import PCA9685
import Jetson.GPIO as GPIO

# Cleanup any previous GPIO settings
GPIO.cleanup()

# Setup GPIO mode
GPIO.setmode(GPIO.BCM)

# Initialize the I2C bus and PCA9685 module
i2c_bus = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c_bus)
pca.frequency = 100  # Set the PWM frequency to 100 Hz

# Define motor control GPIO pins (direction pins) for shared usage
motor_pins = {
    'AIN1_left': 24,   # AIN1 for left motors
    'AIN2_left': 25,   # AIN2 for left motors
    'AIN1_right': 18,  # AIN1 for right motors
    'AIN2_right': 23,  # AIN2 for right motors
}

# Setup motor control pins as outputs
GPIO.setup(list(motor_pins.values()), GPIO.OUT)

# Initialize PWM outputs from PCA9685 for both left and right motor pairs
pwm_left_front = pca.channels[0]  # PWM channel for left front motor
pwm_left_rear = pca.channels[1]   # PWM channel for left rear motor
pwm_right_front = pca.channels[3] # PWM channel for right front motor
pwm_right_rear = pca.channels[2]  # PWM channel for right rear motor

def set_motor_speed(pwm, speed):
    """ Set motor speed with PWM (0-100%) """
    if speed < 0 or speed > 100:
        raise ValueError("Speed must be between 0 and 100")
    duty_cycle = int((speed / 100) * 0xffff)  # Convert percentage to 16-bit duty cycle
    pwm.duty_cycle = duty_cycle

def move_forward(speed=50):
    """ Move forward by setting both motors to move in the forward direction """
    GPIO.output(motor_pins['AIN1_left'], GPIO.LOW)
    GPIO.output(motor_pins['AIN2_left'], GPIO.HIGH)
    GPIO.output(motor_pins['AIN1_right'], GPIO.LOW)
    GPIO.output(motor_pins['AIN2_right'], GPIO.HIGH)
    set_motor_speed(pwm_left_front, speed)
    set_motor_speed(pwm_left_rear, speed)
    set_motor_speed(pwm_right_front, speed)
    set_motor_speed(pwm_right_rear, speed)

def move_reverse(speed=50):
    """ Move reverse by setting both motors to move in the reverse direction """
    GPIO.output(motor_pins['AIN1_left'], GPIO.HIGH)
    GPIO.output(motor_pins['AIN2_left'], GPIO.LOW)
    GPIO.output(motor_pins['AIN1_right'], GPIO.HIGH)
    GPIO.output(motor_pins['AIN2_right'], GPIO.LOW)
    set_motor_speed(pwm_left_front, speed)
    set_motor_speed(pwm_left_rear, speed)
    set_motor_speed(pwm_right_front, speed)
    set_motor_speed(pwm_right_rear, speed)

def turn_left():
    """ Turn left by rotating the front left wheel backward and front right wheel forward """
    GPIO.output(motor_pins['AIN1_left'], GPIO.HIGH)  # Reverse left
    GPIO.output(motor_pins['AIN2_left'], GPIO.LOW)
    GPIO.output(motor_pins['AIN1_right'], GPIO.LOW)  # Forward right
    GPIO.output(motor_pins['AIN2_right'], GPIO.HIGH)
    set_motor_speed(pwm_left_front, 50)
    set_motor_speed(pwm_right_front, 50)

def turn_right():
    """ Turn right by rotating the front right wheel backward and front left wheel forward """
    GPIO.output(motor_pins['AIN1_left'], GPIO.LOW)  # Forward left
    GPIO.output(motor_pins['AIN2_left'], GPIO.HIGH)
    GPIO.output(motor_pins['AIN1_right'], GPIO.HIGH)  # Reverse right
    GPIO.output(motor_pins['AIN2_right'], GPIO.LOW)
    set_motor_speed(pwm_left_front, 50)
    set_motor_speed(pwm_right_front, 50)

try:
    print("Moving forward")
    move_forward(70)
    time.sleep(4)

    print("Moving reverse")
    move_reverse(70)
    time.sleep(4)

    print("Turning left 90 degrees")
    turn_left()
    time.sleep(5)  # Adjust timing based on actual turn rate and desired angle

    print("Turning right 90 degrees")
    turn_right()
    time.sleep(5)  # Adjust timing based on actual turn rate and desired angle

finally:
    # Stop all motors
    set_motor_speed(pwm_left_front, 0)
    set_motor_speed(pwm_left_rear, 0)
    set_motor_speed(pwm_right_front, 0)
    set_motor_speed(pwm_right_rear, 0)
    
    # Properly deinitialize the PCA9685 and cleanup GPIO
    pca.deinit()
    GPIO.cleanup()
    print("Test complete. Cleanup successful.")
