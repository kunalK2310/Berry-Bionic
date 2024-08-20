import Jetson.GPIO as GPIO
import time

# Define the pins for Step and Direction for both motors
motor1_step = 17   # BCM 8 for Motor 1 Step
motor1_dir = 4    # BCM 7 for Motor 1 Direction
motor2_step = 27 # BCM 18 for Motor 2 Step
motor2_dir = 22   # BCM 23 for Motor 2 Direction

# Setup GPIO
GPIO.setmode(GPIO.BCM)  # Broadcom pin-numbering scheme
GPIO.setup([motor1_step, motor1_dir, motor2_step, motor2_dir], GPIO.OUT)

def step_motors(steps, direction='cw', step_delay=0.003):
    """
    Function to perform steps simultaneously on both motors.
    - steps: number of steps to perform
    - direction: 'cw' for clockwise, 'ccw' for counterclockwise
    - step_delay: delay between each step in seconds
    """
    # Set direction for both motors
    GPIO.output(motor1_dir, GPIO.HIGH if direction == 'cw' else GPIO.LOW)
    GPIO.output(motor2_dir, GPIO.HIGH if direction == 'cw' else GPIO.LOW)

    # Perform steps simultaneously
    for _ in range(steps):
        GPIO.output([motor1_step, motor2_step], GPIO.HIGH)
        time.sleep(step_delay)
        GPIO.output([motor1_step, motor2_step], GPIO.LOW)
        time.sleep(step_delay)

try:
    # Testing motor movements
    print("Moving both motors clockwise simultaneously")
    step_motors(100, 'cw')
    print("Moving both motors counterclockwise simultaneously")
    step_motors(100, 'ccw')

finally:
    # Clean up GPIO settings
    GPIO.cleanup()
    print("GPIO cleanup complete. Test finished.")
