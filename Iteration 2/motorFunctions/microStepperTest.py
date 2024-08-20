import Jetson.GPIO as GPIO
import time

# Define the pins for Step and Direction
motor_step = 11  # BCM 11 for Motor Step
motor_dir = 9    # BCM 9 for Motor Direction

# Setup GPIO
GPIO.setmode(GPIO.BCM)  # Broadcom pin-numbering scheme
GPIO.setup([motor_step, motor_dir], GPIO.OUT)

def step_motor(steps, direction='cw', step_delay=0.0015):
    """
    Function to perform steps on the motor.
    - steps: number of steps to perform
    - direction: 'cw' for clockwise, 'ccw' for counterclockwise
    - step_delay: delay between each step in seconds
    """
    # Set direction
    GPIO.output(motor_dir, GPIO.HIGH if direction == 'cw' else GPIO.LOW)

    # Perform steps
    for _ in range(steps):
        GPIO.output(motor_step, GPIO.HIGH)
        time.sleep(step_delay)
        GPIO.output(motor_step, GPIO.LOW)
        time.sleep(step_delay)

try:
    print("Operating motor")
    # Example usage of the function
    step_motor(100, 'cw', 0.0015)  # Move motor clockwise for 100 steps
    step_motor(100, 'ccw', 0.0015)  # Move motor counterclockwise for 100 steps

finally:
    # Clean up GPIO settings
    GPIO.cleanup()
    print("GPIO cleanup complete. Test finished.")
