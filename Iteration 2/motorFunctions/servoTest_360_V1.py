import time
from adafruit_motor import motor

# Assuming `continuous_servo` is your continuous rotation servo object
def rotate_servo(angle, gear_ratio):
    neutral_pulse = 1500
    rotate_pulse = 1600  # Clockwise
    if angle < 0:
        rotate_pulse = 1400  # Counterclockwise
        angle = -angle
    
    rotation_time = calculate_time_for_angle(angle, gear_ratio)
    continuous_servo.throttle = rotate_pulse
    time.sleep(rotation_time)
    continuous_servo.throttle = neutral_pulse  # Stop the servo

def calculate_time_for_angle(angle, gear_ratio):
    # Hypothetical function to calculate time based on speed tests
    base_time_per_degree = 0.02  # Time per degree at a specific pulse width
    return (angle / gear_ratio) * base_time_per_degree

# Example of rotating the gear
rotate_servo(30, 6)   # Rotate the gear by 30 degrees
rotate_servo(90, 6)   # Rotate the gear by 90 degrees
rotate_servo(270, 6)  # Rotate the gear by 270 degrees
rotate_servo(-390, 6) # Rotate back to the initial position
