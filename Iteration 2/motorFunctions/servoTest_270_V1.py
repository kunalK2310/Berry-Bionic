import time
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

# Set up the I2C bus and PCA9685
i2c_bus = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c_bus)
pca.frequency = 50

# Initialize the servo on channel 12 with specific pulse width range
servo_270 = servo.Servo(pca.channels[12], min_pulse=500, max_pulse=2500)

try:
    # Move servo to minimum angle (approx. 0 degrees)
    print("Moving to minimum angle...")
    servo_270.angle = 0
    time.sleep(2)  # Wait for 2 seconds

    # Move servo to maximum angle (approx. 180 degrees)
    print("Moving to maximum angle...")
    servo_270.angle = 180
    time.sleep(2)  # Wait for 2 seconds

finally:
    # Ensure the PCA9685 is properly shut down
    pca.deinit()
    print("Test complete and PCA9685 shutdown.")
