import time
import board
import busio
import Jetson.GPIO as GPIO
from adafruit_vl6180x import VL6180X
from adafruit_vl53l0x import VL53L0X

# Initialize I2C bus
i2c = busio.I2C(board.SCL, board.SDA)

# Set GPIO mode to use the Tegra SOC names
GPIO.setmode(GPIO.TEGRA_SOC)

# Define XSHUT pin for VL53L0X using TEGRA_SOC name
xshut_vl53l0x = 'SPI1_CS1'  # Replace with the actual TEGRA_SOC name
GPIO.setup(xshut_vl53l0x, GPIO.OUT, initial=GPIO.LOW)


# Reset VL53L0X
GPIO.output(xshut_vl53l0x, GPIO.LOW)
time.sleep(0.1)  # Ensure it's reset
GPIO.output(xshut_vl53l0x, GPIO.HIGH)
time.sleep(0.1)  # Wait for the sensor to wake up

# Initialize VL53L0X with new I2C address
vl53l0x = VL53L0X(i2c)
vl53l0x.set_address(0x30)  # Set a new address before using the sensor
time.sleep(0.01)

# Initialize VL6180X without controlling XSHUT
vl6180 = VL6180X(i2c)

def read_sensors():
    while True:
        distance_vl6180 = vl6180.range
        distance_vl53l0x = vl53l0x.range
        print(f"VL6180X Distance: {distance_vl6180} mm")
        print(f"VL53L0X Distance: {distance_vl53l0x} mm")
        time.sleep(1)

if __name__ == "__main__":
    read_sensors()
