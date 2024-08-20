import smbus2
import time

# Constants for TF-Luna
I2C_ADDRESS = 0x11
DISTANCE_REGISTER = 0x00  # Check this from the datasheet
BUS_NUMBER = 1  # Typically 1 on Raspberry Pi and Jetson Nano

# Create an instance of the I2C bus
bus = smbus2.SMBus(BUS_NUMBER)

def read_distance():
    # Read two bytes from the distance register
    data = bus.read_i2c_block_data(I2C_ADDRESS, DISTANCE_REGISTER, 2)
    # Combine the two bytes into a single 16-bit value
    distance = data[0] + (data[1] << 8)
    return distance

try:
    while True:
        distance = read_distance()
        print(f"Distance: {distance} cm")
        time.sleep(1)  # Read every second
except KeyboardInterrupt:
    print("Measurement stopped by user")
    bus.close()
except Exception as e:
    print(f"An error occurred: {e}")
    bus.close()
