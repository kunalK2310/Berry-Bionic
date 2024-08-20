import time
import board
import busio
from adafruit_vl6180x import VL6180X
from adafruit_vcnl4010 import VCNL4010

# Initialize I2C bus
i2c = busio.I2C(board.SCL, board.SDA)

# Create sensor instances
vl6180_sensor = VL6180X(i2c)
vcnl4010_sensor = VCNL4010(i2c)

def read_sensors():
    # Read the range in millimeters from VL6180
    distance_vl6180 = vl6180_sensor.range
    print(f"VL6180 Distance: {distance_vl6180} mm")

    # Read the proximity from VCNL4010 (note: proximity is not the same as precise distance in mm)
    proximity_vcnl4010 = vcnl4010_sensor.proximity
    print(f"VCNL4010 Proximity: {proximity_vcnl4010}")

def main():
    while True:
        read_sensors()
        time.sleep(1)  # Delay between readings to avoid overwhelming the sensors

if __name__ == "__main__":
    main()
