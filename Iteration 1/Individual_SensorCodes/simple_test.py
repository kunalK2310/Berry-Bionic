import time
import board
import busio
from adafruit_vl6180x import VL6180X

i2c = busio.I2C(board.SCL, board.SDA)  # Setup I2C
vl6180_sensor = VL6180X(i2c)  # Create an instance of the VL6180X

while True:
    distance = vl6180_sensor.range
    print("Distance: {}mm".format(distance))
    time.sleep(1)
