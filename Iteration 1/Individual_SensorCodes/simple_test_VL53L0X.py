import time
import board
import busio
import adafruit_vl53l0x

# Create I2C bus
i2c = busio.I2C(board.SCL, board.SDA)

# Create a VL53L0X object
vl53 = adafruit_vl53l0x.VL53L0X(i2c)

def main():
    while True:
        distance = vl53.range
        print("Distance: {0}mm".format(distance))
        time.sleep(1)  # Delay for a second and repeat

if __name__ == "__main__":
    main()
