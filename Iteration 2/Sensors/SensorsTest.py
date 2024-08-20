import time
import board
import busio
from adafruit_vl6180x import VL6180X
from adafruit_tca9548a import TCA9548A

# Constants
#I2C_BUS = 1
#TF_LUNA_ADDRESS = 0x10
#DISTANCE_REGISTER = 0x00

# Setup the main I2C bus
i2c_bus = busio.I2C(board.SCL, board.SDA)

# Initialize the TCA9548A I2C multiplexer
tca = TCA9548A(i2c_bus)

# Mapping of sensors to multiplexer channels
channels = {
    'LeftWallFront': tca[0],
    'LeftWallRear': tca[7],
    'LeftPlant1': tca[2],
    'LeftPlant2': tca[1],
    'RightWallFront': tca[3],
    'RightWallRear': tca[6],
    'RightPlant1': tca[5],
    'RightPlant2': tca[4]
}

# Separate bus instance for TF Luna (if on a different bus or requires isolation)
#tf_luna_bus = busio.I2C(board.SCL, board.SDA)  # Adjust pins if necessary

def read_tf_luna():
    """ Read distance from the TF Luna sensor. """
    while not tf_luna_bus.try_lock():
        pass
    try:
        data = tf_luna_bus.read_i2c_block_data(TF_LUNA_ADDRESS, DISTANCE_REGISTER, 2)
        return data[0] << 8 | data[1]
    finally:
        tf_luna_bus.unlock()

def read_vl6180(sensor_channel):
    """ Read distance from a VL6180 sensor on the specified multiplexer channel """
    vl6180 = VL6180X(sensor_channel)
    return vl6180.range

def main():
    try:
        while True:
            # Read from all VL6180 sensors
            for name, sensor_channel in channels.items():
                distance = read_vl6180(sensor_channel)
                print(f"Distance from {name}: {distance} mm")
            
            # Read from TF Luna
            #tf_luna_distance = read_tf_luna()
            #print(f"Distance from TF Luna: {tf_luna_distance} mm")
            
            time.sleep(0.1)  # Short delay to prevent excessive sensor polling
    finally:
        print("Finished reading all sensors.")

if __name__ == "__main__":
    main()
