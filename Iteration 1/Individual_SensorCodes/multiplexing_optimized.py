import Jetson.GPIO as GPIO
import time
import board
import busio
from adafruit_vl6180x import VL6180X

# Set GPIO mode to use the Tegra SoC names
GPIO.setmode(GPIO.TEGRA_SOC)

def setup_gpio():
    # Updated pin names based on your setup; verify these are correct and usable for GPIO
    control_pins = ['SPI1_CS0', 'SPI1_SCK', 'SPI2_MISO', 'SPI1_MISO']  
    for pin in control_pins:
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, GPIO.LOW)
    return control_pins

def set_channel(control_pins, channel):
    """ Set the multiplexer channel by converting the channel number into binary and setting pins """
    binary_channel = bin(channel)[2:].zfill(4)  # Get binary string of the channel, fill to 4 bits
    for i, pin in enumerate(control_pins):
        GPIO.output(pin, GPIO.HIGH if binary_channel[::-1][i] == '1' else GPIO.LOW)  # Reverse binary to match pin order

def read_sensor(control_pins, i2c, channel):
    """ Select channel, initialize sensor, and print distance """
    set_channel(control_pins, channel)
    sensor = VL6180X(i2c)  # Initialize sensor on selected I2C channel
    distance = sensor.range  # Get distance measurement
    print(f"Channel {channel} Distance: {distance} mm")

def main():
    i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)  # Increased I2C speed for faster reads
    control_pins = setup_gpio()  # Setup control pins for the multiplexer
    try:
        while True:
            read_sensor(control_pins, i2c, 14)  # Read from sensor connected to multiplexer channel 14
            time.sleep(0.1)  # Reduced sleep for faster updates
            read_sensor(control_pins, i2c, 15)  # Read from sensor connected to multiplexer channel 15
            time.sleep(0.1)  # Reduced sleep for faster updates
    finally:
        GPIO.cleanup()  # Clean up GPIO resources on exit

if __name__ == "__main__":
    main()
