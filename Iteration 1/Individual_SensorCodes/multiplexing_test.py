import Jetson.GPIO as GPIO
import time
import busio
import board
from adafruit_vl6180x import VL6180X

# Set GPIO mode to use the Tegra SoC names
GPIO.setmode(GPIO.TEGRA_SOC)

def setup_gpio():
    # Update these pin names to TEGRA_SOC names corresponding to pins 24, 23, 22, 21 on your setup
    control_pins = ['SPI1_CS0', 'SPI1_SCK', 'SPI2_MISO', 'SPI1_MISO']  # Example: SOC_GPIOXX are placeholders
    for pin in control_pins:
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, GPIO.LOW)
    return control_pins

def set_channel(control_pins, channel):
    for i in range(4):
        GPIO.output(control_pins[i], GPIO.HIGH if channel & (1 << i) else GPIO.LOW)

def read_sensor(control_pins, i2c, channel):
    set_channel(control_pins, channel)
    sensor = VL6180X(i2c)
    distance = sensor.range
    print(f"Channel {channel} Distance: {distance} mm")

def main():
    i2c = busio.I2C(board.SCL, board.SDA,frequency=400000)  # Setup I2C
    control_pins = setup_gpio()
    try:
        while True:
            read_sensor(control_pins, i2c, 14)  # Adjust channel numbers as necessary
            time.sleep(0.5)
            read_sensor(control_pins, i2c, 15)
            time.sleep(0.5)
    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    main()
