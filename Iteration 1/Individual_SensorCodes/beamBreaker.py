import Jetson.GPIO as GPIO
import time

# Use Broadcom pin numbering
GPIO.setmode(GPIO.TEGRA_SOC)

# Pin configuration
beam_pin = 'SPI1_CS1'  # Set to the BCM number of the pin connected to the IR beam breaker

# Set up the GPIO pin as an input
# Assuming the IR sensor pulls the pin LOW when the beam is broken
GPIO.setup(beam_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

def beam_broken():
    return not GPIO.input(beam_pin)

try:
    print("Starting IR Beam Breaker Test. Press CTRL+C to exit.")
    while True:
        if beam_broken():
            print("Beam is broken!")
        else:
            print("Beam is intact.")
        
        time.sleep(1)  # Check the beam every second

except KeyboardInterrupt:
    print("Exiting program")

finally:
    GPIO.cleanup()  # Clean up GPIO on normal exit
