import time
import serial
import board
import busio
import adafruit_vl6180x
from simple_pid import PID

# Constants
TARGET_DISTANCE = 130  # Target distance in mm 
# Create I2C bus.
i2c = busio.I2C(board.SCL, board.SDA)  # Create I2C bus
SERIAL_PORT = '/dev/ttyACM3'  # Serial port to communicate with Arduino
BAUD_RATE = 9600  # Communication baud rate

# Setup serial communication with Arduino
arduino = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

# Create sensor instance
sensor = adafruit_vl6180x.VL6180X(i2c)

# PID Setup
pid = PID(1.0, 0.1, 0.05, setpoint=TARGET_DISTANCE)

def get_distance():
    # Reads the sensor and returns the distance
    return sensor.range

def send_command_to_arduino(command):
    # Sends commands to the Arduino via serial
    arduino.write(f"{command}\n".encode())

def main():
    try:
        while True:
            # Get distance from the sensor
            current_distance = get_distance()
            print(f"Current Distance: {current_distance} mm")

            # PID calculation
            correction = pid(current_distance)
            print(f"PID Correction: {correction}")

            # Decide on the command based on PID output
            if abs(correction) < 5:  # If correction is minimal, send a gentle move command
                send_command_to_arduino("MOVE 10")  # Move forward with small steps
            else:
                send_command_to_arduino(f"MOVE {int(correction)}")  # Adjust motor speed/direction

            time.sleep(0.1)  # Short delay between commands

    except KeyboardInterrupt:
        print("Program stopped manually")
        send_command_to_arduino("STOP")  # Ensure motors are stopped when program is terminated

    finally:
        arduino.close()  # Close serial port

if __name__ == "__main__":
    main()
