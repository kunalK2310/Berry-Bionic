import serial
import time

## Setup serial connection for Arduino 
ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1) # Check the port used on Jetson Nano/Rpi 4 and baudrate
ser.flush()

## Wait for the robot to stop

def wait_for_stop():
	while True:
		if ser.in_waiting>0:
			line = ser.readline().decode('utf-8').rstrip()
			if line == 'state : 1':  # Check if this picks up
				break

## Capture and process images
#run predict_v5.2_cv2.py
