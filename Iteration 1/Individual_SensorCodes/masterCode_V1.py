import serial
import time

# Connect to Arduino
ser = serial.Serial('/dev/ttyACM0', 9600)  # Check the correct port

def move_chassis(direction):
    if direction == "up":
        ser.write(b"up\n")
    elif direction == "down":
        ser.write(b"down\n")

def rotate_base(angle):
    command = f"rotate:{angle}\n"
    ser.write(command.encode())

def main():
    time.sleep(3)  # Allow some time for the Arduino to reset
    print("Type 'exit' to quit the program.")
    while True:
        angle = input("Enter rotation angle in degrees or 'exit' to quit: ").strip()
        if angle.lower() == 'exit':
            print("Exiting program.")
            break
        try:
            float_angle = float(angle)  # Attempt to convert to float
            rotate_base(float_angle)
            print(f"Rotating plate {angle} degrees")
        except ValueError:
            print("Invalid input. Please enter a numeric value or 'exit'.")


    # Ensure to close the serial connection when done
    ser.close()

if __name__ == "__main__":
    main()