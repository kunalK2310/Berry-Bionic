import serial
import time

# Connect to Arduino
ser = serial.Serial('/dev/ttyACM3', 9600)  # Check the correct port

def move_chassis(direction):
    command = f"{direction}\n"
    ser.write(command.encode())
    print(f"Moving chassis {direction}")

def rotate_base(angle):
    command = f"rotate:{angle}\n"
    ser.write(command.encode())
    print(f"Rotating base {angle} degrees")

def perform_trimming():
    ser.write(b"trim\n")
    print("Performing trimming action")

def reset_base():
    ser.write(b"reset\n")
    print("Resetting base position")

def main():
    time.sleep(3)  # Allow some time for the Arduino to reset
    print("Type 'exit' to quit the program.")
    while True:
        command = input("Enter command ('rotate', 'up', 'down', 'trim', 'reset', or 'exit'): ").strip().lower()
        if command == 'exit':
            print("Exiting program.")
            break
        elif command in ['up', 'down']:
            move_chassis(command)
        elif command == 'rotate':
            angle = input("Enter rotation angle in degrees: ").strip()
            try:
                float_angle = float(angle)
                rotate_base(float_angle)
            except ValueError:
                print("Invalid input. Please enter a numeric value.")
        elif command == 'trim':
            perform_trimming()
        elif command == 'reset':
            reset_base()
        else:
            print("Invalid command. Please enter 'rotate', 'up', 'down', 'trim', 'reset', or 'exit'.")

    # Ensure to close the serial connection when done
    ser.close()

if __name__ == "__main__":
    main()
