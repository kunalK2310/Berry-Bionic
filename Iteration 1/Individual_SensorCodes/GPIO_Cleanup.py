import Jetson.GPIO as GPIO

def setup():
    try:
        GPIO.setmode(GPIO.BCM)
        print("No problem")
    except ValueError:
        GPIO.cleanup()
        print("Cleaning up...")
        GPIO.setmode(GPIO.BCM)
    # Now set up your GPIO channels

def main():
    setup()
    # Your main application code here

if __name__ == "__main__":
    main()
