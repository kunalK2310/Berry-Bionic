// Pin configuration for stepper motor
const int stepPins[4] = {2, 3, 4, 12}; // Pins for stepping (X, Y, Z, A motors)
const int dirPins[4] = {5, 6, 7, 13}; // Pins for direction (X, Y, Z, A motors)

// Motor indices for easier understanding
const int RIGHT_REAR = 0;
const int RIGHT_FRONT = 1;
const int LEFT_FRONT = 2;
const int LEFT_REAR = 3;

// Movement control variables
const int motorSpeed = 4000; // Speed of motors in microseconds between steps

void setup() {
  Serial.begin(9600); // Start serial communication at 9600 baud rate
  for (int i = 0; i < 4; i++) {
    pinMode(stepPins[i], OUTPUT);
    pinMode(dirPins[i], OUTPUT);
  }
}

void loop() {
  static String incomingData = "";  // Buffer to hold incoming serial data
  if (Serial.available() > 0) {
    char received = Serial.read();
    if (received == '\n') {
      if (incomingData == "STOP") {
        stopMotors();
      } else {
        int pidOutput = incomingData.toInt();  // Convert the accumulated string to an integer
        adjustMotors(pidOutput);
      }
      incomingData = "";  // Clear the buffer
    } else {
      incomingData += received;  // Append received character to buffer
    }
  }
}

void performSteps(int steps, int motorIndex) {
  for (int i = 0; i < steps; i++) {
    digitalWrite(stepPins[motorIndex], HIGH);
    delayMicroseconds(motorSpeed);
    digitalWrite(stepPins[motorIndex], LOW);
    delayMicroseconds(motorSpeed);
  }
}

void moveForward() {
  digitalWrite(dirPins[RIGHT_REAR], LOW);
  digitalWrite(dirPins[RIGHT_FRONT], LOW);
  digitalWrite(dirPins[LEFT_FRONT], HIGH);
  digitalWrite(dirPins[LEFT_REAR], HIGH);

  for (int i = 0; i < 4; i++) {
    performSteps(10, i);  // Assuming a default step count for moving forward
  }
}

void adjustMotors(int pidOutput) {
  bool direction = pidOutput > 0;
  digitalWrite(dirPins[LEFT_FRONT], !direction);
  digitalWrite(dirPins[LEFT_REAR], !direction);
  digitalWrite(dirPins[RIGHT_FRONT], direction);
  digitalWrite(dirPins[RIGHT_REAR], direction);

  int leftSteps = direction ? abs(pidOutput) : abs(pidOutput) * 0.8;  // Inner wheel moves less
  int rightSteps = direction ? abs(pidOutput) * 0.8 : abs(pidOutput);

  for (int i = 0; i < 4; i++) {
    int steps = (i == LEFT_FRONT || i == LEFT_REAR) ? leftSteps : rightSteps;
    performSteps(steps, i);
  }
}

void stopMotors() {
  for (int i = 0; i < 4; i++) { // Stop all motors
    digitalWrite(stepPins[i], LOW);
  }
}
