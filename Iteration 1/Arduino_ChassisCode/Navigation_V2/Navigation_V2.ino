// Pin configuration for stepper motor
const int stepPins[4] = {2, 3, 4, 12}; // Pins for stepping (X, Y, Z, A motors)
const int dirPins[4] = {5, 6, 7, 13}; // Pins for direction (X, Y, Z, A motors)

// Define motor indices for clarity
enum Motors {
  X_MOTOR, A_MOTOR, Y_MOTOR, Z_MOTOR
};

void setup() {
  Serial.begin(9600); // Start serial communication at 9600 baud rate
  for (int i = 0; i < 4; i++) {
    pinMode(stepPins[i], OUTPUT);
    pinMode(dirPins[i], OUTPUT);
  }
}

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    if (command == "STOP") {
      stopMotors();
    } else if (command.startsWith("MOVE ")) {
      moveMotors(command.substring(5).toInt());
    }
  }
}

void performSteps(int steps, bool directions[]) {
  for (int i = 0; i < steps; i++) { 
    for (int j = 0; j < 4; j++) {
      digitalWrite(dirPins[j], directions[j]); // Set each motor's direction
      digitalWrite(stepPins[j], HIGH); // Step HIGH
    }
    delayMicroseconds(2000); // Shorter pulse for quicker response
    for (int j = 0; j < 4; j++) {
      digitalWrite(stepPins[j], LOW); // Step LOW
    }
    delayMicroseconds(2000); // Shorter pulse for quicker response
  }
}

void moveMotors(int pidOutput) {
  bool direction = pidOutput >= 0;
  int steps = abs(pidOutput);
  bool directions[4];

  for (int i = 0; i < 4; i++) {
    directions[i] = determineMotorDirection(i, direction);
  }
  performSteps(steps, directions);
}

bool determineMotorDirection(int motorIndex, bool defaultDirection) {
  // Custom logic for direction based on motor index
  // For example, in a vehicle, opposite sides may need to move in opposite directions to turn
  switch (motorIndex) {
    case X_MOTOR: // Assuming this is the front-right motor
    case A_MOTOR: // Assuming this is the rear-right motor
      return defaultDirection;
    case Y_MOTOR: // Assuming this is the front-left motor
    case Z_MOTOR: // Assuming this is the rear-left motor
      return !defaultDirection;
  }
  return defaultDirection;
}

void stopMotors() {
  for (int i = 0; i < 4; i++) { // Stop all motors
    digitalWrite(stepPins[i], LOW);
  }
}
