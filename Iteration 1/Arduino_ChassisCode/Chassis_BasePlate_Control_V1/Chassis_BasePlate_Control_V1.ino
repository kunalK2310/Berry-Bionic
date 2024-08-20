#include <AccelStepper.h>

// Define pin connections
// GearMotor (Base Rotation)
const int dirPin = 24; 
const int stepPin = 25; 
const int sleepPin = 11;
const int enablePin = 10; 

// SideMotor1 (Up/Down Control)
const int dirPin1 = 28; 
const int stepPin1 = 29; 
const int sleep1 = 7; 
const int enable1 = 6;

// SideMotor2 (Up/Down Control)
const int dirPin2 = 22; 
const int stepPin2 = 23; 
const int sleep2 = 3;
const int enable2 = 2;

const int stepsPlatform = 10000; // Adjust as necessary for maximum elevation of Platform
const int stepDelay = 500; // Microseconds

// Initialize the stepper library on the pins for the GearMotor
AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin);

void setup() {
  Serial.begin(9600);

  // Setup GearMotor
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(sleepPin, OUTPUT);
  pinMode(enablePin, OUTPUT);

  // Setup SideMotor1
  pinMode(dirPin1, OUTPUT);
  pinMode(stepPin1, OUTPUT);
  pinMode(sleep1, OUTPUT);
  pinMode(enable1, OUTPUT);

  // Setup SideMotor2
  pinMode(dirPin2, OUTPUT);
  pinMode(stepPin2, OUTPUT);
  pinMode(sleep2, OUTPUT);
  pinMode(enable2, OUTPUT);

  // Set initial motor states
  digitalWrite(enablePin, LOW);
  digitalWrite(sleepPin, HIGH);
  digitalWrite(enable1, LOW);
  digitalWrite(sleep1, HIGH);
  digitalWrite(enable2, LOW);
  digitalWrite(sleep2, HIGH);

  // Configure the stepper motor used for the base rotation
  stepper.setMaxSpeed(1000); // Adjust max speed as needed
  stepper.setAcceleration(500); // Adjust acceleration as needed
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    handleCommand(command);
  }
}

void handleCommand(String command) {
  if (command == "up") {
    moveChassis(true);
  } else if (command == "down") {
    moveChassis(false);
  } else if (command.startsWith("rotate:")) {
    rotateBase(command.substring(7).toFloat());
  }
}

void moveChassis(bool goingUp) {
  int dir = goingUp ? LOW : HIGH; // Determine direction based on command
  digitalWrite(dirPin1, dir);
  digitalWrite(dirPin2, dir);
  
  for(int x = 0; x < stepsPlatform; x++) {
    digitalWrite(stepPin1, HIGH);
    digitalWrite(stepPin2, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(stepPin1, LOW);
    digitalWrite(stepPin2, LOW);
    delayMicroseconds(stepDelay);
  }
}

void rotateBase(float angle) {
    const float gearRatio = 33.5;  // Gear ratio of smaller to bigger gear
    const int stepsPerRevolution = 200;  // Total steps for one full revolution of the motor

    // Calculate steps needed for the desired angle on the bigger gear, incorporating gear ratio
    long steps = round((stepsPerRevolution / 360.0) * gearRatio * angle);

    // Move the stepper motor to the calculated position
    stepper.move(steps);
    stepper.runToPosition();
}

