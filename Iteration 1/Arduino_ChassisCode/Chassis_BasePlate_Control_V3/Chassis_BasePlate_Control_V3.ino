#include <AccelStepper.h>
#include <Adafruit_PWMServoDriver.h>

// Define pin connections for GearMotor (Base Rotation)
const int dirPin = 24; 
const int stepPin = 25; 
const int sleepPin = 11;
const int enablePin = 10; 

// Define pin connections for SideMotor1 and SideMotor2 (Up/Down Control)
const int dirPin1 = 28; 
const int stepPin1 = 29; 
const int sleep1 = 7; 
const int enable1 = 6;
const int dirPin2 = 22; 
const int stepPin2 = 23; 
const int sleep2 = 3;
const int enable2 = 2;

AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void setup() {
  Serial.begin(115200);
  pwm.begin();
  pwm.setPWMFreq(60); // Analog servos run at ~60 Hz updates
  setupMotors();
}

void setupMotors() {
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(sleepPin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  pinMode(stepPin1, OUTPUT);
  pinMode(sleep1, OUTPUT);
  pinMode(enable1, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(stepPin2, OUTPUT);
  pinMode(sleep2, OUTPUT);
  pinMode(enable2, OUTPUT);
  digitalWrite(enablePin, LOW);
  digitalWrite(sleepPin, HIGH);
  digitalWrite(enable1, LOW);
  digitalWrite(sleep1, HIGH);
  digitalWrite(enable2, LOW);
  digitalWrite(sleep2, HIGH);
  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(500);
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    handleCommand(command);
  }
}

void handleCommand(String command) {
  if (command.startsWith("angle:")) {
    float angle = command.substring(6).toFloat();
    rotateBase(angle);
    performTrimming();
  } else if (command == "reset") {
    resetBasePosition();
  }
}

void rotateBase(float angle) {
  const float gearRatio = 33.5;
  const int stepsPerRevolution = 200;
  long steps = round((stepsPerRevolution / 360.0) * gearRatio * angle);
  stepper.move(steps);
  stepper.runToPosition();
}

void performTrimming() {
  // Servo movements to trim the plant
    for (int pos_m=90; pos_m<115; pos_m +=10) {	
      pwm.setPWM(0, 0, pos_m );
      Serial.println("230 pulse length count --> 90 degrees");
      delay(50);
    }    

    for (int pos_m=260; pos_m<293; pos_m +=10) {	
      pwm.setPWM(1, 0, pos_m );
      Serial.println("230 pulse length count --> 90 degrees");
      delay(50);
    }    
    for (int pos_m=115; pos_m<140; pos_m +=10) {	
      pwm.setPWM(0, 0, pos_m );
      Serial.println("230 pulse length count --> 90 degrees");
      delay(50);
    }    

    for (int pos_m=293; pos_m<326; pos_m +=10) {	
      pwm.setPWM(1, 0, pos_m );
      Serial.println("230 pulse length count --> 90 degrees");
      delay(50);
    }    
    for (int pos_m=140; pos_m<165; pos_m +=10) {	
      pwm.setPWM(0, 0, pos_m );
      Serial.println("230 pulse length count --> 90 degrees");
      delay(50);
    }    

    for (int pos_m=326; pos_m<359; pos_m +=10) {	
      pwm.setPWM(1, 0, pos_m );
      Serial.println("230 pulse length count --> 90 degrees");
      delay(50);
    }    
    for (int pos_m=165; pos_m<190; pos_m +=10) {	
      pwm.setPWM(0, 0, pos_m );
      Serial.println("230 pulse length count --> 90 degrees");
      delay(50);
    }    

    for (int pos_m=359; pos_m<392; pos_m +=10) {	
      pwm.setPWM(1, 0, pos_m );
      Serial.println("230 pulse length count --> 90 degrees");
      delay(50);
    }    
    for (int pos_m=190; pos_m<215; pos_m +=10) {	
      pwm.setPWM(0, 0, pos_m );
      Serial.println("230 pulse length count --> 90 degrees");
      delay(50);
    }    

    for (int pos_m=392; pos_m<425; pos_m +=10) {	
      pwm.setPWM(1, 0, pos_m );
      Serial.println("230 pulse length count --> 90 degrees");
      delay(50);
    }    
    for (int pos_m=215; pos_m<240; pos_m +=10) {	
      pwm.setPWM(0, 0, pos_m );
      Serial.println("230 pulse length count --> 90 degrees");
      delay(50);
    }    

    for (int pos_m=425; pos_m<458; pos_m +=10) {	
      pwm.setPWM(1, 0, pos_m );
      Serial.println("230 pulse length count --> 90 degrees");
      delay(50);
    }    
    for (int pos_m=240; pos_m<265; pos_m +=10) {	
      pwm.setPWM(0, 0, pos_m );
      Serial.println("230 pulse length count --> 90 degrees");
      delay(50);
    }    

    for (int pos_m=458; pos_m<491; pos_m +=10) {	
      pwm.setPWM(1, 0, pos_m );
      Serial.println("230 pulse length count --> 90 degrees");
      delay(50);
    }    
    for (int pos_m=265; pos_m<290; pos_m +=10) {	
      pwm.setPWM(0, 0, pos_m );
      Serial.println("230 pulse length count --> 90 degrees");
      delay(50);
    }    

    for (int pos_m=491; pos_m<524; pos_m +=10) {	
      pwm.setPWM(1, 0, pos_m );
      Serial.println("230 pulse length count --> 90 degrees");
      delay(50);
    }    
    for (int pos_m=290; pos_m<320; pos_m +=10) {	
      pwm.setPWM(0, 0, pos_m );
      Serial.println("230 pulse length count --> 90 degrees");
      delay(50);
    }    

    for (int pos_m=524; pos_m<565; pos_m +=10) {	
      pwm.setPWM(1, 0, pos_m );
      Serial.println("230 pulse length count --> 90 degrees");
      Serial.println("motion 2 completed");
      delay(50);
    }    

    for (int pos=320; pos>280; pos -=10) {		
      pwm.setPWM(0, 0, pos );
      Serial.println("230 pulse length count --> 90 degrees");
      delay(50);
    }
    
    for (int pos=565; pos>505; pos -=10) {		 
      pwm.setPWM(1, 0, pos );
      Serial.println("230 pulse length count --> 90 degrees");
      delay(50);
    }

    for (int pos=280; pos>220; pos -=10) {		
      pwm.setPWM(0, 0, pos );
      Serial.println("230 pulse length count --> 90 degrees");
      delay(50);
    }
    
    for (int pos=505; pos>440; pos -=10) {		 
      pwm.setPWM(1, 0, pos );
      Serial.println("230 pulse length count --> 90 degrees");
      delay(50);
    }

    for (int pos=220; pos>160; pos -=10) {		
      pwm.setPWM(0, 0, pos );
      Serial.println("230 pulse length count --> 90 degrees");
      delay(50);
    }
    
    for (int pos=440; pos>350; pos -=10) {		 
      pwm.setPWM(1, 0, pos );
      Serial.println("230 pulse length count --> 90 degrees");
      delay(50);
    }
     
    for (int pos=160; pos>90; pos -=10) {		
      pwm.setPWM(0, 0, pos );
      Serial.println("230 pulse length count --> 90 degrees");
      delay(50);
    }
    
    for (int pos=350; pos>260; pos -=10) {		 
      pwm.setPWM(1, 0, pos );
      Serial.println("230 pulse length count --> 90 degrees");
      Serial.println("motion 3 completed");
      delay(50);  
    }
}

void resetBasePosition() {
  stepper.move(-stepper.currentPosition());
  stepper.runToPosition();
}
