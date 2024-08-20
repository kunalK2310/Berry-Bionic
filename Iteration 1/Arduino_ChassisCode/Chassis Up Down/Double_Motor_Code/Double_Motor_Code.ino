//GearMotor
const int dirPin = 24; 
const int stepPin = 25; 
const int sleep = 11;
const int enable = 10; // enable pin must be set low , check with multimeter

          
//SideMotor1
const int dirPin2 = 28; 
const int stepPin2 = 29; // connect sleep to reset both need to be set high
const int sleep2 = 7; //Set if needed
const int enable2 = 6; //Set if needed

//SideMotor2
const int dirPin3 = 22; 
const int stepPin3 = 23; 
const int sleep3 = 3;
const int enable3 = 2; // enable pin must be set low , check with multimeter



//parameters
const int stepsPerRevolution = 11000; //2400
const int stepDelay=500;





void setup() {

  // //motor 1
  // pinMode(dirPin, OUTPUT);
  // pinMode(stepPin, OUTPUT);
  // pinMode(sleep, OUTPUT);
  // pinMode(enable, OUTPUT);

  //motor 2
  pinMode(dirPin2, OUTPUT);
  pinMode(stepPin2, OUTPUT);
  pinMode(sleep2, OUTPUT);
  pinMode(enable2, OUTPUT);

  //motor3
  pinMode(dirPin3, OUTPUT);
  pinMode(stepPin3, OUTPUT);
  pinMode(sleep3, OUTPUT);
  pinMode(enable3, OUTPUT);


  digitalWrite(enable, LOW);
  digitalWrite(sleep, HIGH);
  digitalWrite(enable2, LOW);
  digitalWrite(sleep2, HIGH);
  digitalWrite(enable3, LOW);
  digitalWrite(sleep3, HIGH);




}

void loop() {
  up();
  delay(3000);
  down();
  delay(3000);
  //approachTape(); // toggle between releaseGripper and driveGripper
  

}

void up()
{
  //clockwise
  digitalWrite(dirPin, LOW);
  digitalWrite(dirPin2, LOW);
  digitalWrite(dirPin3, LOW);
  // Spin motor
  for(int x = 0; x < stepsPerRevolution; x++)
  {
    digitalWrite(stepPin, HIGH);
    digitalWrite(stepPin2, HIGH);
    digitalWrite(stepPin3, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(stepPin, LOW);
    digitalWrite(stepPin2, LOW);
    digitalWrite(stepPin3, LOW);
    delayMicroseconds(stepDelay);
  }
}

void down()

{
  //clockwise
  digitalWrite(dirPin, HIGH);
  digitalWrite(dirPin2, HIGH);
  digitalWrite(dirPin3, HIGH);
  // Spin motor
  for(int x = 0; x < stepsPerRevolution; x++)
  {
    digitalWrite(stepPin, HIGH);
    digitalWrite(stepPin2, HIGH);
    digitalWrite(stepPin3, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(stepPin, LOW);
    digitalWrite(stepPin2, LOW);
    digitalWrite(stepPin3, LOW);
    delayMicroseconds(stepDelay);
  }
}