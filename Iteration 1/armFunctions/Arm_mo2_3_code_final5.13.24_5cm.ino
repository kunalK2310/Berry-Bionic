#include <AccelStepper.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  100                           // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  500                           // This is the 'maximum' pulse length count (out of 4096)
#define dirPin  24
#define stepPin 25
// Define the number of steps per revolution
#define STEPS_PER_REVOLUTION 200
float x=0;

AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin);

void setup() {
  Serial.begin(9600);
  //Serial.println("Reference check of 120 degrees with respect to the pulse length count");

  pwm.begin();
  pwm.setPWMFreq(50);                         // Analog servos run at ~50 Hz updates
  // Set up the stepper motor properties
  stepper.setMaxSpeed(1000); // Adjust max speed as needed
  stepper.setAcceleration(500); // Adjust acceleration as needed
}

void loop() {
  if(x==360){
    stepper.move(-stepper.currentPosition());
    stepper.runToPosition();
    Serial.println(x);
    x=0;
  }
  else{
    x=x+45;
    Serial.println(x);
  }
  // Set the number of steps for a full rotation (360 degrees)
  long steps = 4.1666667*(x/1.8); // Rotate 360 degrees clockwise

  // Move the stepper motor
  stepper.move(steps);
  stepper.runToPosition();

    for (int pos_m=330; pos_m>243; pos_m -=10) {	
      pwm.setPWM(1, 0, pos_m );
      //Serial.println("230 pulse length count --> 90 degrees");
      //Serial.println(pos_m);
      delay(50);
    }  

    for (int pos_m=243; pos_m<253; pos_m +=10) {	
      pwm.setPWM(1, 0, pos_m );
      //Serial.println("230 pulse length count --> 90 degrees");
      delay(50);
    }  

    for (int pos_m=90; pos_m<100; pos_m +=10) {	
      pwm.setPWM(0, 0, pos_m );
      //Serial.println("230 pulse length count --> 90 degrees");
      //Serial.println(pos_m);
      delay(50);
    }    

    for (int pos_m=253; pos_m<263; pos_m +=10) {	
      pwm.setPWM(1, 0, pos_m );
      //Serial.println("230 pulse length count --> 90 degrees");
      delay(50);
    }  

    for (int pos_m=100; pos_m<110; pos_m +=10) {	
      pwm.setPWM(0, 0, pos_m );
      //Serial.println("230 pulse length count --> 90 degrees");
      delay(50);
    }  

    for (int pos_m=273; pos_m<283; pos_m +=10) {	
      pwm.setPWM(1, 0, pos_m );
      //Serial.println("230 pulse length count --> 90 degrees");
      delay(50);
    }  

    for (int pos_m=110; pos_m<115; pos_m +=10) {	
      pwm.setPWM(0, 0, pos_m );
      //Serial.println("230 pulse length count --> 90 degrees");
      delay(50);
    }  

    for (int pos_m=283; pos_m<293; pos_m +=10) {	
      pwm.setPWM(1, 0, pos_m );
      //Serial.println("230 pulse length count --> 90 degrees");
      delay(50);
    }    

    for (int pos_m=115; pos_m<140; pos_m +=10) {	
      pwm.setPWM(0, 0, pos_m );
      //Serial.println("230 pulse length count --> 90 degrees");
      delay(50);
    }    

    for (int pos_m=293; pos_m<326; pos_m +=10) {	
      pwm.setPWM(1, 0, pos_m );
      //Serial.println("230 pulse length count --> 90 degrees");
      delay(50);
    }    

    for (int pos_m=140; pos_m<165; pos_m +=10) {	
      pwm.setPWM(0, 0, pos_m );
      //Serial.println("230 pulse length count --> 90 degrees");
      delay(50);
    }    

    for (int pos_m=326; pos_m<359; pos_m +=10) {	
      pwm.setPWM(1, 0, pos_m );
      //Serial.println("230 pulse length count --> 90 degrees");
      delay(50);
    }    

    for (int pos_m=165; pos_m<190; pos_m +=10) {	
      pwm.setPWM(0, 0, pos_m );
      //Serial.println("230 pulse length count --> 90 degrees");
      delay(50);
    }    

    for (int pos_m=359; pos_m<392; pos_m +=10) {	
      pwm.setPWM(1, 0, pos_m );
      //Serial.println("230 pulse length count --> 90 degrees");
      delay(50);
    }    
    for (int pos_m=190; pos_m<215; pos_m +=10) {	
      pwm.setPWM(0, 0, pos_m );
      //Serial.println("230 pulse length count --> 90 degrees");
      delay(50);
    }    

    for (int pos_m=392; pos_m<425; pos_m +=10) {	
      pwm.setPWM(1, 0, pos_m );
      //Serial.println("230 pulse length count --> 90 degrees");
      delay(50);
    }    
    for (int pos_m=215; pos_m<240; pos_m +=10) {	
      pwm.setPWM(0, 0, pos_m );
      //Serial.println("230 pulse length count --> 90 degrees");
      delay(50);
    }    

    for (int pos_m=425; pos_m<458; pos_m +=10) {	
      pwm.setPWM(1, 0, pos_m );
      //Serial.println("230 pulse length count --> 90 degrees");
      delay(50);
    }    
    for (int pos_m=240; pos_m<265; pos_m +=10) {	
      pwm.setPWM(0, 0, pos_m );
      //Serial.println("230 pulse length count --> 90 degrees");
      delay(50);
    }    

    for (int pos_m=458; pos_m<491; pos_m +=10) {	
      pwm.setPWM(1, 0, pos_m );
      //Serial.println("230 pulse length count --> 90 degrees");
      delay(50);
    }    
    for (int pos_m=265; pos_m<290; pos_m +=10) {	
      pwm.setPWM(0, 0, pos_m );
      //Serial.println("230 pulse length count --> 90 degrees");
      delay(50);
    }    

    for (int pos_m=491; pos_m<524; pos_m +=10) {	
      pwm.setPWM(1, 0, pos_m );
      //Serial.println("230 pulse length count --> 90 degrees");
      delay(50);
    }    
    for (int pos_m=290; pos_m<320; pos_m +=10) {	
      pwm.setPWM(0, 0, pos_m );
      //Serial.println("230 pulse length count --> 90 degrees");
      delay(50);
    }    

    for (int pos_m=524; pos_m<565; pos_m +=10) {	
      pwm.setPWM(1, 0, pos_m );
      //Serial.println("230 pulse length count --> 90 degrees");
      Serial.println("motion 2 completed");
      delay(50);
    }    

    for (int pos=320; pos>280; pos -=10) {		
      pwm.setPWM(0, 0, pos );
      //Serial.println("230 pulse length count --> 90 degrees");
      delay(50);
    }
    
    for (int pos=565; pos>505; pos -=10) {		 
      pwm.setPWM(1, 0, pos );
      //Serial.println("230 pulse length count --> 90 degrees");
      delay(50);
    }

    for (int pos=280; pos>220; pos -=10) {		
      pwm.setPWM(0, 0, pos );
      //Serial.println("230 pulse length count --> 90 degrees");
      delay(50);
    }
    
    for (int pos=505; pos>440; pos -=10) {		 
      pwm.setPWM(1, 0, pos );
      //Serial.println("230 pulse length count --> 90 degrees");
      delay(50);
    }

    for (int pos=220; pos>160; pos -=10) {		
      pwm.setPWM(0, 0, pos );
      //Serial.println("230 pulse length count --> 90 degrees");
      delay(50);
    }
    
    for (int pos=440; pos>360; pos -=10) {		 
      pwm.setPWM(1, 0, pos );
      //Serial.println("230 pulse length count --> 90 degrees");
      delay(50);
    }
     
    for (int pos=160; pos>90; pos -=10) {		
      pwm.setPWM(0, 0, pos );
      //Serial.println("230 pulse length count --> 90 degrees");
      delay(50);
    }
    
    for (int pos=360; pos>330; pos -=10) {		 
      pwm.setPWM(1, 0, pos );
      //Serial.println("230 pulse length count --> 90 degrees");
      Serial.println("motion 3 completed");
      delay(50);
    }

    delay (1000);

}