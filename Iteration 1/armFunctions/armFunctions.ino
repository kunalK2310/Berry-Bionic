void movePhi(double deg) { // Defines a function for moving the first segment of the arm (Phi)
  double pos = (deg)*0.7; // Scales the input degree by 0.7 to match the servo's range or mechanical limitations
  servo1.write(pos); // Commands the first servo to move to the calculated position
}

void moveAlpha(double deg) { // Defines a function for moving the second segment of the arm (Alpha) 
  double pos = (180 - deg)*0.6111; // Converts and scales the input degree to ensure correct orientation and movement range for the second servo
  servo2.write(pos); // Moves the second servo to the desired position.
}

void moveBeta(double deg) { // Defines a function for moving the third segment of the arm (Beta)
  double pos = (deg)*0.65; // 0.6944; Scales the input degree by 0.65, customizing the movement for the third servo
  servo3.write(pos); // Commands the third servo to the calculated position.
}

void moveBucket(double deg) { // Function for operating the bucket or grabber mechanism
  double pos = deg*0.7; // Scales the input degree to fit the servo's operation range.
  servo4.write(pos); // Moves the bucket/grabber servo.
  delay(2000);
}

void moveServos(double phi, double alpha, double beta){ // Manages coordinated movement of the arm.
  Serial.println("moving arm"); // Prints a message indicating arm movement.
  moveAlpha(alpha); delay(500); 
  movePhi(phi); delay(500);
  moveBeta(beta); delay(500); // delays between them 
}

void servosStart(double phi, double alpha, double beta) { // Initializes the servos to a starting position
  moveServos(phi,alpha,beta); // Calls moveServos with specified angles
  delay(2000);
}

void moveToBucket(){ //moves arm to position for dumping cotton
  phi = 150;
  alpha = 90;
  beta = 90;
  moveServos(phi,alpha,beta); delay(1000);
} // Assigns predefined angles to phi, alpha, and beta, then moves the servos to these positions

void dumpBucket() {
  moveBucket(90);
  delay(8000);
  moveBucket(210);
} // Closes (moveBucket(90);) and opens (moveBucket(210);) the grabber with an 8-second delay in between

void moveArmPreSet(double x, double y, double z) { // Adjusts the arm for specified task scenarios based on the object's height
  openClaw();
  if (z > 85) {
    //say its 11 in hole
    moveServos(90, 45, 140); // Moves the arm to preset angles for reaching into an 11-inch deep hole
  }
  else {
    //say its 7 in hole
    moveBeta(180); delay(1000); // Positions the third segment of the arm directly, then waits for 1 second
    moveServos(90, 33, 135); // Adjusts the arm for a 7-inch deep hole
  }
  closeClaw(); 
  moveToBucket();
  openClaw();
  closeClaw();
} //  Closes the claw to grasp the object; Moves the arm to position the object over a bucket; Opens the claw to release the object; Closes the claw

void moveArm(double x, double y, double z) { // Dynamically calculates and moves the arm based on specific (x, y, z) coordinates.
  bool keep = true; 
  x = x + camx; // good
  y = y + camy; // good
  z = z + camz; // good

  double theta1 = atan2(y, x);
 // Calculates the angle theta1 (in radians) from the arm's base to the target, using the arctangent of y over x, providing a correct angle even when x is 0.
  double s = z - l1; // account for origin ground level, middle of servo1

  double t = sqrt((x*x) + (y*y)); // Computes the horizontal distance t to the target from the base of the arm.
  double r = sqrt((t*t) + (s*s)); // Determines the direct line distance r to the target from the shoulder joint

  double A = atan2(s, t); // Calculates the angle A from the horizontal to the line r.
  double B = acos( ((l3*l3) - (r*r) - (l2*l2)) / (-2*r*l2) ); // Computes angle B using the law of cosines, necessary for calculating the elbow's position.
  double testVal = ((l3*l3) - (r*r) - (l2*l2)) / (-2*r*l2); // For debugging or validation, ensuring B is calculable.

  double theta2 = A + B; // elbow up; Determines the shoulder joint angle (theta2), representing the angle for "elbow up" configuration.

  double theta3 = acos( ((r*r) - (l2*l2) - (l3*l3)) / (-2*l2*l3) ); // elbow up; Calculates the elbow joint angle (theta3); also for an "elbow up" posture.
  /*
    double theta2 = A - B; // elbow down

    double theta3 = PI - acos( ((r * r) - (L2 * L2) - (L3 * L3)) / (-2 * L2 * L3) ); // elbow down
  */

  phi = theta1 * (RAD_TO_DEG); //Converts theta1 to degrees, setting the base rotation angle.
  alpha = 180 - (theta2 * (RAD_TO_DEG)); // Converts theta2 to degrees and adjusts for the servo's orientation, setting the shoulder angle.
  beta = 205 - (theta3 * (RAD_TO_DEG)); // Converts theta3 to degrees, with further adjustment, likely specific to the arm's design, for the elbow angle.
  
  openClaw(); delay(2000); // Opens the claw
  moveServos(phi, alpha, beta); delay(1000); // Moves the arm to the calculated angles 
  closeClaw(); delay(2000); // Closes the claw, gripping an object, 
  moveToBucket(); delay(2000); // Moves the arm to a predefined position for dumping the object
  openClaw(); delay(2000);
  closeClaw(); delay(2000);
} // Adjusts the target coordinates by adding the camera's offset (camx, camy, camz), aligning the target with the arm's base coordinate system.


void closeClaw() { // definition of the closeClaw function
  digitalWrite(in1_g,LOW); // Sets the digital pin connected to the claw's motor driver input in1_g to LOW, motor driver's logic
  digitalWrite(in2_g,HIGH); // Sets another pin in2_g to HIGH, likely establishing the motor's direction for closing the claw
  analogWrite(enA_g,actuatorSpeed); // Sends a PWM signal to the enable pin enA_g of the motor driver, controlling the speed of the motor based on the value of actuatorSpeed
  Serial.println("Closing claw"); // Output message
  delay(2000);
}

void openClaw() {
  digitalWrite(in1_g,HIGH); // reversing the motor's direction compared to the closeClaw function
  digitalWrite(in2_g,LOW); // Sets in2_g to LOW, establishing the motor direction for opening.
  analogWrite(enA_g,actuatorSpeed);
  Serial.println("Opening claw");
  delay(2000);
}
