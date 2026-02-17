/*
  Arduino Brushless Motor Control
*/

#include <Servo.h>

Servo ESC;     // create servo object to control the ESC

void setup() {
  // Attach the ESC on pin 9
  ESC.attach(9,1000,2000); // (pin, min pulse width, max pulse width in microseconds) for ESC 
}


int runMotor() {
  for (int i=0; i<180; i++) {
    ESC.write(i);
    delay(100);
  }

  delay(1000);

  for (int k=180; k>0; k--) {
    ESC.write(k);
    delay(100);
  }

  delay(1000);
}


void loop() {  
  runMotor();
}