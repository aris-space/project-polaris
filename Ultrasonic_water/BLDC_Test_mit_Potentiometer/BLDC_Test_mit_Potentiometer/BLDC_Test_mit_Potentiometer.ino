/*
  Arduino Brushless Motor Control
*/

#include <Servo.h>

Servo ESC;     // create servo object to control the ESC

int potValue;  // value from the analog pin
const int potPin = A0; //!!!!!!!!!!!!!!!!!

void setup() {
  // Attach the ESC on pin 9!!!!!!!!!!!
  ESC.attach(9,1000,2000); // (pin, min pulse width, max pulse width in microseconds) for ESC
  
  pinMode(potPin, INPUT);
  Serial.begin(9600);

  delay(1000);
}

void runMotor() {
  potValue = analogRead(potPin);   // reads the value of the potentiometer (value between 0 and 1023)
  int motor_value = map(potValue, 0, 1023, 0, 180);   // scale it to use it with the servo library (value between 0 and 180)
  Serial.println(motor_value);
  ESC.write(motor_value);    // Send the signal to the ESC
}

void loop() {
  runMotor();
}