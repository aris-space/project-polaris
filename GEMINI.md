Goal:
write a cpp ros2 humble driver package for an ultrasonic distance sensor based on the code implementation for arduino.

Setting:
Jetson Nano dev kit 8gb,
Serial pins: Jetson serial pin 1 rx/tx
https://www.dfrobot.com/product-2759.html sensor
package name: ultrasonic_driver
directory: src/sensors/HERE





--Software example on Arduino--

#include <SoftwareSerial.h>
unsigned char buffer_RTT[4] = {0};
uint8_t CS;
#define COM 0x55
int Distance = 0;
SoftwareSerial mySerial(7, 8); 
void setup() {
  Serial.begin(115200);
  mySerial.begin(115200);
}
void loop() {
  mySerial.write(COM);
  delay(100);
  if(mySerial.available() > 0){
    delay(4);
    if(mySerial.read() == 0xff){    
      buffer_RTT[0] = 0xff;
      for (int i=1; i<4; i++){
        buffer_RTT[i] = mySerial.read();   
      }
      CS = buffer_RTT[0] + buffer_RTT[1]+ buffer_RTT[2];  
      if(buffer_RTT[3] == CS) {
        Distance = (buffer_RTT[1] << 8) + buffer_RTT[2];
        Serial.print("Distance:");
        Serial.print(Distance);
        Serial.println("mm");
      }
    }
  }
}