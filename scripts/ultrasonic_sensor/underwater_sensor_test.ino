//#include <SoftwareSerial.h>

// const int rx = 3; //yellow cable
// const int tx = 2; // green cable
// // vcc pin -> 5V 

unsigned char buffer_RTT[4] = {0};  //creates 4 byte arraw for sensor data  
uint8_t CS;

#define COM 0x55
int Distance = 0;

//SoftwareSerial mySerial(7, 8); 

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
}
void loop() {
  Serial1.write(COM);
  delay(100);
  if(Serial1.available() > 0){
    delay(4);
    if(Serial1.read() == 0xff){    
      buffer_RTT[0] = 0xff;
      for (int i=1; i<4; i++){
        buffer_RTT[i] = Serial1.read();   
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









