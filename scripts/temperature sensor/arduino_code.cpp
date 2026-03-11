#include <OneWire.h>
#include <DallasTemperature.h>

const int oneWireBus = 8;

OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);

DeviceAddress sensorAddress;

// Define sensor addresses
DeviceAddress sensor0 = {0x28, 0x61, 0x64, 0x0A, 0xF2, 0x7E, 0xA5, 0xC7};
DeviceAddress sensor1 = {0x28, 0x61, 0x64, 0x35, 0xFB, 0x64, 0x26, 0xFE};
DeviceAddress sensor2 = {0x28, 0x61, 0x64, 0x35, 0xC7, 0xF3, 0x6B, 0x8D};
DeviceAddress sensor3 = {0x28, 0x61, 0x64, 0x0A, 0xF3, 0xF8, 0xD1, 0x70};
DeviceAddress sensor4 = {0x28, 0x61, 0x64, 0x0A, 0xF3, 0xA8, 0x71, 0xA8};
DeviceAddress sensor5 = {0x28, 0x61, 0x64, 0x35, 0xC6, 0x66, 0x73, 0x45};



void setup() {
  Serial.begin(9600);
  sensors.begin();
  delay(500);
}


void loop() {
  sensors.requestTemperatures();
  
  float temperature0 = sensors.getTempC(sensor0);
  float temperature1 = sensors.getTempC(sensor1);
  float temperature2 = sensors.getTempC(sensor2);
  float temperature3 = sensors.getTempC(sensor3);
  float temperature4 = sensors.getTempC(sensor4);
  float temperature5 = sensors.getTempC(sensor5);
  
  Serial.print("DATA,");
  Serial.print(temperature0); Serial.print(",");
  Serial.print(temperature1); Serial.print(",");
  Serial.print(temperature2); Serial.print(",");
  Serial.print(temperature3); Serial.print(",");
  Serial.print(temperature4); Serial.print(",");
  Serial.println(temperature5);
    
  

  delay(3000);

}


