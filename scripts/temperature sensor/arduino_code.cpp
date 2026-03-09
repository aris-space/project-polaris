#include <OneWire.h>
#include <DallasTemperature.h>

const int oneWireBus = 8;

OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);

DeviceAddress sensorAddress;

// Define sensor addresses
//DeviceAddress sensor0 = {0x28, 0x61, 0x64, 0x0A, 0xF2, 0x7E, 0xA5, 0xC7};
//DeviceAddress sensor1 = {0x28, 0x61, 0x64, 0x35, 0xFB, 0x64, 0x26, 0xFE};
DeviceAddress sensor2 = {0x28, 0x61, 0x64, 0x35, 0xC7, 0xF3, 0x6B, 0x8D};
//DeviceAddress sensor3 = {0x28, 0x61, 0x64, 0x0A, 0xF3, 0xF8, 0xD1, 0x70};
DeviceAddress sensor4 = {0x28, 0x61, 0x64, 0x0A, 0xF3, 0xA8, 0x71, 0xA8};
DeviceAddress sensor5 = {0x28, 0x61, 0x64, 0x35, 0xC6, 0x66, 0x73, 0x45};

unsigned long lastRequestTime = 0;


void setup() {
  Serial.begin(115200);
  sensors.begin();
  delay(500);
}


void loop() {
  sensors.requestTemperatures();
  float t2 = sensors.getTempC(sensor2);
  float t4 = sensors.getTempC(sensor4);
  float t5 = sensors.getTempC(sensor5);

  float maxTemp = max(t2, max(t4, t5));

  //Feedback loop for adjusting delay.
  //if steadystate normal temperature
  // TODO: Check if it makes sense in lake application.
  long rawDelay = (85.0 - 1.3 * (int)maxTemp) * 1000L; // Adjust delay based on max temperature
  //long rawDelay = (30.0 - 1.0 * (int)maxTemp) * 1000L;
  // Constrain it so it never goes below 1 second (1000ms) 
  // and never above 65 seconds (65000ms)
  long dynamicDelay = constrain((long)(rawDelay), 5000, 65000);

  Serial.print("DATA,");
  Serial.print(t4); Serial.print(",");
  Serial.print(t5); Serial.print(",");
  Serial.println(t2);  

  delay(dynamicDelay);
}


