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

  if (maxTemp == 0.0 || maxTemp == -127) {
    maxTemp = 1;
  }

  long rawDelay = (1.0 - (int)maxTemp / 70.0) * 7000L;
  long dynamicDelay = constrain((long)(rawDelay), 1000, 4800);

  Serial.print("DATA,");
  Serial.print(t4); Serial.print(","); //Front
  Serial.print(t5); Serial.print(","); //Middle
  Serial.println(t2);  //Back

  delay(dynamicDelay);
}


