#include <OneWire.h>
#include <DallasTemperature.h>

const int oneWireBus = 8;

OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);

DeviceAddress sensorAddress;

void setup()
{
    Serial.begin(9600);
    sensors.begin();
}

void printAddress(DeviceAddress deviceAddress)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        if (deviceAddress[i] < 16)
            Serial.print("0");
        Serial.print(deviceAddress[i], HEX);
    }
}

void loop()
{
    sensors.requestTemperatures();
    float temperature = sensors.getTempCByIndex(0);

    Serial.print("Temperature: ");
    Serial.println(temperature);

    delay(1000);

    if (sensors.getAddress(sensorAddress, 0))
    {
        Serial.print("Sensor 0 Address: ");
        printAddress(sensorAddress);
        Serial.println();
    }
    else
    {
        Serial.println("No sensor found at index 0");
    }
}
