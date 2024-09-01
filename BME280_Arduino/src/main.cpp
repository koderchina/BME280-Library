#include <Arduino.h>
#include "BME280.h"
// put function declarations here:
BME280 envSensor;

void setup()
{
  bool sensorOk = false;
  sensorOk = envSensor.begin(100000);
  Serial.begin(115200);
  Serial.println(sensorOk == true ? "Sensor OK." : "Sensor not OK.");
  envSensor.setMode(NORMAL_MODE);
  // put your setup code here, to run once:
}

void loop()
{
  envSensor.getSensorData();
  delay(1000);
  // put your main code here, to run repeatedly:
}
