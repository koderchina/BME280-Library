#include <Arduino.h>
#include "BME280.h"
// put function declarations here:

void setup()
{
  BME280 envSensor;
  bool sensorOk = false;
  sensorOk = envSensor.begin(100000);
  Serial.begin(115200);
  Serial.println(sensorOk == true ? "Sensor OK." : "Sensor not OK.");
  envSensor.setMode(NORMAL_MODE);
  envSensor.setMode(SLEEP_MODE);
  envSensor.setMode(FORCED_MODE);
  envSensor.setMode(NORMAL_MODE);
  // put your setup code here, to run once:
}

void loop()
{
  // put your main code here, to run repeatedly:
}
