#include <Arduino.h>
#include "BME280.h"
// put function declarations here:
BME280 envSensor;
unsigned long start;
void setup()
{
  bool sensorOk = false;
  sensorOk = envSensor.begin(I2C_STANDARD_MODE);
  Serial.begin(115200);
  Serial.println(sensorOk == true ? "Sensor OK." : "Sensor not OK.");
  envSensor.setMode(NORMAL_MODE);
  envSensor.setOversampling(TEMP, OVERSAMPLING16);
  envSensor.setOversampling(HUM, OVERSAMPLING16);
  envSensor.setOversampling(PRESS, OVERSAMPLING16);
  envSensor.iirFilter(IIR16);
  envSensor.inactiveTime(standBy1000ms);

  // put your setup code here, to run once:
  start = millis();
}

void loop()
{

  // Retrieve sensor values
  float temperature = envSensor.getTemperature();
  float pressure = envSensor.getPressure();
  float humidity = envSensor.getHumidity();

  // Print all values in a single line with tab separators
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.print("\t");

  Serial.print("Pressure: ");
  Serial.print(pressure);
  Serial.print("\t");

  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.println();

  delay(500); // Adjust delay as needed
}
