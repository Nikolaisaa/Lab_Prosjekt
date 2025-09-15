#include <QTRSensors.h>
QTRSensors qtr;
const uint8_t SensorCount = 5;
uint16_t sensorValues[SensorCount];
int sensorThreshold[5] = {350,200,200,200,350}; 

void setup()
{
  Serial.begin(9600);
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){26, 27, 28, 29, 30}, SensorCount);
  delay(1000);
}

void loop()
{
  qtr.read(sensorValues);
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.print('\t');

  for (uint8_t i = 0; i < SensorCount; i++)
  {
    if (sensorValues[i] > sensorThreshold[i]) 
    {
      Serial.print("X");  // black surface detected
    }
    else 
    {
      Serial.print("-");  // white surface detected
    }
  }
  Serial.println();
  delay(250);
}
