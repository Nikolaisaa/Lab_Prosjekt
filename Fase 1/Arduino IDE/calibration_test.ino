#include <QTRSensors.h>
QTRSensors qtr;
const uint8_t SensorCount = 5;
uint16_t sensorValues[SensorCount];

void setup()
{
  Serial.begin(9600);
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){26, 27, 28, 29, 30}, SensorCount);
  delay(1000);

  Serial.println((String)"Start calibration");
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  Serial.println((String)"Stop calibration");

  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  delay(1000);
}

void loop()
{
  qtr.readLineBlack(sensorValues);
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  delay(250);
  Serial.println();
}
