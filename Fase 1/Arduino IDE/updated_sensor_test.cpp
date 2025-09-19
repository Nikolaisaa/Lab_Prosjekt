#include <QTRSensors.h>
QTRSensors qtr;
const uint8_t sensorCount = 11;
uint16_t sensorValues[sensorCount];
int sensorThreshold[11] = {400,400,400,400,400,400,400,400,400,400,400}; 

#define rightMotorPin A0
#define leftMotorPin A1

class sensorsObject {
  int sensorCount_;
  int* sensorPins_;
  int* sensorThresholds_;
  int* sensorValues_;
 
 sensorsObject(int sensorCount, int* sensorPins, int* sensorThreshold){
 this->sensorCount_ = sensorCount;
 for(int _i=0; _i < sensorCount; _i++){
  this->sensorPins_[_i]=sensorPins[_i];
  this->sensorThresholds_[_i]=sensorThreshold[_i];
  this->sensorValues_[_i]=0;
 }
 }
};
void readArray(){
    qtr.read(sensorValues);
  for (uint8_t i = 0; i < sensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.print('\t');

  for (uint8_t i = 0; i < sensorCount; i++)
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
  delay(50);
}
};

void setup()
{
  //Motor motorRL(rightMotorSpeed 0, leftMotorSpeed 0);
  
  Serial.begin(9600);
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){12,11,10, 9, 8, 7, 6, 5,4,3,2}, sensorCount);
  // sensor value                     22,20,18,16,14,13,12,10,8,6,4
}

void loop()
{
sensorsObject sensors1(sensorCount, sensorPins, sensorThreshold)


}
