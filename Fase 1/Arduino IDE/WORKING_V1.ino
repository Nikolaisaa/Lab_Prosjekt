#include <QTRSensors.h>

int lastIndexFlag = 2;

class SensorObj {
private:
    int sensorCount_;
    uint8_t* sensorPins_;
    uint16_t* sensorThresholds_;
    uint16_t* sensorValues_;

public:
    QTRSensors qtr;

    SensorObj(int sensorCount, uint8_t* sensorPins, uint16_t* sensorThresholds) {
        this->sensorCount_ = sensorCount;
        this->sensorPins_ = new uint8_t[sensorCount];
        this->sensorValues_ = new uint16_t[sensorCount];
        this->sensorThresholds_ = new uint16_t[sensorCount];

        for (int i = 0; i < sensorCount; i++) {
            this->sensorPins_[i] = sensorPins[i];
            this->sensorThresholds_[i] = sensorThresholds[i];
            this->sensorValues_[i] = 0;
        }
    }

    int getHighestSensorValueIndex(){
        int highest_sensor_value = 0;
        int highest_sensor_value_index = 20;

        for (int i1 = 0; i1 < sensorCount_; i1++) {
            if (sensorValues_[i1] > sensorThresholds_[i1]) {

                if(sensorValues_[i1] > highest_sensor_value){
                    highest_sensor_value = sensorValues_[i1];
                    highest_sensor_value_index = i1;
                }

            }
        }

        return highest_sensor_value_index;
    }

    int* getMotorSpeedsFromHighestIndex(int theHighestIndex, int* motorSpeeds){
        if(theHighestIndex < 4 && theHighestIndex >= 0){
            motorSpeeds[0] = 0;
            motorSpeeds[1] = 100;
            lastIndexFlag = 0;
            Serial.print(lastIndexFlag);
        }
        else if(theHighestIndex > 6 && theHighestIndex < 12){
            motorSpeeds[0] = 100;
            motorSpeeds[1] = 0;
            lastIndexFlag = 1;
            Serial.print(lastIndexFlag);
        }
        else if(theHighestIndex == 20){
                if(lastIndexFlag == 0){
                    motorSpeeds[0] = 0;
                    motorSpeeds[1] = 100;
                    }
                else{
                    motorSpeeds[0] = 100;
                    motorSpeeds[1] = 0;
                }
        }
        else{
            motorSpeeds[0] = 100;
            motorSpeeds[1] = 100;
        }
        return motorSpeeds;
    }

    void deBug(){
        for (int i = 0; i < sensorCount_; i++) {
            Serial.print(sensorValues_[i]);
            Serial.print('\t');
        }
        Serial.print('\t');

        for (int i = 0; i < sensorCount_; i++) {
            if (sensorValues_[i] > sensorThresholds_[i]) {
                Serial.print("X");
            } else {
                Serial.print("-");
            }
        }
        Serial.println();
    }

    void readArray() {
        qtr.read(sensorValues_);
        //deBug();

    }

    uint8_t* getSensorPins() const {
        return sensorPins_;
    }

    int getSensorCount() const {
        return sensorCount_;
    }

    ~SensorObj() {
        delete[] sensorPins_;
        delete[] sensorValues_;
        delete[] sensorThresholds_;
    }
    
};

class MotorsObj {
    public:
    int rightMotorPin;
    int leftMotorPin;
    int rightMotorSpeed = 0;
    int leftMotorSpeed = 0;

    MotorsObj(int rightMotorPin, int leftMotorPin){
        this->rightMotorPin = rightMotorPin;
        this->leftMotorPin = leftMotorPin;
        
    }

void setMotorSpeed(int* motorSpeeds){
    this->leftMotorSpeed = motorSpeeds[0];
    this->rightMotorSpeed = motorSpeeds[1];
    analogWrite(this->rightMotorPin, this->rightMotorSpeed); //setting right motor speed
    analogWrite(this->leftMotorPin, this->leftMotorSpeed); //setting left motor speed
    
    /*Serial.print(motorSpeeds[0]);
    Serial.print('\t');
    Serial.print(motorSpeeds[1]);
    Serial.println();
    */
}
};

const int sensorCount = 11;
uint8_t sensorPins[] = {12,11,10, 9, 8, 7, 6, 5,4,3,2};
//                      22,20,18,16,14,13,12,10,8,6,4
uint16_t sensorThresholds[] = {350,350,350,350,350,350,350,350,350,350,350};
const int rightMotorPin = A0;
const int leftMotorPin = A1;
const int AIN1 = A4;
const int AIN2 = A5;
const int BIN1 = A6;
const int BIN2 = A7;

SensorObj so(sensorCount, sensorPins, sensorThresholds);
MotorsObj motors(rightMotorPin, leftMotorPin);

void setup() {
    Serial.begin(9600);
    so.qtr.setTypeRC();
    so.qtr.setSensorPins(so.getSensorPins(), so.getSensorCount());
    pinMode(rightMotorPin, OUTPUT);
    pinMode(leftMotorPin, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);

    delay(500);
}

void loop() {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    so.readArray();
    so.deBug();
    int highestSensorValueIndex = so.getHighestSensorValueIndex();
    Serial.print(highestSensorValueIndex);
    Serial.print('\n');
    
    //analogWrite(rightMotorPin, 255);
    //analogWrite(leftMotorPin, 255);
    int ms[2] = {0,0};
    so.getMotorSpeedsFromHighestIndex(highestSensorValueIndex, ms);

    //Serial.print(ms[0]);
    //Serial.println();
    //Serial.print(ms[1]);
    motors.setMotorSpeed(ms);
    delay(50);
}
