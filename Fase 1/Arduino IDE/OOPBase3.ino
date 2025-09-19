#include <QTRSensors.h>

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

    void readArray() {
        qtr.read(sensorValues_);
        
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

    int getSensorCount() const {
        return sensorCount_;
    }

    uint8_t* getSensorPins() const {
        return sensorPins_;
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

void setMotorSpeed(){
    analogWrite(this->rightMotorPin, this->rightMotorSpeed); //setting right motor speed
    analogWrite(this->leftMotorPin, this->leftMotorSpeed); //setting left motor speed
}
void 
};

const int sensorCount = 11;
uint8_t sensorPins[] = {12,11,10, 9, 8, 7, 6, 5,4,3,2};
uint16_t sensorThresholds[] = {400,400,400,400,400,400,400,400,400,400,400};
const int rightMotorPin = A0;
const int leftMotorPin = A1;

SensorObj so(sensorCount, sensorPins, sensorThresholds);
MotorsObj motors(rightMotorPin, leftMotorPin);


void setup() {
    Serial.begin(9600);

    so.qtr.setTypeRC();
    so.qtr.setSensorPins(so.getSensorPins(), so.getSensorCount());
//
//    delay(500);
}

void loop() {
    so.readArray();
    delay(50);
}
