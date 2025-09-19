#include <Arduino.h>
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



    int getHighestSensorValueIndex(){
        int highest_sensor_value = 0;
        int highest_sensor_value_index = 5;

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
        
        int highestSensorValueIndex = getHighestSensorValueIndex();

        Serial.print(highestSensorValueIndex);
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

const int sensorCount = 1;
uint8_t sensorPins[] = {1};
uint16_t sensorThresholds[] = {500};

SensorObj so(sensorCount, sensorPins, sensorThresholds);

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
