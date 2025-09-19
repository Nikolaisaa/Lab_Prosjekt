#include <iostream>

class SensorObj {
private:
    int sensorCount_;
    int* sensorPins_;
    int* sensorThresholds_;
    int* sensorValues_;

public:
    SensorObj(int sensorCount, int* sensorPins, int* sensorThresholds) {
        this->sensorCount_ = sensorCount;
        this->sensorPins_ = new int[sensorCount];
        this->sensorValues_ = new int[sensorCount];
        this->sensorThresholds_ = new int[sensorCount];

        for (int _i = 0; _i < sensorCount; _i++) {
            this->sensorPins_[_i] = sensorPins[_i];
            this->sensorThresholds_[_i] = sensorThresholds[_i];
            this->sensorValues_[_i] = 0;
        }
    }

    int getSensorCount() const {
        return this->sensorCount_;
    }

    int* getSensorPins() const {
        return this->sensorPins_;
    }

    ~SensorObj() {
        delete[] sensorPins_;
        delete[] sensorValues_;
        delete[] sensorThresholds_;
    }
};


int sensorCount = 1;
int sensorPins[] = {1};
int sensorThresholds[] = {1};

SensorObj so(sensorCount, sensorPins, sensorThresholds);

void setup() {
    //Brrr setup
}

void loop() {
    int* sPins = so.getSensorPins();
    for (int _k = 0; _k < so.getSensorCount(); _k++) {
        std::cout << "Sensor pin: " << sPins[_k] << std::endl;
    }
}

int main() {
    setup();
    loop();
    std::cout << "Hello" << std::endl;
    return 0;
}
