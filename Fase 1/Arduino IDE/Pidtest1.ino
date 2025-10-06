#include <QTRSensors.h>

// Motor pins
const int AIN1 = A4;
const int AIN2 = A5;
const int BIN1 = A6;
const int BIN2 = A7;
const int rightMotorPin = A0;
const int leftMotorPin = A1;

// Motor limits
int maxSpeed = 180;
int minSpeed = maxSpeed / 10;

// PID parameters
float Kp = 25.0;
float Ki = 0.0;
float Kd = 8.0;

// Sensor configuration
const int sensorCount = 11;
uint8_t sensorPins[] = {12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2};
uint16_t sensorThresholds[] = {250, 250, 250, 300, 310, 310, 350, 370, 370, 370, 370};

class SensorObj {
private:
    int sensorCount_;
    uint8_t* sensorPins_;
    uint16_t* sensorThresholds_;
    uint16_t* sensorValues_;

public:
    QTRSensorsRC qtr;

    SensorObj(int sensorCount, uint8_t* sensorPins, uint16_t* sensorThresholds) {
        sensorCount_ = sensorCount;
        sensorPins_ = new uint8_t[sensorCount];
        sensorThresholds_ = new uint16_t[sensorCount];
        sensorValues_ = new uint16_t[sensorCount];
        for (int i = 0; i < sensorCount; i++) {
            sensorPins_[i] = sensorPins[i];
            sensorThresholds_[i] = sensorThresholds[i];
            sensorValues_[i] = 0;
        }
    }

    void readSensors() {
        qtr.read(sensorValues_);
    }

    int getLinePosition() {
        long numerator = 0;
        long denominator = 0;
        for (int i = 0; i < sensorCount_; i++) {
            uint16_t value = sensorValues_[i];
            numerator += (long)value * i * 1000;
            denominator += value;
        }

        if (denominator != 0) {
            return numerator / denominator;
        } else {
            // Return extreme left or right if line is lost
            return sensorCount_ * 1000 / 2; // center as default
        }
    }

    void debug() {
        for (int i = 0; i < sensorCount_; i++) {
            Serial.print(sensorValues_[i]);
            Serial.print('\t');
        }
        Serial.println();
    }

    ~SensorObj() {
        delete[] sensorPins_;
        delete[] sensorThresholds_;
        delete[] sensorValues_;
    }
};

class MotorsObj {
private:
    int rightMotorPin_;
    int leftMotorPin_;
    int lastError = 0;
    float integral = 0;

public:
    MotorsObj(int rightMotorPin, int leftMotorPin) {
        rightMotorPin_ = rightMotorPin;
        leftMotorPin_ = leftMotorPin;
    }

    void drive(int error) {
        integral += error;
        float derivative = error - lastError;
        float correction = Kp * error + Ki * integral + Kd * derivative;
        lastError = error;

        int leftSpeed = maxSpeed + correction / 100;
        int rightSpeed = maxSpeed - correction / 100;

        leftSpeed = constrain(leftSpeed, minSpeed, maxSpeed);
        rightSpeed = constrain(rightSpeed, minSpeed, maxSpeed);

        analogWrite(leftMotorPin_, leftSpeed);
        analogWrite(rightMotorPin_, rightSpeed);

        // Set direction pins
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
        digitalWrite(BIN1, HIGH);
        digitalWrite(BIN2, LOW);
    }
};

// Instantiate objects
SensorObj sensors(sensorCount, sensorPins, sensorThresholds);
MotorsObj motors(rightMotorPin, leftMotorPin);

void setup() {
    Serial.begin(9600);

    sensors.qtr.setTypeRC();
    sensors.qtr.setSensorPins(sensorPins, sensorCount);

    pinMode(rightMotorPin, OUTPUT);
    pinMode(leftMotorPin, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);

    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);

    delay(500);
}

void loop() {
    sensors.readSensors();
    sensors.debug();

    int position = sensors.getLinePosition();
    int targetPosition = (sensorCount - 1) * 500; // center
    int error = targetPosition - position;

    motors.drive(error);

    delay(3);
}
