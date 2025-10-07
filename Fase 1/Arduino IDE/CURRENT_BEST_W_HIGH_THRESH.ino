
#include <QTRSensors.h>

int lastIndexFlag = 2;
int maxSpeed = 220;
int minSpeed = maxSpeed/9;
int outSpeed = 180;
int negativeOutSpeed = 150;
int activeSensorsCount = 0;

const int AIN1 = A4;
const int AIN2 = A5;
const int BIN1 = A6;
const int BIN2 = A7;

/*info:
left motor = B
right motor = A
*/

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

                activeSensorsCount +=1;

                if(sensorValues_[i1] > highest_sensor_value){
                    highest_sensor_value = sensorValues_[i1];
                    highest_sensor_value_index = i1;
                }

            }
        }

        return highest_sensor_value_index;
    }

    int* getMotorSpeedsFromHighestIndex(int theHighestIndex, int* motorSpeeds){ //venstre side av sensor starter her
        if(activeSensorsCount > 3){
            digitalWrite(AIN1, HIGH);
            digitalWrite(AIN2, LOW);
            digitalWrite(BIN1, HIGH);
            digitalWrite(BIN2, LOW);
            motorSpeeds[0] = maxSpeed;
            motorSpeeds[1] = maxSpeed;
        }
        else if(theHighestIndex >= 0 && theHighestIndex < 1){
            digitalWrite(AIN1, HIGH);
            digitalWrite(AIN2, LOW);
            digitalWrite(BIN1, HIGH);
            digitalWrite(BIN2, LOW);
            motorSpeeds[0] = minSpeed;
            motorSpeeds[1] = maxSpeed;
            lastIndexFlag = 0;
        }
        else if (theHighestIndex >=1 && theHighestIndex < 2){
            digitalWrite(AIN1, HIGH);
            digitalWrite(AIN2, LOW);
            digitalWrite(BIN1, HIGH);
            digitalWrite(BIN2, LOW);
            motorSpeeds[0] = minSpeed*2;
            motorSpeeds[1] = maxSpeed;
            lastIndexFlag = 0;
        }
        else if(theHighestIndex >= 2 && theHighestIndex < 3){
            digitalWrite(AIN1, HIGH);
            digitalWrite(AIN2, LOW);
            digitalWrite(BIN1, HIGH);
            digitalWrite(BIN2, LOW);
            motorSpeeds[0] = minSpeed*3;
            motorSpeeds[1] = maxSpeed;
            lastIndexFlag = 0;
        }
        else if (theHighestIndex >=3 && theHighestIndex < 4){
            digitalWrite(AIN1, HIGH);
            digitalWrite(AIN2, LOW);
            digitalWrite(BIN1, HIGH);
            digitalWrite(BIN2, LOW);
            motorSpeeds[0] = minSpeed*4;
            motorSpeeds[1] = maxSpeed;
            lastIndexFlag = 0;
        }
        else if(theHighestIndex >=7 && theHighestIndex < 8){ //høyre side av sensor starter her
            digitalWrite(AIN1, HIGH);
            digitalWrite(AIN2, LOW);
            digitalWrite(BIN1, HIGH);
            digitalWrite(BIN2, LOW);
            motorSpeeds[0] = maxSpeed;
            motorSpeeds[1] = minSpeed*4;
            lastIndexFlag = 1;
        }
        else if(theHighestIndex >= 8 && theHighestIndex < 9){
            digitalWrite(AIN1, HIGH);
            digitalWrite(AIN2, LOW);
            digitalWrite(BIN1, HIGH);
            digitalWrite(BIN2, LOW);
            motorSpeeds[0] = maxSpeed;
            motorSpeeds[1] = minSpeed*3;
            lastIndexFlag = 1;
        }
        else if(theHighestIndex >=9 && theHighestIndex < 10){
            digitalWrite(AIN1, HIGH);
            digitalWrite(AIN2, LOW);
            digitalWrite(BIN1, HIGH);
            digitalWrite(BIN2, LOW);
            motorSpeeds[0] = maxSpeed;
            motorSpeeds[1] = minSpeed*2;
            lastIndexFlag = 1;
        }
        else if(theHighestIndex >= 10 && theHighestIndex < 11){
            digitalWrite(AIN1, HIGH);
            digitalWrite(AIN2, LOW);
            digitalWrite(BIN1, HIGH);
            digitalWrite(BIN2, LOW);
            motorSpeeds[0] = maxSpeed;
            motorSpeeds[1] = minSpeed;
            lastIndexFlag = 1;
        }
        else if(theHighestIndex == 20){ //sensor utenfor linjen starter her
                if(lastIndexFlag == 0){
                    digitalWrite(BIN1, LOW);
                    digitalWrite(BIN2, HIGH);
                    motorSpeeds[0] = negativeOutSpeed; //negative
                    motorSpeeds[1] = outSpeed;
                    }
                else{
                    digitalWrite(AIN1, LOW);
                    digitalWrite(AIN2, HIGH);
                    motorSpeeds[0] = outSpeed;
                    motorSpeeds[1] = negativeOutSpeed; //negative
                }
        }
        else{ //linjen midt på sensor her
            digitalWrite(AIN1, HIGH);
            digitalWrite(AIN2, LOW);
            digitalWrite(BIN1, HIGH);
            digitalWrite(BIN2, LOW);
            motorSpeeds[0] = maxSpeed;
            motorSpeeds[1] = maxSpeed;
        }
        activeSensorsCount = 0;
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
//                      22,20,18,16,14,13,12,10,8,6,5!! (viktig 5)
uint16_t sensorThresholds[] = {290,290,290,340,350,350,390,410,410,410,410};
const int rightMotorPin = A0;
const int leftMotorPin = A1;


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

    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);

    delay(500);
}

void loop() {
    so.readArray();
    so.deBug();
    int highestSensorValueIndex = so.getHighestSensorValueIndex();
    //Serial.print(highestSensorValueIndex);
    //Serial.print('\n');
    
    //analogWrite(rightMotorPin, 255);
    //analogWrite(leftMotorPin, 255);
    int ms[2] = {0,0};
    so.getMotorSpeedsFromHighestIndex(highestSensorValueIndex, ms);

    //Serial.print(ms[0]);
    //Serial.println();
    //Serial.print(ms[1]);
    motors.setMotorSpeed(ms);
    delay(2);
}
