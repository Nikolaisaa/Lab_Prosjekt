#include <Arduino.h>
#include <QTRSensors.h>

#define sensorCount 15

const int leftF = 2;
const int leftB = 4;
const int rightF = 15;
const int rightB = 16;
const int minDiff = 40;

float lastPositionError = 0; // For å huske siste kjente posisjonsfeil
unsigned long lastActiveTime = 0; // Tidspunkt for siste aktive sensor
const unsigned long maxInactiveDuration = 1000; // Maksimum inaktiv tid i millisekunder

// Opprett et QTR-sensorobjekt og arrays for sensordata
QTRSensors qtr;
uint16_t sensorValues[sensorCount];
uint16_t startValues[sensorCount];

// Definer sensorpinnene (oppdatert med nye sensorer)
const uint8_t sensorPins[sensorCount] = {
    32, // Sensor 0 (venstre ytterste)
    5,  // Sensor 1 (ny sensor på pinne 5)
    33, // Sensor 2
    25, // Sensor 3
    26, // Sensor 4
    27, // Sensor 5
    14, // Sensor 6
    12, // Sensor 7 (midten)
    17, // Sensor 8 (ny sensor på pinne 17)
    13, // Sensor 9
    23, // Sensor 10
    22, // Sensor 11
    21, // Sensor 12
    19, // Sensor 13
    18  // Sensor 14 (høyre ytterste)
};

// Definer sensorvekter for vekting (oppdatert for nye sensorer)
int sensorWeights[sensorCount] = {
    -15, // Sensor 0
    -9, // Sensor 1 (ny sensor)
    -7, // Sensor 2
    -5, // Sensor 3
    -3, // Sensor 4
    -2, // Sensor 5
    -1, // Sensor 6
     0, // Sensor 7 (midten)
     1, // Sensor 8 (ny sensor)
     2, // Sensor 9
     3, // Sensor 10
     5, // Sensor 11
     7, // Sensor 12
     9, // Sensor 13
     15  // Sensor 14
};

void setup() {
    Serial.begin(9600);

    pinMode(leftF, OUTPUT);
    pinMode(rightF, OUTPUT);
    pinMode(leftB, OUTPUT);
    pinMode(rightB, OUTPUT);

    for (uint8_t i = 0; i < sensorCount; i++) {
        pinMode(sensorPins[i], INPUT);
    }

    qtr.setTypeRC();
    qtr.setSensorPins(sensorPins, sensorCount);
    qtr.read(startValues);  // Les og lagre startverdiene ved oppstart

    lastActiveTime = millis(); // Initialiser siste aktive tid

    Serial.println("Sensoroppsett fullført. Startverdier lagret.");
}

void setMotorSpeed(int leftSpeed, int rightSpeed) {
    // Venstre motor
    if (leftSpeed > 0) {
        analogWrite(leftF, leftSpeed);
        analogWrite(leftB, 0);
    } else if (leftSpeed < 0) {
        analogWrite(leftF, 0);
        analogWrite(leftB, -leftSpeed);
    } else {
        analogWrite(leftF, 0);
        analogWrite(leftB, 0);
    }

    // Høyre motor
    if (rightSpeed > 0) {
        analogWrite(rightF, rightSpeed);
        analogWrite(rightB, 0);
    } else if (rightSpeed < 0) {
        analogWrite(rightF, 0);
        analogWrite(rightB, -rightSpeed);
    } else {
        analogWrite(rightF, 0);
        analogWrite(rightB, 0);
    }
}

void linjeOgMotor() {
    const int maxSpeed = 255; // Maksimal motorhastighet
    const int baseSpeed = 120; // Grunnhastighet når ikke perfekt justert

    qtr.read(sensorValues);   // Les alle sensorverdiene

    int weightedSum = 0;
    int totalActiveSensors = 0;
    bool anySensorActive = false;

    // Iterer over alle sensorene
    for (int i = 0; i < sensorCount; i++) {
        int diff = abs(sensorValues[i] - startValues[i]);
        if (diff > minDiff) {
            // Sensor er aktiv
            weightedSum += sensorWeights[i];
            totalActiveSensors++;
            anySensorActive = true;
        }
    }

    // Oppdater siste aktive tid hvis noen sensorer er aktive
    if (anySensorActive) {
        lastActiveTime = millis();
    }

    // Sjekk om maks inaktiv tid er overskredet
    if (millis() - lastActiveTime >= maxInactiveDuration) {
        // Stopper motorene
        setMotorSpeed(0, 0);
        Serial.println("Ingen sensorer aktive i over 1 sekund. Stopper motorene.");
        return; // Avslutt funksjonen
    }

    float positionError = 0;

    if (totalActiveSensors > 0) {
        // Beregn posisjonsfeil
        positionError = (float)weightedSum / totalActiveSensors;
    } else {
        // Ingen sensorer er aktive, bruk siste kjente posisjonsfeil
        positionError = lastPositionError;
        Serial.println("Ingen sensorer aktive, bruker siste kjente posisjonsfeil");
    }

    // Beregn derivatet av posisjonsfeilen
    float derivative = positionError - lastPositionError;

    // Oppdaterer siste kjente posisjonsfeil
    lastPositionError = positionError;

    // Juster motorene basert på posisjonsfeilen og derivatet
    float Kp = 25.0; // Proporsjonal konstant
    float Kd = 25.0; // Derivatkonstant

    int correction = (int)(Kp * positionError + Kd * derivative);

    // Bestem basehastigheten basert på posisjonsfeilen
    int currentBaseSpeed;
    if (positionError == 0) {
        currentBaseSpeed = maxSpeed; // Maksimal hastighet når perfekt justert
    } else {
        currentBaseSpeed = baseSpeed; // Grunnhastighet ellers
    }

    int leftMotorSpeed = currentBaseSpeed + correction;
    int rightMotorSpeed = currentBaseSpeed - correction;

    // Begrens motorhastighetene til gyldig område (-maxSpeed til +maxSpeed)
    leftMotorSpeed = constrain(leftMotorSpeed, -maxSpeed, maxSpeed);
    rightMotorSpeed = constrain(rightMotorSpeed, -maxSpeed, maxSpeed);

    // Sett motorhastighetene
    setMotorSpeed(leftMotorSpeed, rightMotorSpeed);

    // Debugging-utskrifter
    Serial.print("Posisjonsfeil: ");
    Serial.println(positionError);
    Serial.print("Derivat: ");
    Serial.println(derivative);
    Serial.print("Basehastighet: ");
    Serial.println(currentBaseSpeed);
    Serial.print("Venstre motorhastighet: ");
    Serial.println(leftMotorSpeed);
    Serial.print("Høyre motorhastighet: ");
    Serial.println(rightMotorSpeed);
}


void loop() {
    linjeOgMotor();  // Kjør funksjonen for å sjekke sensorverdier og motorstyring
    // Ingen delay for raskere respons
}
