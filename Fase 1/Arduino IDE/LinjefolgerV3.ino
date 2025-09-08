#include <QTRSensors.h>
#define sensorCount 13

const int leftF = 2;
const int leftB = 4;
const int rightF = 15;
const int rightB = 16;
const int minDiff = 60;

// Opprett et QTR-sensorobjekt og arrays for sensordata
QTRSensors qtr;
uint16_t sensorValues[sensorCount];
uint16_t startValues[sensorCount];

// Definer sensorpinnene
const uint8_t sensorPins[sensorCount] = {32, 33, 25, 26, 27, 14, 12, 13, 23, 22, 21, 19, 18};
//                    Sensor-pinner:     25  23  21  19  17  15  13  11   9   7   5   3   1

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

    Serial.println("Sensor setup complete. Startverdier lagret.");
}

// Kombinert funksjon for sensoravlesning og motorstyring
void linjeOgMotor() {
    const int max = 200;
    qtr.read(sensorValues);  // Les alle sensorverdiene
    bool sensorAktiv = false;  // Flag for å spore om noen sensorer er aktive

    // Sjekk sensor 11 først
    if (abs(sensorValues[11] - startValues[11]) > minDiff) {
        Serial.println("Sensor 11 aktiv: Kjører motorer");
        analogWrite(rightF, 0);
        analogWrite(leftF, max);
        sensorAktiv = true;
    }
    // Deretter sensor 10
    else if (abs(sensorValues[10] - startValues[10]) > minDiff) {
        Serial.println("Sensor 10 aktiv: Kjører motorer");
        analogWrite(rightF, max/5);
        analogWrite(leftF, max);
        sensorAktiv = true;
    }
    // Så sensor 12
    else if (abs(sensorValues[12] - startValues[12]) > minDiff) {
        Serial.println("Sensor 12 aktiv: Kjører motorer");
        analogWrite(leftF, max);   // Sett venstre motor til maks
        analogWrite(leftB, 0);
        analogWrite(rightB, max/2);
        analogWrite(rightF, 0);
        sensorAktiv = true;
    }
    // Resten av sensorene i prioritert rekkefølge
    else if (abs(sensorValues[0] - startValues[0]) > minDiff) {
        Serial.println("Sensor 0 aktiv: Kjører motorer");
        analogWrite(rightF, max);  // Sett høyre motor til maks
        analogWrite(leftB, max/2);
        analogWrite(leftF, 0);
        analogWrite(rightB, 0);
        sensorAktiv = true;       // Marker at en sensor er aktiv
    }
    else if (abs(sensorValues[1] - startValues[1]) > minDiff) {
        Serial.println("Sensor 1 aktiv: Kjører motorer");
        analogWrite(rightF, max);
        analogWrite(leftF, 0);
        sensorAktiv = true;
    }
    else if (abs(sensorValues[2] - startValues[2]) > minDiff) {
        Serial.println("Sensor 2 aktiv: Kjører motorer");
        analogWrite(rightF, max);
        analogWrite(leftF, max/5);
        sensorAktiv = true;
    }
    else if (abs(sensorValues[3] - startValues[3]) > minDiff) {
        Serial.println("Sensor 3 aktiv: Kjører motorer");
        analogWrite(rightF, max);
        analogWrite(leftF, max/3);
        sensorAktiv = true;
    }
    else if (abs(sensorValues[9] - startValues[9]) > minDiff) {
        Serial.println("Sensor 9 aktiv: Kjører motorer");
        analogWrite(rightF, max/3);
        analogWrite(leftF, max);
        sensorAktiv = true;
    }
    else if (abs(sensorValues[4] - startValues[4]) > minDiff) {
        Serial.println("Sensor 4 aktiv: Kjører motorer");
        analogWrite(rightF, max);
        analogWrite(leftF, max/2);
        sensorAktiv = true;
    }
    else if (abs(sensorValues[8] - startValues[8]) > minDiff) {
        Serial.println("Sensor 8 aktiv: Kjører motorer");
        analogWrite(rightF, max/2);
        analogWrite(leftF, max);
        sensorAktiv = true;
    }
    else if (abs(sensorValues[5] - startValues[5]) > minDiff) {
        Serial.println("Sensor 5 aktiv: Kjører motorer");
        analogWrite(rightF, max);
        analogWrite(leftF, max/1.5);
        sensorAktiv = true;
    }
    else if (abs(sensorValues[7] - startValues[7]) > minDiff) {
        Serial.println("Sensor 7 aktiv: Kjører motorer");
        analogWrite(rightF, max/1.5);
        analogWrite(leftF, max);
        sensorAktiv = true;
    }
    else if (abs(sensorValues[6] - startValues[6]) > minDiff) {
        Serial.println("Sensor 6 aktiv: Kjører motorer");
        analogWrite(rightF, max);
        analogWrite(leftF, max);
        sensorAktiv = true;
    }

    // Hvis ingen sensorer er aktive, stopp motorene
    if (!sensorAktiv) {
        Serial.println("Ingen sensorer er aktive: Stopper motorene");
        analogWrite(rightF, 0);
        analogWrite(leftF, 0);
        analogWrite(leftB, 0);
        analogWrite(rightB, 0);
    }
}

void loop() {
    linjeOgMotor();  // Kjør funksjonen for å sjekke sensorverdier og motorstyring

    delay(100);  // Kort forsinkelse for å unngå overbelastning
}
