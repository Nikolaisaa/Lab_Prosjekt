#include <QTRSensors.h>

#define sensorCount 21

const int left = 1;
const int right = 2;

QTRSensors qtr;                      // Opprett et QTR-sensorobjekt
uint16_t sensorValues[sensorCount];  // Array for å lagre sensorverdier

void setup() {
  // Definer sensorpinnene
  const uint8_t sensorPins[sensorCount] = {13, 12, 14, 27, 26, 25, 33, 32, 17, 3, 4, 16, 17, 18, 20, 21, 22, 5, 22, 23, 15};
  //                                       1   2   3   4    5   6   7   8  9  10  11 12  13  14  15  16  17  18 19  20  21
  // Start kommunikasjon med Serial Monitor
  Serial.begin(115200);

  // Sett alle sensorpinner til INPUT
  for (uint8_t i = 0; i < sensorCount; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  // Konfigurer sensorene
  qtr.setTypeRC();                             // Sett sensorene til RC-typen
  qtr.setSensorPins(sensorPins, sensorCount);  // Sett sensorpinnene og antallet sensorer

  Serial.println("Sensor setup complete.");
}

void loop() {
  // Les verdier fra sensorene
  qtr.read(sensorValues);  // Les sensorverdiene

  for (uint8_t i = 0; i < sensorCount; i++) {
    Serial.print("Sensor ");
    Serial.print(i + 1);  // Skriv ut sensornummer (starter fra 1)
    Serial.print(": ");
    Serial.print(sensorValues[i]);  // Skriv ut sensorverdi
    Serial.print("\t");             // Legg til en tab for å formatere
  }
  Serial.println();  // Ny linje etter alle sensorverdiene

  delay(1000);  // Vent ett sekund mellom hver lesing
}
