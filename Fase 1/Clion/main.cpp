//Tried to implement an average centered way of controlling the motor forces, this might not work since I haven't tested it

#include <Arduino.h>
#include <QTRSensors.h>
#define sensorCount 13

const int left = 2;
const int right = 15;
const int minDiff = 40;
const int turning_speed = 200;


uint16_t last_powerful_force[2];

// Opprett et QTR-sensorobjekt og arrays for sensordata
QTRSensors qtr;
uint16_t sensorValues[sensorCount];
uint16_t startValues[sensorCount];

// Motorhastigheter for hver sensor
const int motorSpeeds[][2] = {
    {0, 255},    // Sensor 25, motorjustering for venstre og høyre
    {100, 200},  // Sensor 23
    {120, 200},  // Sensor 21
    {130, 200},  // Sensor 19
    {140, 200},  // Sensor 17
    {150, 200},  // Sensor 15
    {160, 200},  // Sensor 13
    {170, 200},  // Sensor 11
    {255, 255},  // Sensor 9
    {200, 170},  // Sensor 7
    {200, 160},  // Sensor 5
    {200, 150},  // Sensor 3
    {200, 140},  // Sensor 1
};

// Definer sensorpinnene
const uint8_t sensorPins[sensorCount] = {32, 33, 25, 26, 27, 14, 12, 13, 23, 22, 21, 19, 18};
//                    Sensor-pinner:     25  23  21  19  17  15  13  11   9   7   5   3   1
void setup() {
    Serial.begin(9600);
    pinMode(left, OUTPUT);
    pinMode(right, OUTPUT);

    for (uint8_t i = 0; i < sensorCount; i++) {
        pinMode(sensorPins[i], INPUT);
    }

    qtr.setTypeRC();
    qtr.setSensorPins(sensorPins, sensorCount);
    qtr.read(startValues);  // Les og lagre startverdiene ved oppstart

    Serial.println("Sensor setup complete. Startverdier lagret.");
}

double transform_sensor_speed_force(double speed_value, int sensor_activation_ind, double reduction_power=10){
  return ((double) speed_value / ( (double) sensor_activation_ind) * (double) reduction_power);
}

// Kombinert funksjon for sensoravlesning og motorstyring
void linjeOgMotor() {
    int sisteAktiveSensor = -1;  // Variabel for å spore den nyligste aktive sensoren

    qtr.read(sensorValues);  // Les alle sensorverdiene


    int total_right_force = 0;
    int total_left_force = 0;
    int total_active_sensors = 0;

    // Gå gjennom sensorene for å finne den nyligste aktive sensoren
    for (uint8_t i = 0; i < sensorCount; i++) {
        if ( ( abs(sensorValues[i] - startValues[i]) > minDiff) ) {
            sisteAktiveSensor = i;  // Oppdater med den nyligst aktive sensoren

            total_active_sensors += 1;

            total_left_force += transform_sensor_speed_force(motorSpeeds[i][0], total_active_sensors);
            total_right_force += transform_sensor_speed_force(motorSpeeds[i][1], total_active_sensors);
        }
    }


    // Hvis en aktiv sensor ble funnet, juster motorene
    if (total_active_sensors > 0) {
        double average_left_force = total_left_force / total_active_sensors;
        double average_right_force = total_right_force / total_active_sensors;

        analogWrite(left, average_left_force);   // Bruk motorhastigheten for den nyligst aktive sensoren
        analogWrite(right, average_right_force);  // Bruk motorhastigheten for den nyligst aktive sensoren

        if (average_left_force > average_right_force){
            last_powerful_force[0] = 1;
            last_powerful_force[1] = 0;
        } else {
            last_powerful_force[0] = 0;
            last_powerful_force[1] = 1;
        }

    } else {
        // Hvis ingen sensor har oppdaget en linje, turn the bot the opposite direction to refind the line.
        analogWrite(left, last_powerful_force[1] * turning_speed);
        analogWrite(right, last_powerful_force[0] * turning_speed);
    }
}

void loop() {
    linjeOgMotor();  // Kjør linjefølger og motorstyring i én funksjon

    // Valgfri: Skriv ut sensorverdier for feilsøking
   /* for (uint8_t i = 0; i < sensorCount; i++) {
        Serial.print("S");
        Serial.print(sensorPins[i]);  // Skriv ut sensornummeret basert på sensorPins[]-arrayet
        Serial.print(":");
        Serial.print(sensorValues[i]);
        Serial.print(" St:");
        Serial.print(startValues[i]);
        Serial.print(" D:");
        Serial.print(sensorValues[i] - startValues[i]);
        Serial.print(" ");
    }

    delay(1000);  // Vent ett sekund mellom hver syklus
    Serial.println();
    Serial.println();*/
}