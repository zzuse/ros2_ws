#include "Wire.h"
#include <Arduino.h>
#include <MPU6050_light.h>

#define TRIG 27
#define ECHO 21

MPU6050 mpu(Wire);

void setup()
{
    // put your setup code here, to run once:
    Serial.begin(115200);
    pinMode(2, OUTPUT);
    pinMode(TRIG, OUTPUT);
    pinMode(ECHO, INPUT);
    Wire.begin(18, 19);

    byte status = mpu.begin();
    Serial.print(F("MPU6050 status: "));
    Serial.println(status);
    while (status != 0) {
    } // stop everything if could not connect to MPU6050

    Serial.println(F("Calculating offsets, do not move MPU6050"));
    delay(1000);
    mpu.calcOffsets(true, true); // gyro and accelero
    Serial.println("MPU6050 Done!\n");
}

unsigned long timer = 0;

void loop()
{
    // MPU
    mpu.update();

    if (millis() - timer > 1000) { // print data every second
        Serial.print(F("TEMPERATURE: "));
        Serial.println(mpu.getTemp());
        Serial.print(F("ACCELERO  X: "));
        Serial.print(mpu.getAccX());
        Serial.print("\tY: ");
        Serial.print(mpu.getAccY());
        Serial.print("\tZ: ");
        Serial.println(mpu.getAccZ());

        Serial.print(F("GYRO      X: "));
        Serial.print(mpu.getGyroX());
        Serial.print("\tY: ");
        Serial.print(mpu.getGyroY());
        Serial.print("\tZ: ");
        Serial.println(mpu.getGyroZ());

        Serial.print(F("ACC ANGLE X: "));
        Serial.print(mpu.getAccAngleX());
        Serial.print("\tY: ");
        Serial.println(mpu.getAccAngleY());

        Serial.print(F("ANGLE     X: "));
        Serial.print(mpu.getAngleX());
        Serial.print("\tY: ");
        Serial.print(mpu.getAngleY());
        Serial.print("\tZ: ");
        Serial.println(mpu.getAngleZ());
        Serial.println(F("=====================================================\n"));
        timer = millis();
    }

    // Ultrasonic Sensor
    digitalWrite(TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG, LOW);
    double duration = pulseIn(ECHO, HIGH);
    float distance = duration * 0.034 / 2; // 340 m/s = 0.034 cm/µs
    Serial.printf("Distance: %f cm\n", distance);

    // LED
    digitalWrite(2, LOW);
    delay(1000);
    digitalWrite(2, HIGH);
    delay(1000);
}