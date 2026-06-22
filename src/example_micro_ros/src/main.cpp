#include "Wire.h"
#include <Arduino.h>
#include <Esp32McpwmMotor.h>
#include <Esp32PcntEncoder.h>
#include <Kinematics.h>
#include <MPU6050_light.h>
#include <PidController.h>

#define TRIG 27
#define ECHO 21

MPU6050 mpu(Wire);
Esp32McpwmMotor motor;
Esp32PcntEncoder encoders[2];
PidController pid[2];
Kinematics kinematics;

// Rotate
float target_linear_speed = 20.0; // mm/s
float target_angular_speed = 0.1; // rad/s
float out_left_speed = 0.0;
float out_right_speed = 0.0;

unsigned long timer = 0;

void setup()
{
    Serial.begin(115200);
    pinMode(2, OUTPUT);
    pinMode(TRIG, OUTPUT);
    pinMode(ECHO, INPUT);
    Wire.begin(18, 19); // SDA, SCL

    byte status = mpu.begin();
    Serial.print(F("MPU6050 status: "));
    Serial.println(status);
    while (status != 0) {
    } // stop everything if could not connect to MPU6050

    Serial.println(F("Calculating offsets, do not move MPU6050"));
    delay(1000);
    mpu.calcOffsets(true, true); // gyro and accelero
    Serial.println("MPU6050 Done!\n");

    motor.attachMotor(0, 22, 23); // dir, cdir
    motor.attachMotor(1, 12, 13); // dir, cdir
    encoders[0].init(0, 32, 33);  // A, B
    encoders[1].init(1, 26, 25);  // A, B

    // pid controller
    pid[0].update_pid(0.625, 0.125, 0.00);
    pid[1].update_pid(0.625, 0.125, 0.00);
    pid[0].out_limit(-100, 100);
    pid[1].out_limit(-100, 100);
    // 初始化运动学参数
    kinematics.set_wheel_distance(175); // mm
    // where 10 laps ticks 19766
    // 67mm is the wheel diameter
    // distance per tick = 3.141593*67/1976 = 0.10657556 mm
    kinematics.set_motor_param(0, 0.10657556);
    kinematics.set_motor_param(1, 0.10657556);
    kinematics.kinematics_inverse(target_linear_speed, target_angular_speed, &out_left_speed, &out_right_speed);
    Serial.printf("OUT:left_speed=%f, right_speed=%f\n", out_left_speed, out_right_speed);
    pid[0].update_target(out_left_speed);
    pid[1].update_target(out_right_speed);
}

void readMPU6050()
{
    // MPU IMU sensor
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
}

void readLED()
{
    // LED
    digitalWrite(2, LOW);
    delay(1000);
    digitalWrite(2, HIGH);
    delay(1000);
}

void readUltrasonicSensor()
{
    // Ultrasonic Sensor
    digitalWrite(TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG, LOW);
    double duration = pulseIn(ECHO, HIGH);
    float distance = duration * 0.034 / 2; // 340 m/s = 0.034 cm/µs
    Serial.printf("Distance: %f cm\n", distance);
}

void loop()
{
    // readMPU6050();
    // readLED();
    readUltrasonicSensor();
    // Motor
    delay(10);
    kinematics.update_motor_speed(millis(), encoders[0].getTicks(), encoders[1].getTicks());
    motor.updateMotorSpeed(0, pid[0].update(kinematics.get_motor_speed(0)));
    motor.updateMotorSpeed(1, pid[1].update(kinematics.get_motor_speed(1)));
    Serial.printf("x, y, yaw=%f, %f, %f\n", kinematics.get_odom().x, kinematics.get_odom().y,
                  kinematics.get_odom().angle);
}