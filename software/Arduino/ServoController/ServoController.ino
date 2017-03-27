/**********************************************************************
 * Arduino Uno Program - Controls Servo and Reads IMU                 *
 * Project: Eclipse Tracking (2017)                                   *
 * Version: 1.0 (3/27/2017)                                           *
 * Max Bowman / Jeremy Seeman / George Moe                            *
 **********************************************************************/
#include <Stepper.h>
#include <Servo.h>
#include <Wire.h>
#include <MsTimer2.h>
#include <SparkFunLSM9DS1.h>
#include "config.h"

// Declare global variables
LSM9DS1 imu; // imu chip
int stepperPins[] = {9, 10, 11, 12};
int stepperSteps = 200;
Stepper motor(stepperSteps, stepperPins[0], stepperPins[1], stepperPins[2], stepperPins[3]);
Servo altitude;

int n = 10; // gyro sample
int k = 200; // servo sample
int j = 0; // keep track of what to sample
float runningSumXY = 0;
float runningSumZ = 0;

float currentServoVal = 90;

bool int_flag = false; // interrupt flag for executing gyro read every 100 ms
const int LED = 13; // indicator LED

void setup() {
  Serial.begin(115200);
  pinMode(LED, OUTPUT);
  setupIMU(); // set up hardware
  motor.setSpeed(10);
  altitude.attach(3);

  blinkLED(20, 10); // Let the use know everything initialized

  MsTimer2::set(n, a); // accepts ms argument
  MsTimer2::start(); // set up and start a timer interrupt  
}

void loop() {
  if (Serial.available() == 8) {
    int correctionFactorXY = 0;
    int correctionFactorZ = 0;
    byte negXY = 0;
    byte negZ = 0;
    if (Serial.read() == '1') negXY = 1;
    correctionFactorXY += (Serial.read() - '0') * 100;
    correctionFactorXY += (Serial.read() - '0') * 10;
    correctionFactorXY += (Serial.read() - '0');
    if (negXY) correctionFactorXY = -correctionFactorXY;
    rotateStepperBy(correctionFactorXY);
    if (Serial.read() == '1') negZ = 1;
    correctionFactorZ += (Serial.read() - '0') * 100;
    correctionFactorZ += (Serial.read() - '0') * 10;
    correctionFactorZ += (Serial.read() - '0');
    if (negZ) correctionFactorZ = -correctionFactorZ;
    altitude.write(currentServoVal + correctionFactorZ);
    currentServoVal += correctionFactorZ;
  }
  if (int_flag) {
    j++;
    j %= k / n + 1;
    runningSumXY += n * readGyroZ() / 1000;
    runningSumZ += n * readGyroY() / 1000;
    if (j == 0) {
      rotateStepperBy(-runningSumXY); // runningSumXY is the total theta change on the Z axis of rotation
      runningSumXY = 0;
      altitude.write(currentServoVal + runningSumZ); // runningSumZ is the total theta change on the Y axis of rotation
      currentServoVal += runningSumZ;
      runningSumZ = 0;
    }
  }
}

void a() {
  int_flag = true;
}

void setupIMU() {
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
  while (!imu.begin()) {
    blinkLED(500, 1);
  }
}

// For stepper motors
void rotateStepperBy(float deg) {
  int steps = deg*5/9;
  motor.step(steps);  
}

float readGyroZ() {
  if (imu.gyroAvailable()) {
    imu.readGyro();
  }
  return imu.calcGyro(imu.gz);
}

float readGyroY() {
  if (imu.gyroAvailable()) {
    imu.readGyro();
  }
  return imu.calcGyro(imu.gy);
}

void blinkLED(int del, int n) {
  for (int i = 0; i < n; i++) {
    digitalWrite(LED, HIGH);
    delay(del);
    digitalWrite(LED, LOW);
    delay(del);
  }
}
