/**********************************************************************
 * Arduino Uno Program - Controls Servos and Reads IMU                *
 * Project: Eclipse Tracking (2017)                                   *
 * Version: 2.0 (4/15/2017)                                           *
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

int motorUpdateRate = 50; // gyro sample rate
int sampleCounter = 0; // keep track of what to sample
int timer = millis();
float runningSumAlt = 0;
float runningSumZ = 0;
float currentYBodyPosition = 90;
float currentZBodyPosition = 0;
float currentZMotorPosition = 0;
int calibrationSamples = 1000;
float calibratedYOffset = 0;
float calibratedZOffset = 0;

bool update_flag = false; // interrupt flag for executing motor update
const int LED = 13; // indicator LED

void setup() {
  Serial.begin(115200);
  pinMode(LED, OUTPUT);
  setupIMU(); // set up hardware
  motor.setSpeed(10);
  altitude.attach(3);

  altitude.write(currentYBodyPosition);
  zeroGyro();

  blinkLED(20, 10); // Let the use know everything initialized

  MsTimer2::set(motorUpdateRate, updateMotor); // accepts ms argument
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
    currentZBodyPosition += correctionFactorXY;
    currentZMotorPosition += correctionFactorXY;
    if (Serial.read() == '1') negZ = 1;
    correctionFactorZ += (Serial.read() - '0') * 100;
    correctionFactorZ += (Serial.read() - '0') * 10;
    correctionFactorZ += (Serial.read() - '0');
    if (negZ) correctionFactorZ = -correctionFactorZ;
    altitude.write(currentYBodyPosition + correctionFactorZ);
    currentYBodyPosition += correctionFactorZ;
    blinkLED(500, correctionFactorZ);
  }

  int currentTime = millis();
  int deltaT = currentTime - timer;
  currentZBodyPosition += deltaT * readGyroZ() / 1000;
  currentYBodyPosition += deltaT * readGyroY() / 1000;
  timer = currentTime;

  if(update_flag) {
    altitude.write(currentYBodyPosition);
    rotateStepperBy(currentZBodyPosition - currentZMotorPosition);
    currentZMotorPosition = currentZBodyPosition;
    sampleCounter = 0;
    update_flag = false;
  }
}

void updateMotor() {
  update_flag = true;
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
  return imu.calcGyro(imu.gz) - calibratedZOffset;
}

float readGyroY() {
  if (imu.gyroAvailable()) {
    imu.readGyro();
  }
  return imu.calcGyro(imu.gy) - calibratedYOffset;
}

void blinkLED(int del, int n) {
  for (int i = 0; i < n; i++) {
    digitalWrite(LED, HIGH);
    delay(del);
    digitalWrite(LED, LOW);
    delay(del);
  }
}

void zeroGyro() {
  float ySum = 0;
  float zSum = 0;

  for(int i = 0; i < calibrationSamples; i++) {

    if(i%100==0) {
      digitalWrite(LED, HIGH);
    }
    else if(i%100==10) {
      digitalWrite(LED, LOW);
    }
    
    imu.readGyro();
    float y = imu.calcGyro(imu.gy);
    float z = imu.calcGyro(imu.gz);
    ySum += y;
    zSum += z;
    delay(10);
  }

  calibratedYOffset = ySum/calibrationSamples;
  calibratedZOffset = zSum/calibrationSamples;
}

