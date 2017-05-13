/**********************************************************************
 * Arduino Uno Program - Controls Servos and Reads IMU                *
 * Project: Eclipse Tracking (2017)                                   *
 * Version: 3.0 (5/07/2017)                                           *
 * Max Bowman / Jeremy Seeman / George Moe                            *
 **********************************************************************/
#include <Stepper.h>
#include <Wire.h>
#include <MsTimer2.h>
#include <SparkFunLSM9DS1.h>
#include "config.h"

// ================ SYSTEM PARAMETERS ===================
// Sample rates
int motorUpdateRate = 50; // gyro sample rate
int calibrationSamples = 1000;

// Initial calibration value
float calibratedOffset = 0;

// Indicator LED
const int LED = 13;

const byte GEAR_RATIO = 1; // servo gear : slip gear

// ============ Initialize electronic interfaces =========
LSM9DS1 imu;

int stepperPins[] = {9, 10, 11, 12};
int numSteps = 200;
float stepSize = 9.0 / 5.0;

Stepper azimuth(numSteps, stepperPins[0], stepperPins[1], stepperPins[2], stepperPins[3]);
// ============ Runtime variables ========================
// Orientation data
float positionChange;

// Misc.
int timer = millis();
bool update_flag = false; // interrupt flag for executing motor update

/**
    Sets up everything
*/
void setup() {
  // Initialize I/O
  Serial.begin(115200);
  pinMode(LED, OUTPUT);

  // Setup hardware
  setupIMU();         // Gyro

  // Calibrate the gyro
  calibratedOffset = zeroGyro(calibrationSamples);

  // Setup motor update interval timer
  MsTimer2::set(motorUpdateRate, updateMotor);
  MsTimer2::start();

  // Blink LED to indicate that initilization has completed!
  blinkLED(20, 10);
}

void loop() {

  // Accept corrections from serial
  if (Serial.available() == 4) {
    int correctionFactor = 0;
    byte negative = 0;
    if (Serial.read() == '1') negative = 1;
    for (int i = 100; i >= 1; i /= 10) {
      correctionFactor += (Serial.read() - '0') * 100;
    }
    if (negative) correctionFactor = -correctionFactor;
    rotateStepperBy(correctionFactor);
  }

  // Update orientation data with measurements from gyro
  // Just using angular rates from gyro and time deltas to track position using a Reimann sum.
  int currentTime = millis();
  int deltaT = currentTime - timer;
  positionChange += deltaT * readGyro() / 1000; // Gyro data is given in 1000ths of a degree, so we divide by 1000
  timer = currentTime;

  // Update motor position
  if(update_flag) {
    // Update stepper position (Z/AZM)
    rotateStepperBy(-positionChange);
    positionChange = 0;
    // Reset flag
    update_flag = false;
  }
}

/**
    Triggered every motorUpdateRate milliseconds
    Starts the motor update
*/
void updateMotor() {
  update_flag = true;
}

/**
    Sets up the IMU
*/
void setupIMU() {
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
  while (!imu.begin()) {
    blinkLED(500, 1);
  }
}

/**
    Rotates a stepper motor a specified number
    of degrees.
    @param degrees to rotate the stepper motor
*/
int rotateStepperBy(float deg) {
  int steps = deg / stepSize;
  azimuth.step(steps);  
  return steps;
}

/**
    Reads the IMU's gyro
    @return rate in degrees / ms
*/
float readGyro() {
  if (imu.gyroAvailable()) {
    imu.readGyro();
  }
  return imu.calcGyro(imu.gz) - calibratedOffset;
}

/**
    Blinks an LED on pin 13
    @param delay in milliseconds
    @param number of blinks
*/
void blinkLED(int del, int n) {
  for (int i = 0; i < n; i++) {
    digitalWrite(LED, HIGH);
    delay(del);
    digitalWrite(LED, LOW);
    delay(del);
  }
}

/**
    Finds the calibration constant of the
    gyro.
    @param number of calibration samples
*/
int zeroGyro(int calibrationSamples) {
  float zSum = 0;
  for(int i = 0; i < calibrationSamples; i++) {
    if(i % 1000 == 0) {
      blinkLED(500, 1);
    }
    imu.readGyro();
    float z = imu.calcGyro(imu.gz);
    zSum += z;
    delay(10);
  }
  return zSum / calibrationSamples;
}
