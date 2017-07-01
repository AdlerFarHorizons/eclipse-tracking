/***********************************************************************
 * Description: Counteracts a change in angle by reading a gyro sensor *
 * and rotating a stepper motor. Accepts correction factors via serial *
 * at 115200 baud.                                                     *
 * Project: Eclipse Tracking (2017)                                    *
 * @Version 6/02/2017                                                  *
 * @Author Max Bowman / Jeremy Seeman / George Moe                     *
 ***********************************************************************/
#include <Stepper.h>
#include <Wire.h>
#include <MsTimer2.h>
#include <SparkFunLSM9DS1.h>
#include "config.h"

// ================ SYSTEM PARAMETERS ===================
// Sample rates
const int motorUpdateRate = 50; // gyro sample rate
const int calibrationSamples = 1000;

// Initial calibration value
float calibratedOffset;

// Indicator LED
const int LED = 13;

const byte GEAR_RATIO = 1; // servo gear : slip gear

// ============ Initialize electronic interfaces =========
LSM9DS1 imu;

int stepperPins[] = {9, 10, 11, 12};
int numSteps = 200;
int stepperRPM = 40;
float stepSize = 1.8;

Stepper azimuth(numSteps, stepperPins[0], stepperPins[1], stepperPins[2], stepperPins[3]);
// ============ Runtime variables ========================
// Orientation data
float positionChange;
float correctionSum;

// Interrupt flag
int timer = millis();
bool update_flag = false; // interrupt flag for executing motor update
int ref;
/**
    Sets up system
*/
void setup() {
  // Initialize I/O
  Serial.begin(115200);

  // Setup hardware
  pinMode(LED, OUTPUT);
  azimuth.setSpeed(stepperRPM);
  setupIMU(); // Gyro

  // Calibrate the gyro
  calibratedOffset = zeroGyro(calibrationSamples);

  // Find the sun to center on
  findSun(); // comment this line out for basic testing
  
  // Setup motor update interval timer
  MsTimer2::set(motorUpdateRate, updateMotor);
  MsTimer2::start();

  ref = millis();
}

void loop() {

  // Accept corrections from serial
  if (Serial.available() == 4) {
    float correctionFactor = 0;
    bool negative = false;
    if (Serial.read() == '1') negative = true;
    for (int i = 100; i >= 1; i /= 10) {
      correctionFactor += (float)(Serial.read() - '0') * i;
    }
    if (negative) correctionFactor *= -1;
    rotateStepperBy(correctionFactor);
    positionChange = 0; // reset gyro based corrections
  }

  // Update orientation data with measurements from gyro
  // Just using angular rates from gyro and time deltas to track position using a Reimann sum.
  int currentTime = millis();
  
  int deltaT = currentTime - timer;
  positionChange += deltaT * (readGyro()) / 1000; // Gyro data is given in 1000ths of a degree, so we divide by 1000
  timer = currentTime;

  // Update motor position
  if(update_flag) {
    // Update stepper position (Z/AZM)
    float degreesActuallyCorrected = rotateStepperBy(-positionChange);
    positionChange += degreesActuallyCorrected;
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
    blinkLED(1000, 1);
  }
  blinkLED(50, 15);
}

/**
    Rotates a stepper motor by 1.8 degrees until
    it recieves a byte from the Raspberry Pi 3
    indicating that it found the sun
*/
void findSun() {
  while (Serial.available() == 0) {
    azimuth.step(1);
    delay(500);
  }
  Serial.read();
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
    Finds the calibration constant of the
    gyro.
    @param number of calibration samples
    @returns degrees of gyro drift per second
*/
float zeroGyro(int calibrationSamples) {
  float gyroSum = 0;
  digitalWrite(LED, HIGH);
  for (int i = 0; i < calibrationSamples; i++) {
    gyroSum += readGyro();
  }
  blinkLED(300, 5);
  Serial.println(gyroSum / calibrationSamples);
  return gyroSum / calibrationSamples;
}

/**
    Rotates a stepper motor a specified number
    of degrees.
    @param degrees to rotate the stepper motor
    @return number of degrees actually rotated given precision
    of the stepper motor
*/
float rotateStepperBy(float deg) {
  int steps = deg / stepSize * GEAR_RATIO;
  azimuth.step(steps);  
  return steps * stepSize;
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
