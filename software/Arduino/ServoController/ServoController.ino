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

// ================ SYSTEM PARAMETERS ===================
// Sample rates
int motorUpdateRate = 50; // gyro sample rate
int calibrationSamples = 1000;

// Initial calibration value
float calibratedOffset = 0;

// Indicator LED
const int LED = 13;

// These values will need to be experimentally found using
// a servo
const float RPM = 1.5;
const float CLOCK_WISE_SPEED = 40;
const float COUNTER_CLOCK_WISE_SPEED = 100;
const float ZERO_SPEED = 75;
// =======================================================

// ============ Initialize electronic interfaces =========
Servo azimuth;
LSM9DS1 imu;

// ============ Runtime variables ========================
// Orientation data
float positionChange;

// Misc.
int timer = millis();
bool update_flag = false; // interrupt flag for executing motor update

// Initialize everything
void setup() {
  // Initialize I/O
  Serial.begin(115200);
  pinMode(LED, OUTPUT);

  // Setup hardware
  setupIMU();         // Gyro
  azimuth.attach(3);

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
    rotateServoBy(correctionFactor);
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
    rotateServoBy(-positionChange);
    positionChange = 0;
    // Reset flag
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
void rotateServoBy(float deg) {
  float wait = abs((deg / 360) / RPM);
  if (deg < 0) azimuth.write(COUNTER_CLOCK_WISE_SPEED);
  else azimuth.write(CLOCK_WISE_SPEED);
  delay(wait);
  azimuth.write(ZERO_SPEED);
}

float readGyro() {
  if (imu.gyroAvailable()) {
    imu.readGyro();
  }
  return imu.calcGyro(imu.gz) - calibratedOffset;
}

void blinkLED(int del, int n) {
  for (int i = 0; i < n; i++) {
    digitalWrite(LED, HIGH);
    delay(del);
    digitalWrite(LED, LOW);
    delay(del);
  }
}

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

