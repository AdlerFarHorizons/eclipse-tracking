/**********************************************************************
 * Arduino Uno Program - Controls Servo and Reads Sensors             *
 * Project: Eclipse Tracking (2017)                                   *
 * Version: 3/04/2017                                                 *
 * Max Bowman / Jeremy Seeman                                         *
 **********************************************************************/
#include <Servo.h>
#include <Wire.h>
#include <MsTimer2.h>
#include <SparkFunLSM9DS1.h>
#include "config.h"

// Declare global variables
LSM9DS1 imu; // imu chip
Servo xy; // x-y plane servo
int servoVal = 90; // initial value to set servo at
float oldHeading = 90;
float newHeading;
bool int_flag = false; // interrupt flag for executing gyro read every 100 ms
const int LED = 13; // indicator LED - subject to change (depends on board)
int samplingRate = 30;
int motorFlag = 0;

void setup() {
  Serial.begin(11520);
  pinMode(LED, OUTPUT);
  setupIMU(); // set up hardware
  
  xy.attach(3); // pro mini / uno pwm pin
  xy.write(servoVal); // Start at a 90 degrees

  blinkLED(50, 10); // Let the use know everything initialized

  MsTimer2::set(samplingRate, a);
  MsTimer2::start(); // set up and start a timer interrupt to run every 100 ms
  
}

void loop() {
  if (int_flag) {
    newHeading = getHeading();
    servoVal += newHeading - oldHeading;
    oldHeading = newHeading;
    int_flag = false;
    motorFlag++;
  }
  motorFlag %= 4;
  if (motorFlag == 0) {
    xy.write(servoVal);
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
      // could not connect - pulse the LED every 250 ms until the imu connects
      blinkLED(250, 1);
  }
}

float getHeading() {
  float result = 0;
  if (imu.magAvailable()) {
    imu.readMag();
  }
  float magY = imu.my;
  float magX = imu.mx;
  if (magY == 0) {
    // if magnetometer y vector is 0, then the heading is either 180 degrees or 0 degrees
    if (magX < 0) result = PI;
    // otherwise, start at 0.
  }
  else {
    result = atan2(magX, magY); // illustration for why is on elgg
  }
  result -= DECLINATION * PI / 180.0; // convert declination to degrees and subtract it from the result

  // adjust values to appropriate range
  if (result > PI) result -= 2 * PI;
  else if (result < -PI) result += 2 * PI;
  else if (result < 0) result += 2 * PI;

  result *= 180.0 / PI; // convert result to degrees
  return result;
}

void blinkLED(int del, int n) {
  for (int i = 0; i < n; i++) {
    digitalWrite(LED, HIGH);
    delay(del);
    digitalWrite(LED, LOW);
    delay(del);
  }
}

