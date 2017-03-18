/**********************************************************************
 * Arduino Uno Program - Controls Servo and Reads IMU                 *
 * Project: Eclipse Tracking (2017)                                   *
 * Version: 1.0 (3/18/2017)                                           *
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

int n = 10; // gyro sample
int k = 200; // servo sample
int j = 0; // keep track of what to sample
float runningSum = 0;

float oldVal = 90;
float servoVal = 90; // initial value to set servo at
bool int_flag = false; // interrupt flag for executing gyro read every 100 ms
const int LED = 13; // indicator LED

void setup() {
  Serial.begin(9600);
  pinMode(LED, OUTPUT);
  setupIMU(); // set up hardware

  blinkLED(20, 10); // Let the use know everything initialized

  xy.attach(3); // pro mini / uno pwm pin
  xy.write(servoVal); // Start at a 90 degrees

  MsTimer2::set(n, a); // accepts ms argument
  MsTimer2::start(); // set up and start a timer interrupt
  
}

void loop() {
  if (int_flag) {
    j++;
    j %= k / n + 1;
    runningSum += n * readGyro() / 1000;
    if (j == 0) {
      oldVal += runningSum;
      xy.write(oldVal);
      runningSum = 0;
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

// For continous rotation servos
void rotateTo(float deg) {
  
}

float readGyro() {
  if (imu.gyroAvailable()) {
    imu.readGyro();
  }
  return imu.calcGyro(imu.gz);
}

void blinkLED(int del, int n) {
  for (int i = 0; i < n; i++) {
    digitalWrite(LED, HIGH);
    delay(del);
    digitalWrite(LED, LOW);
    delay(del);
  }
}
