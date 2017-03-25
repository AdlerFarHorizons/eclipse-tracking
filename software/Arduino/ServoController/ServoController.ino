/**********************************************************************
 * Arduino Uno Program - Controls Servo and Reads IMU                 *
 * Project: Eclipse Tracking (2017)                                   *
 * Version: 1.0 (3/18/2017)                                           *
 * Max Bowman / Jeremy Seeman / George Moe                            *
 **********************************************************************/
#include <Stepper.h>
#include <Wire.h>
#include <MsTimer2.h>
#include <SparkFunLSM9DS1.h>
#include "config.h"

// Declare global variables
LSM9DS1 imu; // imu chip
int stepperPins[] = {9, 10, 11, 12};
int stepperSteps = 200;
Stepper motor(stepperSteps, stepperPins[0], stepperPins[1], stepperPins[2], stepperPins[3]);

int n = 10; // gyro sample
int k = 200; // servo sample
int j = 0; // keep track of what to sample
float runningSum = 0;

bool int_flag = false; // interrupt flag for executing gyro read every 100 ms
const int LED = 13; // indicator LED

char ch[4]; // for receiving commands from Raspberry Pi 3

void setup() {
  Serial.begin(115200);
  pinMode(LED, OUTPUT);
  setupIMU(); // set up hardware
  motor.setSpeed(10);

  blinkLED(20, 10); // Let the use know everything initialized

  MsTimer2::set(n, a); // accepts ms argument
  MsTimer2::start(); // set up and start a timer interrupt
  
}

void loop() {
  if (Serial.available() == 4) {
    int correctionFactor = 0;
    int neg = 0;
    if (Serial.read() == '1') neg = 1;
    correctionFactor += (Serial.read() - '0') * 100;
    correctionFactor += (Serial.read() - '0') * 10;
    correctionFactor += (Serial.read() - '0');
    if (neg) correctionFactor -= 2 * correctionFactor;
    rotateStepperBy(correctionFactor);
  }
  if (int_flag) {
    j++;
    j %= k / n + 1;
    runningSum += n * readGyro() / 1000;
    if (j == 0) {
      rotateStepperBy(-runningSum); // runningSum is the total theta change
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

// For stepper motors
void rotateStepperBy(float deg) {
  int steps = deg*5/9;
  motor.step(steps);  
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
