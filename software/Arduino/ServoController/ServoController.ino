/**********************************************************************
 * Arduino Uno Program - Controls Servo and Reads Sensors             *
 * Project: Eclipse Tracking (2017)                                   *
 * Version: 1.0 (2/18/2017)                                           *
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
float entries[10]; // rolling average array
int j = 0; // for maintaining the rolling average array
float runningSum = 0; // keep a running sum to make average algorithm O(1)
float oldAverage; // to compute angular movement
int servoVal = 90; // initial value to set servo at
bool int_flag = false; // interrupt flag for executing gyro read every 100 ms
const int LED = 13; // indicator LED
void setup() {
  Serial.begin(115200);

  pinMode(LED, OUTPUT);
  setupIMU(); // set up hardware

  for (int i = 0; i < 10; i++) {
    entries[i] = readGyro();
    runningSum += entries[i];
  } // fill up the rolling average

  oldAverage = getAverage();
  
  xy.attach(3); // pro mini / uno pwm pin
  xy.write(servoVal); // Start at a 90 degrees

  blinkLED(50, 10); // Let the use know everything initialized

  MsTimer2::set(100, a);
  MsTimer2::start(); // set up and start a timer interrupt
  
}

void loop() {
  if (int_flag) {
    j %= 10;
    runningSum -= entries[j];
    entries[j] = readGyro();
    runningSum += entries[j];
    float newAverage = getAverage();
    servoVal += newAverage - oldAverage;
    xy.write(servoVal);
    oldAverage = newAverage;
    j++;
    int_flag = false;
  }
}

void a() {
  int_flag = true;
}

void setupIMU() {
  Serial.print("Setting up IMU...");
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
  if (!imu.begin()) {
    Serial.println(" [ FATAL ERROR ] ");
    Serial.println(" There was an error connecting to the IMU.");
    Serial.println(" Please check connections.");
    Serial.println(" --- Connection Diagram --- ");
    Serial.println(" Arduino |    IMU   ");
    Serial.println("--------------------");
    Serial.println(" 3.3V    | 1.9-3.6 V");
    Serial.println(" GND     | GND");
    Serial.println(" A4      | SDA");
    Serial.println(" A5      | SCL");
    while (!imu.begin()) {
      Serial.println(" [ FAILED ] ");
      blinkLED(500, 1);
      Serial.print("Connecting...");
      delay(1000);
    }
    Serial.println("[ SUCCESS ] ");
  }
  Serial.println(" [ DONE ] ");
  
}

float readGyro() {
  if (imu.gyroAvailable()) {
    imu.readGyro();
  }
  return imu.calcGyro(imu.gz);
}

void blinkLED(int del, int n) {
  for (int i = 0; i < n; i++) {
    digitalWrite(13, HIGH);
    delay(del);
    digitalWrite(13, LOW);
    delay(del);
  }
}

float getAverage() {
  return runningSum / 10;
}

