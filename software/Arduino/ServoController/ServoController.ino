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

// Initial calibration values
float calibratedYOffset = 0;
float calibratedZOffset = 0;

// Stepper motor
int stepperPins[] = {9, 10, 11, 12};
int stepperSteps = 200;
float stepperStepSize = 9.0/5;

// Servo motor
float currentServoVal = 90;

// Indicator LED
const int LED = 13;
// =======================================================

// ============ Initialize electronic interfaces =========
// Stepper
Stepper motor(stepperSteps, stepperPins[0], stepperPins[1], stepperPins[2], stepperPins[3]);

// Servo
Servo altitude;

// Gyro
LSM9DS1 imu;

// ============ Runtime variables ========================
// Orientation data
float currentYBodyPosition = 90;
float currentZBodyPosition = 0;
float currentZMotorPosition = 0;

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
  motor.setSpeed(20); // Stepper
  altitude.attach(3); // Servo

  // Calibrate the gyro
  zeroGyro();

  // Setup motor update interval timer
  MsTimer2::set(motorUpdateRate, updateMotor);
  MsTimer2::start();

  // Blink LED to indicate that initilization has completed!
  blinkLED(20, 10);
}

void loop() {

  // Accept corrections from serial
  if (Serial.available() == 4) {
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
    /*
    currentZBodyPosition += correctionFactorXY;
    currentZMotorPosition += correctionFactorXY;
    if (Serial.read() == '1') negZ = 1;
    correctionFactorZ += (Serial.read() - '0') * 100;
    correctionFactorZ += (Serial.read() - '0') * 10;
    correctionFactorZ += (Serial.read() - '0');
    if (negZ) correctionFactorZ = -correctionFactorZ;
    altitude.write(currentYBodyPosition + correctionFactorZ);
    currentYBodyPosition += correctionFactorZ;
    */
  }

  // Update orientation data with measurements from gyro
  // Just using angular rates from gyro and time deltas to track position using a Reimann sum.
  int currentTime = millis();
  int deltaT = currentTime - timer;
  currentZBodyPosition += deltaT * readGyroZ() / 1000; // Gyro data is given in 1000ths of a degree, so we divide by 1000
  currentYBodyPosition += deltaT * readGyroY() / 1000;
  timer = currentTime;

  // Update motor positions
  if(update_flag) {
    // Debug output
    // Update servo position (Y/ALT)
    altitude.write(currentYBodyPosition);

    // Update stepper position (Z/AZM)
    float difference = currentZBodyPosition - currentZMotorPosition; // we want currentZMotorPosition + currentZBodyPosition = 0
    int steps = rotateStepperBy(-difference);
    currentZMotorPosition += -steps * stepperStepSize;

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
int rotateStepperBy(float deg) {
  int steps = deg/stepperStepSize;
  motor.step(steps);  
  return steps;
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

