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

// Initial calibration value [x, y, z]
char dims[] = {'x', 'y', 'z'};
float calibrationOffset[] = {0, 0, 0};

// Indicator LED
const int LED = 13;

const byte GEAR_RATIO = 1; // servo gear : slip gear

// ============ Initialize electronic interfaces =========
LSM9DS1 imu;

const int numSteps = 200;
const int stepperRPM = 40;
const float stepSize = 1.8;

Stepper azimuth(numSteps, 9, 10, 11, 12);
// ============ Runtime variables ========================
// Orientation data
float positionChange;

// Interrupt flag
int timer = millis();
bool update_flag = false; // interrupt flag for executing motor update
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
  zeroGyro();

  // Find the sun to center on
  // findSun(); // comment this line out for basic testing

  // Notify the user the system is set up correctly and calibrated
  blinkLED(1000, 1);
  
  // Setup motor update interval timer
  MsTimer2::set(motorUpdateRate, updateMotor);
  MsTimer2::start();
}

void loop() {

  // Uncomment to accept corrections from serial
  /*
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
  */

  // Update orientation data with measurements from gyro
  // Just using angular rates from gyro and time deltas to track position using a Reimann sum.
  int currentTime = millis();
  
  int deltaT = currentTime - timer;
  positionChange += deltaT * readGyro('z') / 1000; // Gyro data is given in 1000ths of a degree, so we divide by 1000
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

  digitalWrite(LED, HIGH);
  while (!imu.begin());
  digitalWrite(LED, LOW);
}

/**
    Reads the IMU's gyro
    @return rate in degrees / ms
*/
float readGyro(char dim) {
  if (imu.gyroAvailable()) {
    imu.readGyro();
  }
  switch (dim) {
  case 'x':
    return imu.calcGyro(imu.gx) - calibrationOffset[0];
    break;
  case 'y':
    return imu.calcGyro(imu.gy) - calibrationOffset[1];
    break;
  case 'z':
    return imu.calcGyro(imu.gz) - calibrationOffset[2];
    break;
  default:
    return 0;
    break;
  } 
}

/**
    Finds the calibration constant of the
    gyro.
    @param number of calibration samples
    @returns degrees of gyro drift per second
*/
void zeroGyro() {
  float gyroSum;
  char active_dim; 
  digitalWrite(LED, HIGH);
  for (int j = 0; j < 3; j++) {
    active_dim = dims[j];
    gyroSum = 0;
    for (int i = 0; i < calibrationSamples; i++) {
      gyroSum += readGyro(active_dim);
     }
    // Serial.println(active_dim);
    // Serial.println(gyroSum / calibrationSamples);
    calibrationOffset[j] = gyroSum / calibrationSamples;
    blinkLED(300, 1);
  }
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

/**
    Rotates a stepper motor by 1.8 degrees until
    it recieves a byte over serial
    indicating that it found the sun
*/
/*
void findSun() {
  while (Serial.available() == 0) {
    azimuth.step(1);
    delay(500);
  }
  Serial.read();
}
*/
