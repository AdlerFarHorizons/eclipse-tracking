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
LSM9DS1 imu;
Servo xy;
float entries[10];
int i = 0;
int j = 0;
bool int_flag = false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  setupIMU();

  for (int i = 0; i < 10; i++) {
    entries[i] = readGyro();
  } // fill up the rolling average
  
  xy.attach(3); // pro mini pwm pin
  xy.write(0); // Start at a well-defined value

  MsTimer2::set(1000, a);
  MsTimer2::start();
  
}

void loop() {
  if (int_flag) {
    entries[j % 10] = readGyro();
    Serial.println(getAverage(), 2);
    int_flag = false;
  }
  //delay(100);
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
    Serial.println(" 3.3V    | 1.9-3.6 V");
    Serial.println(" GND     | GND");
    Serial.println(" A4      | SDA");
    Serial.println(" A5      | SCL");
    while (1) {
      blinkLED();
    }
  }
  Serial.println(" [ DONE ] ");
  
}

float readGyro() {
  if (imu.gyroAvailable()) {
    imu.readGyro();
  }
  return imu.calcGyro(imu.gz);
}
// Arguments: integer 0 - 180 degrees
// Postcondition: servo rotates to integer argument 
void rotate(int deg) {
  xy.write(deg);
}

void blinkLED() {
  digitalWrite(13, HIGH);
  delay(500);
  digitalWrite(13, LOW);
  delay(500);
}

float getAverage() {
  float sum = 0;
  for (int i = 0; i < 10; i++) {
    sum += entries[i];
  }
  return sum / 10.;
}

