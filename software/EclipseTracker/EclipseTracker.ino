/************************************
 * Tracks a light source.           *
 * Version: 1.0.0                   *
 * Written for the Eclipse Project. *
 * Uses the SparkFunLSM9DS1 library.*
 ***********************************/

#include <LSM9DS1_Registers.h>
#include <LSM9DS1_Types.h>
#include <SparkFunLSM9DS1.h>
#include <Servo.h>
#define NUMSENSORS 5
#define LSM9DS1_M  0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW
#define DECLINATION 3.5 // Declination (degrees) in Chicago, IL.

Servo zServo;
Servo xyServo;

LSM9DS1 imu;
// Sensor Array ***
int sensorArray[] = {0, 0, 0, 0, 0};
// END Sensor Array ***

const int infoPin = 13;

void setup() {
  pinMode(infoPin, OUTPUT);
  zServo.attach(3);
  xyServo.attach(5);
  float calibrationAzimuth, calibrationAltitude;
  Serial.begin(9600);
  Serial.println("*** Starting Calibration ***");
  Serial.print("Azimuth of sun in degrees?");
  while (!Serial.available());
  calibrationAzimuth = Serial.parseFloat();
  Serial.println(" [ OK ]");
  Serial.print("Altitude of sun in degrees?");
  while (!Serial.available());
  calibrationAltitude = Serial.parseFloat();
  Serial.println(" [ OK ]");
  calibrate(calibrationAzimuth, calibrationAltitude);
  Serial.println("\nCalibrated. [ OK ]");
  // Calibration done.
  // i2c stuff
  // Before initializing the IMU, there are a few settings
  // we may need to adjust. Use the settings struct to set
  // the device's communication mode and addresses:
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
  Serial.print("Connecting to LSM9DS1 sensor...");
  if (!imu.begin())
  {
    Serial.println(" [ FAILED ]");
    while (1) {
      digitalWrite(infoPin, HIGH);
      delay(500);
      digitalWrite(infoPin, LOW);
      delay(500);
    }
  }
  Serial.println(" [ Done ]");
  digitalWrite(infoPin, HIGH);
  delay(1000);
  digitalWrite(infoPin, LOW);
}

void loop() {
  readArray();
  printGyro();  // Print "G: gx, gy, gz"
  printAccel(); // Print "A: ax, ay, az"
  printMag();   // Print "M: mx, my, mz"
  
  // Print the heading and orientation for fun!
  // Call print attitude. The LSM9DS1's magnetometer x and y
  // axes are opposite to the accelerometer, so my and mx are
  // substituted for each other.
  printAttitude(imu.ax, imu.ay, imu.az, -imu.my, -imu.mx, imu.mz);
  Serial.println();
  
  delay(250);
}

void readArray() {
  for (int i = 0; i < NUMSENSORS; i++) {
    sensorArray[i] = analogRead(i);
  }
}

void calibrate(float azimuth, float altitude) {
  // Move camera to point at azimuth and altitude.
  // First point north.
  // Level 180 degree servo at 90 degrees.
  zServo.write(90);
  zServo.write(90 + altitude); // Change depending on orientation of the servo.
  // Move 180 degree servo up by the azimuth.
  // Rotate the continuous 360 degree servo by altitude.
  xyServo.write(azimuth);
}

void printGyro()
{
  // To read from the gyroscope, you must first call the
  // readGyro() function. When this exits, it'll update the
  // gx, gy, and gz variables with the most current data.
  imu.readGyro();
  
  // Now we can use the gx, gy, and gz variables as we please.
  // Either print them as raw ADC values, or calculated in DPS.
  Serial.print("G: ");
  // If you want to print calculated values, you can use the
  // calcGyro helper function to convert a raw ADC value to
  // DPS. Give the function the value that you want to convert.
  Serial.print(imu.calcGyro(imu.gx), 2);
  Serial.print(", ");
  Serial.print(imu.calcGyro(imu.gy), 2);
  Serial.print(", ");
  Serial.print(imu.calcGyro(imu.gz), 2);
  Serial.println(" deg/s");

}

void printAccel()
{
  // To read from the accelerometer, you must first call the
  // readAccel() function. When this exits, it'll update the
  // ax, ay, and az variables with the most current data.
  imu.readAccel();
  
  // Now we can use the ax, ay, and az variables as we please.
  // Either print them as raw ADC values, or calculated in g's.
  Serial.print("A: ");
  // If you want to print calculated values, you can use the
  // calcAccel helper function to convert a raw ADC value to
  // g's. Give the function the value that you want to convert.
  Serial.print(imu.calcAccel(imu.ax), 2);
  Serial.print(", ");
  Serial.print(imu.calcAccel(imu.ay), 2);
  Serial.print(", ");
  Serial.print(imu.calcAccel(imu.az), 2);
  Serial.println(" g");
}

void printMag()
{
  // To read from the magnetometer, you must first call the
  // readMag() function. When this exits, it'll update the
  // mx, my, and mz variables with the most current data.
  imu.readMag();
  
  // Now we can use the mx, my, and mz variables as we please.
  // Either print them as raw ADC values, or calculated in Gauss.
  Serial.print("M: ");
  // If you want to print calculated values, you can use the
  // calcMag helper function to convert a raw ADC value to
  // Gauss. Give the function the value that you want to convert.
  Serial.print(imu.calcMag(imu.mx), 2);
  Serial.print(", ");
  Serial.print(imu.calcMag(imu.my), 2);
  Serial.print(", ");
  Serial.print(imu.calcMag(imu.mz), 2);
  Serial.println(" gauss");
}

// Calculate pitch, roll, and heading.
// Pitch/roll calculations take from this app note:
// http://cache.freescale.com/files/sensors/doc/app_note/AN3461.pdf?fpsp=1
// Heading calculations taken from this app note:
// http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/Magnetic__Literature_Application_notes-documents/AN203_Compass_Heading_Using_Magnetometers.pdf
void printAttitude(float ax, float ay, float az, float mx, float my, float mz)
{
  float roll = atan2(ay, az);
  float pitch = atan2(-ax, sqrt(ay * ay + az * az));
  
  float heading;
  if (my == 0)
    heading = (mx < 0) ? 180.0 : 0;
  else
    heading = atan2(mx, my);
    
  heading -= DECLINATION * PI / 180;
  
  if (heading > PI) heading -= (2 * PI);
  else if (heading < -PI) heading += (2 * PI);
  else if (heading < 0) heading += 2 * PI;
  
  // Convert everything from radians to degrees:
  /*heading *= 180.0 / PI;
  pitch *= 180.0 / PI;
  roll  *= 180.0 / PI;*/
  
  Serial.print("Pitch, Roll in radians: ");
  Serial.print(pitch, 2);
  Serial.print(", ");
  Serial.println(roll, 2);
  Serial.print("Heading: "); Serial.println(heading, 2);
}
