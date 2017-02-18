/* Main System Program
 * Reads data from Sensor Array and 9DoF Razor over Serial
 * Last touched on 1/21/17
 */

#include <SoftwareSerial.h>

SoftwareSerial mySerial(10, 11); // Init RX and TX Pins

void setup() {
  Serial.begin(115200);
  while(! Serial); // wait to connect
}

void loop() {
  if (mySerial.available()) {
    Serial.println(mySerial.read());
  }  
}
