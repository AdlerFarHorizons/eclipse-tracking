import processing.opengl.*;
import processing.serial.*;
float y = 0.0;
float x = 0.0;
float z = 0.0;
String val;
Serial myPort;
void setup()
  {
    size(800,600,P3D);
    smooth();
    String portName = Serial.list()[0];
    myPort = new Serial(this, portName, 9600);
  }
void draw()
  {
    if (myPort.available() > 0)
    {
        val = myPort.readStringUntil('\n');
    }
    String[] info = split(val, ',');
    translate(400,300,0);
    rotateX(x);
    rotateY(y);
    rotateZ(z);
    background(255);
    fill(255,228,225);
    box(200);
    if (info != null) {
      x = float(info[0]);
      y = float(info[1]);
      z = float(info[2]);
    }
  }