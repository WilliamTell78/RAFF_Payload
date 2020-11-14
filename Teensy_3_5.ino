

// Microcontroller Teensy 3.5
// SD - Version: Latest 
#include <SD.h>
#include <Wire.h>
#include "I2C.h"

#include "i2c_BMP280.h"  // Library has already been written, this file is just pulling those functions into our needed use
BMP280 bmp280;

#define 19 SCL     // Defining which pins are which
#define 18 SDA
#define 17 SDA_1
#define 16 SCL_1

int incomingByte = 0; // starts the value of the data we get at 0 

import processing.serial.*;
import java.awt.datatransfer.*;
import java.awt.Toolkit;
import processing.opengl.*;
import saito.objloader.*;
import g4p_controls.*;

float roll  = 0.0F;
float pitch = 0.0F;
float yaw   = 0.0F;
float temp  = 0.0F;
float alt   = 0.0F;

OBJModel model;

// Serial port state.
Serial       port;
final String serialConfigFile = "serialconfig.txt";
boolean      printSerial = false;

// UI controls.
GPanel    configPanel;
GDropList serialList;
GLabel    serialLabel;
GLabel    calLabel;
GCheckbox printSerialCheckbox;

void setup() {
    
    pinmode(19, OUTPUT);  // designates pins 19 and 18 for recieving data
    pinmode(18, OUTPUT);
    pinmode(17, INPUT);   // designates pins 17 and 16 for sending data out
    pinmode(16, INPUT);
    
     Serial.begin(115200);

    Serial.print("Probe BMP280: ");
    if (bmp280.initialize()) Serial.println("Sensor found");
    else
    {
        Serial.println("Sensor missing");
        while (1) {}
    }

    // onetime-measure:
    //bmp280.setEnabled(0);
    bmp280.triggerMeasurement();
    
    size(640, 480, OPENGL);
  frameRate(30);
  model = new OBJModel(this);
  model.load("bunny.obj");
  model.scale(20);
  
  // Serial port setup.
  // Grab list of serial ports and choose one that was persisted earlier or default to the first port.
  int selectedPort = 0;
  String[] availablePorts = Serial.list();
  if (availablePorts == null) {
    println("ERROR: No serial ports available!");
    exit();
  }
  String[] serialConfig = loadStrings(serialConfigFile);
  if (serialConfig != null && serialConfig.length > 0) {
    String savedPort = serialConfig[0];
    // Check if saved port is in available ports.
    for (int i = 0; i < availablePorts.length; ++i) {
      if (availablePorts[i].equals(savedPort)) {
        selectedPort = i;
      } 
    }
  }
  // Build serial config UI.
  configPanel = new GPanel(this, 10, 10, width-20, 90, "Configuration (click to hide/show)");
  serialLabel = new GLabel(this,  0, 20, 80, 25, "Serial port:");
  configPanel.addControl(serialLabel);
  serialList = new GDropList(this, 90, 20, 200, 200, 6);
  serialList.setItems(availablePorts, selectedPort);
  configPanel.addControl(serialList);
  calLabel = new GLabel(this, 300, 20, 350, 25, "Calibration: Sys=? Gyro=? Accel=? Mag=?");
  configPanel.addControl(calLabel); 
  printSerialCheckbox = new GCheckbox(this, 5, 50, 200, 20, "Print serial data");
  printSerialCheckbox.setSelected(printSerial);
  configPanel.addControl(printSerialCheckbox);
  // Set serial port.
  setSerialPort(serialList.getSelectedText());
    
}

void draw()
{
  background(0,0,0);

  // Set a new co-ordinate space
  pushMatrix();

  // Simple 3 point lighting for dramatic effect.
  // Slightly red light in upper right, slightly blue light in upper left, and white light from behind.
  pointLight(255, 200, 200,  400, 400,  500);
  pointLight(200, 200, 255, -400, 400,  500);
  pointLight(255, 255, 255,    0,   0, -500);
  
  // Move bunny from 0,0 in upper left corner to roughly center of screen.
  translate(300, 380, 0);
  
  // Rotate shapes around the X/Y/Z axis (values in radians, 0..Pi*2)
  //rotateZ(radians(roll));
  //rotateX(radians(pitch)); // extrinsic rotation
  //rotateY(radians(yaw));
  float c1 = cos(radians(roll));
  float s1 = sin(radians(roll));
  float c2 = cos(radians(pitch)); // intrinsic rotation
  float s2 = sin(radians(pitch));
  float c3 = cos(radians(yaw));
  float s3 = sin(radians(yaw));
  applyMatrix( c2*c3, s1*s3+c1*c3*s2, c3*s1*s2-c1*s3, 0,
               -s2, c1*c2, c2*s1, 0,
               c2*s3, c1*s2*s3-c3*s1, c1*c3+s1*s2*s3, 0,
               0, 0, 0, 1);

  pushMatrix();
  noStroke();
  model.draw();
  popMatrix();
  popMatrix();
  //print("draw");
}

void serialEvent(Serial p) 
{
  String incoming = p.readString();
  if (printSerial) {
    println(incoming);
  }
  
  if ((incoming.length() > 8))
  {
    String[] list = split(incoming, " ");
    if ( (list.length > 0) && (list[0].equals("Orientation:")) ) 
    {
      roll  = float(list[3]); // Roll = Z
      pitch = float(list[2]); // Pitch = Y 
      yaw   = float(list[1]); // Yaw/Heading = X
    }
    if ( (list.length > 0) && (list[0].equals("Alt:")) ) 
    {
      alt  = float(list[1]);
    }
    if ( (list.length > 0) && (list[0].equals("Temp:")) ) 
    {
      temp  = float(list[1]);
    }
    if ( (list.length > 0) && (list[0].equals("Calibration:")) )
    {
      int sysCal   = int(list[1]);
      int gyroCal  = int(list[2]);
      int accelCal = int(list[3]);
      int magCal   = int(trim(list[4]));
      calLabel.setText("Calibration: Sys=" + sysCal + " Gyro=" + gyroCal + " Accel=" + accelCal + " Mag=" + magCal);
    }
  }
}

// Set serial port to desired value.
void setSerialPort(String portName) {
  // Close the port if it's currently open.
  if (port != null) {
    port.stop();
  }
  try {
    // Open port.
    port = new Serial(this, portName, 115200);
    port.bufferUntil('\n');
    // Persist port in configuration.
    saveStrings(serialConfigFile, new String[] { portName });
  }
  catch (RuntimeException ex) {
    // Swallow error if port can't be opened, keep port closed.
    port = null; 
  }
}

// UI event handlers

void handlePanelEvents(GPanel panel, GEvent event) {
  // Panel events, do nothing.
}

void handleDropListEvents(GDropList list, GEvent event) { 
  // Drop list events, check if new serial port is selected.
  if (list == serialList) {
    setSerialPort(serialList.getSelectedText()); 
  }
}

void handleToggleControlEvents(GToggleControl checkbox, GEvent event) { 
  // Checkbox toggle events, check if print events is toggled.
  if (checkbox == printSerialCheckbox) {
    printSerial = printSerialCheckbox.isSelected(); 
  }
}

void loop() {
    
    bmp280.awaitMeasurement();

    float temperature;
    bmp280.getTemperature(temperature);

    float pascal;
    bmp280.getPressure(pascal);

    static float meters, metersold;
    bmp280.getAltitude(meters);
    metersold = (metersold * 10 + meters)/11;

    bmp280.triggerMeasurement();

    Serial.print(" HeightPT1: ");
    Serial.print(metersold);
    Serial.print(" m; Height: ");
    Serial.print(meters);
    Serial.print(" Pressure: ");
    Serial.print(pascal);
    Serial.print(" Pa; T: ");
    Serial.print(temperature);
    Serial.println(" C");
    
    
    // send data only when we recieve it:
    //if (Serial.available() > 0) {
      // read the incoming byte:
      //incomingByte = Serial.read();
      
      // print what we just got
      //Serial.print(incomingByte, DEC);
    }
}
