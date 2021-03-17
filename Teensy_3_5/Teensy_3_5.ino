
// SD - Version: Latest 
#include <SD.h>
#include <Wire.h>
#include "I2C.h"

#include "i2c_BMP280.h"  // Library has already been written, this file is just pulling those functions into our needed use
BMP280 bmp280;

/*
  Microcontroller 
*/
 
 #define 19 SCL     // Defining which pins are which
 #define 18 SDA
 #define 14 SDA
 #define 17 SDA_1
 #define 16 SCL_1
 
 int incomingByte = 0; // starts the value of the data we get at 0
 
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
