/***************************************************
  This Arduino Sketch is for the IAQ Spoke device

  - Uses BMP388 and SCD40 sensors over I2C
  - Communicates with Zigbee Hub device

****************************************************/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"

#define SCL_PIN 7
#define SDA_PIN 6
#define SEALEVELPRESSURE_HPA (1013.25)  // FIXME: Adjust properly

// Sensors
Adafruit_BMP3XX bmp388;

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);

  // BMP388 Init:
  if(!bmp388.begin_I2C()){
    Serial.println("Could not find BMP388 sensor");
  }

}

void loop() {

}
