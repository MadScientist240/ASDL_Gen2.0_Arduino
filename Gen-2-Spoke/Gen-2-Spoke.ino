/***************************************************
  This Arduino Sketch is for the IAQ Spoke device

  - Uses BMP388 and SCD40 sensors over I2C
  - Communicates with Zigbee Hub device

****************************************************/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"

#define SCL_PIN 6
#define SDA_PIN 7
#define SEALEVELPRESSURE_HPA (1013.25)  // FIXME: Adjust properly

// Sensors
Adafruit_BMP3XX bmp388;

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:
}
