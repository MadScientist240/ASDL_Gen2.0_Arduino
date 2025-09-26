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

// Sensor Data Structure
typedef struct {
  float temperature;    // °C
  float pressure;       // kPa
  float humidity;       // %
  float altitude;       // Meters
  float co2;            // ppm
  float battery_level;  // %
}data_struct;

data_struct data;

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);

  // BMP388 Init:
  bmp_init();
}

void loop() {
  bmp_read();
}

bool bmp_init(){
  if(!bmp388.begin_I2C()){
    Serial.println("Could not find BMP388 sensor");
    return false;
  } else {
    // Set up oversampling and filter initialization
    bmp388.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp388.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp388.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp388.setOutputDataRate(BMP3_ODR_50_HZ);
  }
  return true;
}

bool bmp_read(){
  if(!bmp388.performReading()){
    Serial.print("BMP388 Reading Failed");
    return false;
  } else {
    Serial.println("========== BMP388 ==========");

    data.temperature = bmp388.temperature;
    data.pressure = bmp388.pressure / 100.0;
    data.altitude = bmp388.readAltitude(SEALEVELPRESSURE_HPA);

    Serial.print("Temperature = ");
    Serial.print(data.temperature);
    Serial.println(" *C");

    Serial.print("Pressure = ");
    Serial.print(data.pressure);
    Serial.println(" hPa");

    Serial.print("Approx. Altitude = ");
    Serial.print(data.altitude);
    Serial.println(" m");

    Serial.println();
  }
  return true;
}
