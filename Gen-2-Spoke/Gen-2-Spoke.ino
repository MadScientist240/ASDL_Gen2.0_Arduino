/***************************************************
  This Arduino Sketch is for the IAQ Spoke device

  - Uses BMP388 and SCD40 sensors over I2C
  - Communicates with Zigbee Hub device

****************************************************/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <SensirionI2cScd4x.h>

// Sensors
Adafruit_BMP3XX bmp388;
SensirionI2cScd4x scd40;

// Sensor Structures
typedef struct {
  float temperature;    // °C
  float pressure;       // kPa
  float humidity;       // %
  float altitude;       // Meters
  float co2;            // ppm
  float battery_level;  // %
}data_struct;

typedef struct {
  int sample_rate;      // seconds
  bool bmp_enabled;
  bool scd_enabled;
}config_struct;

data_struct data;
config_struct config = {5, true, true};

#define SCL_PIN 7
#define SDA_PIN 6
#define SEALEVELPRESSURE_HPA (1013.25)  // FIXME: Adjust properly

// SCD40 variables
static int16_t error;
uint64_t serialNumber = 0;
bool dataReady = false;
uint16_t co2Concentration;
float temperature;
float relativeHumidity;


void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);

  // BMP388 Init:
  if(config.bmp_enabled){
    bmp_init();
  } else {
    Serial.println("BMP388 is disabled in config");
  }

  // SCD40 Init:
  if(config.scd_enabled){
    scd_init();
  } else {
    Serial.println("SCD40 is diasbled in config");
  }

  
}

void loop() {
  delay(config.sample_rate * 1000);
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
    Serial.println("BMP388 Reading Failed");
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

bool scd_init(){
  scd40.begin(Wire, SCD41_I2C_ADDR_62);

  error = scd40.wakeUp();
  if(error != 0){
    Serial.println("Error trying to wake up SCD40");
    return false;
  }

  error = scd40.stopPeriodicMeasurement();
  if(error != 0){
    Serial.println("Error trying to stop SCD40 periodic measurements");
    return false;
  }

  error = scd40.reinit();
  if(error != 0){
    Serial.println("Error trying to reinit SCD40");
    return false;
  }
  
  error = scd40.getSerialNumber(serialNumber);
  if(error != 0){
    Serial.println("Error trying to get SCD40 serial number");
    return false;
  }

  error = scd40.startPeriodicMeasurement();
  if(error != 0){
    Serial.println("Error trying to start SCD40 periodic measurements");
    return false;
  }
  return true;
}

bool scd_ready(){
  error = scd40.getDataReadyStatus(dataReady);
  if(error != 0){
    Serial.print("Error trying to get SCD40 data ready status");
    return false;
  }

  error = scd40.readMeasurement(co2Concentration, temperature, relativeHumidity);
  if(error != 0){
    Serial.print("Error trying to read SCD40 measurement");
    return false;
  }

  data.co2 = co2Concentration;
  data.humidity = relativeHumidity;

  Serial.print("CO2 concentration [ppm]: ");
  Serial.print(data.co2);
  Serial.println();
  Serial.print("Temperature [°C]: ");
  Serial.print(data.temperature);
  Serial.println();
  Serial.print("Relative Humidity [RH]: ");
  Serial.print(data.humidity);
  Serial.println();

  return true;
}
