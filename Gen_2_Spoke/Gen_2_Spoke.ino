/********************************************************
  This Arduino Sketch is for the GenII IAQ Spoke device

  - Uses BMP388, SCD40, and PM2.5 sensors over I2C
  - Communicates with GenII Hub device (Zb Coord.)

*********************************************************/

#include "Zigbee.h"
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <SensirionI2cScd4x.h>
#include "Adafruit_PM25AQI.h"

// Zigbee endpoints
ZigbeeTempSensor zbTemperature = ZigbeeTempSensor(10);
ZigbeeAnalog zbHumidity = ZigbeeAnalog(11);
ZigbeePM25Sensor zbPM25 = ZigbeePM25Sensor(12);
ZigbeePressureSensor zbPressure = ZigbeePressureSensor(13);
ZigbeeAnalog zbCO2 = ZigbeeAnalog(14);
ZigbeeAnalog zbAltitude = ZigbeeAnalog(15);


// Configuration
bool PM25_ACTIVE = false;
bool BMP_ACTIVE = false;
bool SCD_ACTIVE = false;
int SAMPLE_RATE = 10; // seconds

// Sensors
Adafruit_BMP3XX bmp388 = Adafruit_BMP3XX();
SensirionI2cScd4x scd40;
Adafruit_PM25AQI pm25 = Adafruit_PM25AQI();
#define SEALEVELPRESSURE_HPA (1013.25)  // FIXME: Adjust properly
PM25_AQI_Data data;

// SCD40
static int16_t error;
uint64_t serialNumber = 0;
bool dataReady = false;
uint16_t co2Concentration;
float temperature;
float relativeHumidity;

// IO
#define SCL_PIN 7
#define SDA_PIN 6

void setup() {
  Serial.begin(115200);

  Zigbee.addEndpoint(&zbTemperature);
  Zigbee.addEndpoint(&zbHumidity);
  Zigbee.addEndpoint(&zbPM25);
  Zigbee.addEndpoint(&zbPressure);
  Zigbee.addEndpoint(&zbCO2);
  Zigbee.addEndpoint(&zbAltitude);

  // Cluster Initialization
  zbHumidity.addAnalogInput();
  zbHumidity.setAnalogInputApplication(0);
  zbHumidity.setAnalogInputDescription("Humidity");

  zbCO2.addAnalogInput();
  zbCO2.setAnalogInputApplication(0);
  zbCO2.setAnalogInputDescription("CO2");

  zbAltitude.addAnalogInput();
  zbAltitude.setAnalogInputApplication(0);
  zbAltitude.setAnalogInputDescription("Altitude");
  
  Zigbee.begin(ZIGBEE_END_DEVICE, true);

}

void loop() {
  if(Zigbee.connected()){
    globalSensorReport();

    delay(SAMPLE_RATE * 1000);
  } else {
    Serial.println("Waiting for Zigbee to connect...");
    delay(5000);
  }
}

/*
 *  Read, and then report to Zigbee
 */
void globalSensorReport(){
  //readBMP();
  //readSCD();
  //readPM25();

  // DUMMY DATA:
  zbTemperature.setTemperature(8330);
  zbHumidity.setAnalogInput(68.221);
  zbCO2.setAnalogInput(242.11);
  zbPressure.setPressure(7640);
  zbAltitude.setAnalogInput(2000.0);
  zbPM25.setPM25(650);

  zbTemperature.report();
  zbHumidity.reportAnalogInput();
  zbCO2.reportAnalogInput();
  zbPressure.report();
  zbAltitude.reportAnalogInput();
  zbPM25.report();
  
  Serial.println("Data sent!");
}

void globalSensorInit(){
// SCD40 Initialization
  if(SCD_ACTIVE){
    scd40.begin(Wire, SCD40_I2C_ADDR_62);

    if(scd40.wakeUp() != 0){
      Serial.println("SCD40: Error trying to wake up");
    }

    if(scd40.stopPeriodicMeasurement() != 0){
      Serial.println("SCD40: Error trying to stop periodic measurements");
    }

    if(scd40.reinit() != 0){
      Serial.println("SCD40: Error trying to reinit");
    }
    
    if(scd40.getSerialNumber(serialNumber) != 0){
      Serial.println("SCD40: Error trying to get serial number");
    }

    if(scd40.startPeriodicMeasurement() != 0){
      Serial.println("SCD40: Error trying to start periodic measurements");
    }
  } else {
    Serial.println("SCD40 Disabled in configuration");
  }

// BMP388 Initialization
  if(BMP_ACTIVE){
    if(!bmp388.begin_I2C()){
      Serial.println("BMP388: Could not find BMP388 sensor");
    } else {
      // Set up oversampling and filter initialization
      bmp388.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
      bmp388.setPressureOversampling(BMP3_OVERSAMPLING_4X);
      bmp388.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
      bmp388.setOutputDataRate(BMP3_ODR_50_HZ);
    }
  } else {
    Serial.println("BMP388 Disabled in configuration");
  }

// PM2.5 Initialization
  if(PM25_ACTIVE){
    if(pm25.begin_I2C()){
      while (1) delay(10);
    } else {
      Serial.println("Could not find PM2.5 sensor");
    }
  } else {
    Serial.println("PM25 Disabled in configuration");
  }
}

// Low level
bool readSCD(){
  if(scd40.getDataReadyStatus(dataReady) != 0){
    Serial.print("SCD40: Error trying to get data ready status");
    return false;
  }

  if(scd40.readMeasurement(co2Concentration, temperature, relativeHumidity) != 0){
    Serial.print("SCD40: Error trying to read measurement");
    return false;
  }

  zbTemperature.setTemperature(temperature);
  zbHumidity.setAnalogInput(relativeHumidity);
  zbCO2.setAnalogInput(co2Concentration);
  
  return true;
}

bool readBMP(){
  if(!bmp388.performReading()){
    Serial.println("BMP388: Reading Failed");
    return false;
  } else {

    zbPressure.setPressure(bmp388.pressure);
    zbAltitude.setAnalogInput(bmp388.readAltitude(SEALEVELPRESSURE_HPA));

    // Get temp from SCD40 BY DEFAULT, else rely on BMP388
    if(!SCD_ACTIVE){ 
      zbTemperature.setTemperature(bmp388.temperature);
    }
  }
  return true;
}

bool readPM25(){
  if(!pm25.read(&data)){
    Serial.print("Could not read from AQI");
    delay(1000);
    return false;
  }

  // Standard = Indoor Reporting
  zbPM25.setPM25(data.pm25_standard);
  return true;
}
