/********************************************************
  This Arduino Sketch is for the GenII IAQ Spoke device

  - Uses BMP388 and SCD40 sensors over I2C
  - Communicates with GenII Hub device (Zb Coord.)

*********************************************************/

#include "Zigbee.h"
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <SensirionI2cScd4x.h>

// Zigbee endpoints
ZigbeeTempSensor zbTemperature = ZigbeeTempSensor(10);
ZigbeeAnalog zbHumidity = ZigbeeAnalog(11);
ZigbeePressureSensor zbPressure = ZigbeePressureSensor(13);
ZigbeeAnalog zbCO2 = ZigbeeAnalog(14);
ZigbeeAnalog zbAltitude = ZigbeeAnalog(15);
ZigbeeAnalog zbHubHeartbeat = ZigbeeAnalog(16);
bool report_success = true;


// Configuration
bool BMP_ACTIVE = true;
bool SCD_ACTIVE = true;
int SAMPLE_RATE = 300; // seconds

// Sensors
Adafruit_BMP3XX bmp388 = Adafruit_BMP3XX();
SensirionI2cScd4x scd40;
#define SEALEVELPRESSURE_HPA (1013.25)

// SCD40
static int16_t error;
uint64_t serialNumber = 0;
bool dataReady = false;
uint16_t co2Concentration;
float temperature;
float relativeHumidity;

// IO
/*
 * For ESP32C6 v 1.2, pins are default to 6 and 7
 * For ESP32C6 Mini, use pins 4 and 5
 */
#define SCL_PIN 5
#define SDA_PIN 4
#define LED_PIN 8
#define BUTTON_PIN 9

// LED
bool led_flashing = false;
uint8_t led_r = 0, led_g = 0, led_b = 0;
unsigned long last_flash = 0;
bool flash_state = false;

// Misc.
extern TaskHandle_t loopTaskHandle;
static unsigned long last_hub_ping = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);

  neopixelWrite(LED_PIN, 0, 0, 0);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  Zigbee.addEndpoint(&zbTemperature);
  Zigbee.addEndpoint(&zbHumidity);
  Zigbee.addEndpoint(&zbPressure);
  Zigbee.addEndpoint(&zbCO2);
  Zigbee.addEndpoint(&zbAltitude);
  Zigbee.addEndpoint(&zbHubHeartbeat);

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

  zbHubHeartbeat.addAnalogOutput();
  zbHubHeartbeat.setAnalogOutputDescription("Heartbeat");
  zbHubHeartbeat.onAnalogOutputChange(onPingReceived);

  // Enable Multiple Binding
  zbTemperature.allowMultipleBinding(true);
  zbHumidity.allowMultipleBinding(true);
  zbPressure.allowMultipleBinding(true);
  zbCO2.allowMultipleBinding(true);
  zbAltitude.allowMultipleBinding(true);
  zbHubHeartbeat.allowMultipleBinding(true);
  
  esp_zb_cfg_t role_config = esp_zb_cfg_t();

  Zigbee.begin(ZIGBEE_END_DEVICE, true);
  globalSensorInit();

  delay(2000);

  // Create sensor testing task
  xTaskCreate(localSensorTest, "Sensor Testing Task", 2048, NULL, 5, NULL);
}

void loop() {
  static int reconnect_attempts = 1;
  static unsigned long last_reconnect_attempt = 0;

  updateLED();

  if(Zigbee.connected() && (millis() - last_hub_ping < 90000)){
    setLED(0, 255, 0, false);
    reconnect_attempts = 0;
    globalSensorReport();
    delay(SAMPLE_RATE * 1000);
  } else {
    setLED(255, 0, 0, true);

    if(millis() - last_reconnect_attempt > 5000){
      Serial.printf("Disconnected. Reconnect attempt %d/10\n", reconnect_attempts++);
      last_reconnect_attempt = millis();

      //Restart device after 10 failed attempts
      if(reconnect_attempts > 10){
        setLED(255, 0, 0, false);
        delay(1000);
        Serial.println("Too many failed attempts, attempting factory reset...");
        Zigbee.factoryReset(true);
        ESP.restart();
      }
    }
  }
}

// Higher level
bool globalSensorReport(){
  readBMP();
  readSCD();

  report_success = zbTemperature.report();
  report_success = zbHumidity.reportAnalogInput();
  report_success = zbCO2.reportAnalogInput();
  report_success = zbPressure.report();
  report_success = zbAltitude.reportAnalogInput();
  
  if(report_success){
    Serial.println("Data sent!");
  } else {
    Serial.println("Failed silently!");
  }
  return report_success;
}

void globalSensorInit(){
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
}

void localSensorTest(void *pvParameters){

  for(;;){
    if (digitalRead(BUTTON_PIN) == LOW) {
      vTaskSuspend(loopTaskHandle);

      bool totalTesting = true;
      Serial.println("\n=== LOCAL SENSOR TEST ===");
      
      // Read sensors immediately
      if(readBMP()) {
        Serial.println("✓ BMP388 OK");
      } else {
        Serial.println("✗ BMP388 FAILED");
        totalTesting = false;
      }

      if(readSCD()) {
        Serial.println("✓ SCD40 OK");
      } else {
        Serial.println("✗ SCD40 FAILED");
        totalTesting = false;
      }

      if(totalTesting){
        setLED(255, 0, 255, false);
      } else {
        setLED(255, 255, 0, false);
      }
      delay(3000);  // Give time for test results to display

      Serial.println("=== TEST COMPLETE ===\n");

      vTaskResume(loopTaskHandle);
    }
    delay(100);
  }
}

// Low level
bool readSCD(){
  if(scd40.getDataReadyStatus(dataReady) != 0){
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
    return false;
  }
    zbPressure.setPressure(bmp388.pressure / 100.0);  // Pa → hPa
    zbAltitude.setAnalogInput(bmp388.readAltitude(SEALEVELPRESSURE_HPA));

    // Get temp from SCD40 BY DEFAULT, else rely on BMP388
    if(!SCD_ACTIVE){ 
      zbTemperature.setTemperature(bmp388.temperature);
    }
  return true;
}

void setLED(uint8_t r, uint8_t g, uint8_t b, bool flash) {
  led_r = r;
  led_g = g;
  led_b = b;
  led_flashing = flash;
  
  if (!flash) {
    neopixelWrite(LED_PIN, r, g, b);
  }
}

void updateLED() {
  if (led_flashing) {
    if (millis() - last_flash > 500) {
      flash_state = !flash_state;
      last_flash = millis();
      
      if (flash_state) {
        neopixelWrite(LED_PIN, led_r, led_g, led_b);
      } else {
        neopixelWrite(LED_PIN, 0, 0, 0);
      }
    }
  }
}

// Misc
void onPingReceived(float value){
  last_hub_ping = millis();
  Serial.println("Hub Ping received!");
}
