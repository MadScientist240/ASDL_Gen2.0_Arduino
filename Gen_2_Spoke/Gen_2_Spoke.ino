/***************************************************
  This Arduino Sketch is for the IAQ Spoke device

  - Uses BMP388 and SCD40 sensors over I2C
  - Communicates with Zigbee Hub device

****************************************************/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <SensirionI2cScd4x.h>
#include "Zigbee.h"
#include "esp_zigbee_core.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_zigbee_core.h"

// Sensors
Adafruit_BMP3XX bmp388;
SensirionI2cScd4x scd40;

// Sensor Structures
struct data_struct{
  float temperature;    // °C
  float pressure;       // kPa
  float humidity;       // %
  float altitude;       // Meters
  float co2;            // ppm
  float battery_level;  // %
}__attribute__((packed));

struct config_struct{
  int sample_rate;      // seconds
  bool bmp_enabled;
  bool scd_enabled;
}__attribute__((packed));

data_struct data;
config_struct config = {10, true, true}; // Adjust to desired interval

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

// Zigbee
#define ASDL_CUSTOM_CLUSTER_ID 0xFC00
#define SENSOR_DATA_ATTR_ID 0x0001


void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);

  // Zigbee Init:
  nvs_flash_init();
  Serial.println("ZIGBEE: Starting Zigbee task");
  xTaskCreate(zigbee_task, "zigbee", 4096, NULL, 5, NULL);

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
    Serial.println("SCD40 is disabled in config");
  }
}

void loop() {
  static bool was_connected = false;
    bool is_connected = esp_zb_bdb_dev_joined();
    
    if (is_connected && !was_connected) {
        Serial.println("✓ JOINED NETWORK!");
        was_connected = true;
    } else if (!is_connected && was_connected) {
        Serial.println("✗ DISCONNECTED");
        was_connected = false;
    } 
    
    if (is_connected) {
      // Poll sensors here:
      delay(config.sample_rate * 1000);
      scd_read();
      bmp_read();
      print_data();
      send_data();
    }
    delay(1000);
}

bool bmp_init(){
  if(!bmp388.begin_I2C()){
    Serial.println("BMP388: Could not find BMP388 sensor");
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
  if(config.bmp_enabled){
    if(!bmp388.performReading()){
      Serial.println("BMP388: Reading Failed");
      return false;
    } else {

      data.pressure = bmp388.pressure / 100.0;
      data.altitude = bmp388.readAltitude(SEALEVELPRESSURE_HPA);

      // Get temp from SCD40 BY DEFAULT, else rely on BMP388
      if(error !=  0 || !config.scd_enabled){ 
        data.temperature = bmp388.temperature;
      }
      /*
      Serial.println("=========== BMP388 ===========");
      Serial.print("Temperature = ");
      Serial.print(bmp388.temperature);
      Serial.println(" °C");

      Serial.print("Pressure = ");
      Serial.print(bmp388.pressure);
      Serial.println(" hPa");

      Serial.print("Approx. Altitude = ");
      Serial.print(data.altitude);
      Serial.println(" m");

      Serial.println();
      */
    }
    return true;
  } else {
    Serial.println("BMP388: Disabled in config");
    return false;
  }
}

bool scd_init(){
  scd40.begin(Wire, SCD40_I2C_ADDR_62);

  error = scd40.wakeUp();
  if(error != 0){
    Serial.println("SCD40: Error trying to wake up");
    return false;
  }

  error = scd40.stopPeriodicMeasurement();
  if(error != 0){
    Serial.println("SCD40: Error trying to stop periodic measurements");
    return false;
  }

  error = scd40.reinit();
  if(error != 0){
    Serial.println("SCD40: Error trying to reinit");
    return false;
  }
  
  error = scd40.getSerialNumber(serialNumber);
  if(error != 0){
    Serial.println("SCD40: Error trying to get serial number");
    return false;
  }

  error = scd40.startPeriodicMeasurement();
  if(error != 0){
    Serial.println("SCD40: Error trying to start periodic measurements");
    return false;
  }
  return true;
}

bool scd_read(){
  if(config.scd_enabled){
    error = scd40.getDataReadyStatus(dataReady);
    if(error != 0){
      Serial.print("SCD40: Error trying to get data ready status");
      return false;
    }

    error = scd40.readMeasurement(co2Concentration, temperature, relativeHumidity);
    if(error != 0){
      Serial.print("SCD40: Error trying to read measurement");
      return false;
    }

    data.co2 = co2Concentration;
    data.humidity = relativeHumidity;
    data.temperature = temperature;
      /*
    Serial.println("============ SCD40 ============");
    Serial.print("CO2 concentration: ");
    Serial.print(co2Concentration);
    Serial.println(" ppm");

    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" °C");

    Serial.print("Relative Humidity: ");
    Serial.print(relativeHumidity);
    Serial.println(" %");

    Serial.println();
      */
    return true;
  } else {
    Serial.println("SCD40: Disabled in config");
    return false;
  }
}

void print_data(){
  Serial.println("============ Packaged Data ============");
  Serial.print("Temperature: ");
  Serial.print(data.temperature);
  Serial.print(" °C");
  Serial.println();

  Serial.print("Pressure: ");
  Serial.print(data.pressure);
  Serial.print(" kPa");
  Serial.println();

  Serial.print("Humidity: ");
  Serial.print(data.humidity);
  Serial.print(" %");
  Serial.println();

  Serial.print("Altitude: ");
  Serial.print(data.altitude);
  Serial.print(" m");
  Serial.println();

  Serial.print("CO2: ");
  Serial.print(data.co2);
  Serial.print(" ppm");
  Serial.println();

  Serial.print("Battery Level: ");
  Serial.print(data.battery_level);
  Serial.print(" %");
  Serial.println();
  Serial.println();
}

// ================= ZIGBEE FUNCTIONS =================
void zigbee_task(void *arg){
  // Init Zigbee
    esp_zb_cfg_t zb_cfg;
    zb_cfg.esp_zb_role = ESP_ZB_DEVICE_TYPE_ED;
    zb_cfg.install_code_policy = false;
    zb_cfg.nwk_cfg.zczr_cfg.max_children = 0;
    esp_zb_init(&zb_cfg);
    
    // Create custom cluster
    esp_zb_attribute_list_t *cluster = esp_zb_zcl_attr_list_create(ASDL_CUSTOM_CLUSTER_ID);
    esp_zb_cluster_add_attr(cluster, ASDL_CUSTOM_CLUSTER_ID, SENSOR_DATA_ATTR_ID, ESP_ZB_ZCL_ATTR_TYPE_OCTET_STRING,
                            ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY, &data);
    
    // Add cluster to list
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_custom_cluster(cluster_list, cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    
    // Create endpoint
    esp_zb_ep_list_t *endpoint_list = esp_zb_ep_list_create();
    esp_zb_endpoint_config_t ep_cfg = {
        .endpoint = 10,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_CUSTOM_ATTR_DEVICE_ID
    };
    esp_zb_ep_list_add_ep(endpoint_list, cluster_list, ep_cfg);
    
    // Register and start
    esp_zb_device_register(endpoint_list);
    esp_zb_start(false);
    esp_zb_main_loop_iteration();
}

void send_data() {
    esp_zb_zcl_report_attr_cmd_t cmd;
    cmd.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    cmd.zcl_basic_cmd.dst_addr_u.addr_short = 0x0000;  // Coordinator
    cmd.zcl_basic_cmd.dst_endpoint = 1;
    cmd.zcl_basic_cmd.src_endpoint = 10;
    cmd.clusterID = ASDL_CUSTOM_CLUSTER_ID;      // Your custom cluster ID
    cmd.attributeID = SENSOR_DATA_ATTR_ID;    // Your custom attribute ID
    //cmd.cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE;
    
    esp_zb_zcl_report_attr_cmd_req(&cmd);
}