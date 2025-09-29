/***************************************************
  This Arduino Sketch is for the Hub device

  - Uses BMP388 sensor over I2C to validate incoming data
  - Communicates with Particle Boron 404x

****************************************************/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include "Zigbee.h"
#include "esp_zigbee_core.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_zigbee_core.h"

// Sensor
Adafruit_BMP3XX bmp388;
#define SEALEVELPRESSURE_HPA (1013.25)  // FIXME: Adjust properly

// IAQ Data Structures
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

data_struct agg_data;
config_struct def_iaq_config = {5, true, true};

// IO
#define SCL_PIN 22
#define SDA_PIN 21
#define TX_PIN 2
#define RX_PIN 3
#define BOOT_BUTTON 9

// Zigbee
#define ASDL_CUSTOM_CLUSTER_ID 0xFC00
#define SENSOR_DATA_ATTR_ID 0x0001

// Pairing state
bool pairing_mode = false;
unsigned long pairing_start_time = 0;


void setup() {
  Serial.begin(115200);

  // Setup pairing button
  pinMode(BOOT_BUTTON, INPUT_PULLUP);

  Serial.println("Starting Zigbee Coordinator...");
  Serial.println("Press BOOT button to enable pairing mode");

  // Zigbee Init
  nvs_flash_init();
  // Register action handler
  esp_zb_core_action_handler_register(zb_action_handler);
  // Start Zigbee task
  xTaskCreate(zigbee_task, "zigbee", 4096, NULL, 5, NULL);
}

void loop() {
  handle_pairing();
  delay(10);
}

// ================== MISC FUNCTIONS ==================
// Handle pairing button and status
void handle_pairing(){  //FIXME
    static bool last_button_state = HIGH;
    static unsigned long last_debounce = 0;
    
    // Read button with debounce
    bool button_state = digitalRead(BOOT_BUTTON);
    
    if (button_state != last_button_state) {
        last_debounce = millis();
    }
    
    if (button_state == LOW && last_button_state == HIGH) {  // Button pressed
        Serial.println("\n*** PAIRING MODE ENABLED FOR 180 SECONDS ***");
        esp_zb_bdb_open_network(180);  // Open for 3 minutes
        pairing_mode = true;
        pairing_start_time = millis();
    }
    
    last_button_state = button_state;
    
    // Check if pairing window expired
    if (pairing_mode && (millis() - pairing_start_time > 180000)) {
        Serial.println("*** PAIRING MODE CLOSED ***");
        pairing_mode = false;
    }
    
    // Status update every 5 seconds
    static unsigned long last_status = 0;
    if (millis() - last_status > 5000) {
        if (pairing_mode) {
            unsigned long remaining = 180 - ((millis() - pairing_start_time) / 1000);
            Serial.printf("Pairing mode: %lu seconds remaining\n", remaining);
        } else {
            Serial.println("Coordinator running - Press BOOT to pair new devices");
        }
        last_status = millis();
    }

    if(esp_zb_bdb_dev_joined()){
        Serial.println("Device joined the network");
    }
}

// ================= ZIGBEE FUNCTIONS =================
void zigbee_task(void *arg){
    // Init Zigbee as COORDINATOR
    esp_zb_cfg_t zb_cfg;
    zb_cfg.esp_zb_role = ESP_ZB_DEVICE_TYPE_COORDINATOR;
    zb_cfg.install_code_policy = false;
    zb_cfg.nwk_cfg.zczr_cfg.max_children = 10;  // Allow 10 devices to join
    
    esp_zb_init(&zb_cfg);
    
    // Create custom cluster (as CLIENT to receive from endpoints)
    esp_zb_attribute_list_t *custom_cluster = esp_zb_zcl_attr_list_create(ASDL_CUSTOM_CLUSTER_ID);
    esp_zb_cluster_add_attr(custom_cluster, ASDL_CUSTOM_CLUSTER_ID, SENSOR_DATA_ATTR_ID, ESP_ZB_ZCL_ATTR_TYPE_OCTET_STRING, 
                            ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &agg_data);
    
    // Create basic cluster (coordinators need at least one)
    esp_zb_attribute_list_t *esp_zb_basic_cluster = esp_zb_basic_cluster_create(NULL);
    
    // Create cluster list
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_custom_cluster(cluster_list, custom_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
    esp_zb_cluster_list_add_basic_cluster(cluster_list, esp_zb_basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    
    // Create endpoint
    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    esp_zb_endpoint_config_t ep_cfg = {
        .endpoint = 1,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_ON_OFF_OUTPUT_DEVICE_ID
    };
    esp_zb_ep_list_add_ep(ep_list, cluster_list, ep_cfg);
    
    // Register and start
    esp_zb_device_register(ep_list);
    
    // Set network parameters
    esp_zb_set_primary_network_channel_set(ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK);
    
    esp_zb_start(false);
    esp_zb_main_loop_iteration();
}

// Attribute report handler - receives data from endpoint devices
static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message){
  if (message->info.cluster == ASDL_CUSTOM_CLUSTER_ID && message->attribute.id == SENSOR_DATA_ATTR_ID) {
      data_struct *data = (data_struct*)message->attribute.data.value;
      
      Serial.println("=== Received Data ===");
      Serial.printf("Temperature: %d\n", data->temperature);
      Serial.printf("Pressure: %d\n", data->pressure);
      Serial.printf("Altitude: %d\n", data->altitude);
      Serial.printf("CO2: %d\n", data->co2);
      Serial.printf("Battery Level: %d\n", data->battery_level);
      Serial.println("====================");
  }
  return ESP_OK;
}

// Action handler - for commands and reports
static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message){
  esp_err_t ret = ESP_OK;
  
  switch (callback_id) {
      case ESP_ZB_CORE_REPORT_ATTR_CB_ID:
          ret = zb_attribute_handler((esp_zb_zcl_set_attr_value_message_t *)message);
          break;
          
      default:
          break;
  }
  return ret;
}
