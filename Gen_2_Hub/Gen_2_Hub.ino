/***************************************************
  This Arduino Sketch is for the Hub device

  - Uses BMP388 sensor over I2C to validate incoming data
  - Communicates with Particle Boron 404x

****************************************************/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
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
  uint16_t device_addr;
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

// LCD Info
LiquidCrystal_I2C lcd(0x27, 16, 2);
static uint8_t NUM_DEVICES_CONNECTED = 0;
static bool BORON_UPLINK_STATUS = false;
static uint16_t devices[10];

// Pairing state
bool pairing_mode = false;
unsigned long pairing_start_time = 0;


void setup() {
  Wire.begin(6, 7, 100000);
  Serial.begin(115200);

  // LCD init
    lcd.init();
    lcd.backlight();
    lcd.clear();

  // Setup pairing button
  pinMode(BOOT_BUTTON, INPUT_PULLUP);

  Serial.println("Starting Zigbee Coordinator...");
  
  // Zigbee Init
  nvs_flash_init();
  // Register action handler
  esp_zb_core_action_handler_register(zb_action_handler);
  // Start Zigbee task
  xTaskCreate(zigbee_task, "zigbee", 4096, NULL, 5, NULL);
}

void loop() {
  static bool group_joined = false;
    
  // Check if hub is ready
  if (!group_joined && Zigbee.connected()) {
      delay(1000); // Give it a moment to stabilize
      
      Serial.println("Adding hub to group 0x0001...");
      
      // Create a groups cluster add group command
      esp_zb_zcl_groups_add_group_cmd_t add_group_cmd;
      memset(&add_group_cmd, 0, sizeof(add_group_cmd));
      
      add_group_cmd.zcl_basic_cmd.src_endpoint = 1;
      add_group_cmd.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
      add_group_cmd.group_id = 0x0001;
      
      esp_err_t err = esp_zb_zcl_groups_add_group_cmd_req(&add_group_cmd);
      Serial.printf("Add to group result: %s\n", esp_err_to_name(err));
      
      group_joined = true;
  }


  update_lcd();
  
  delay(1000);
}

// ================== MISC FUNCTIONS ==================
void update_lcd(){
  lcd.setCursor(0, 0);
  lcd.print("Connections: "); 
  lcd.setCursor(14, 0);
  lcd.print(NUM_DEVICES_CONNECTED);

  lcd.setCursor(0, 1);
  lcd.print("Uplink: "); 
  lcd.setCursor(9, 1);
  lcd.print(String(BORON_UPLINK_STATUS));
}

// ================= ZIGBEE FUNCTIONS =================
void zigbee_task(void *arg){
  // Init Zigbee as COORDINATOR
  Serial.println("Zigbee task started");
  esp_zb_cfg_t zb_cfg;
  zb_cfg.esp_zb_role = ESP_ZB_DEVICE_TYPE_COORDINATOR;
  zb_cfg.install_code_policy = false;
  zb_cfg.nwk_cfg.zczr_cfg.max_children = 10;  // Allow 10 devices to join
  
  esp_zb_init(&zb_cfg);
  
  esp_zb_core_action_handler_register(zb_action_handler);

  // Create custom cluster (as CLIENT to receive from endpoints)
  esp_zb_attribute_list_t *custom_cluster = esp_zb_zcl_attr_list_create(ASDL_CUSTOM_CLUSTER_ID);
  esp_zb_cluster_add_attr(custom_cluster, ASDL_CUSTOM_CLUSTER_ID, SENSOR_DATA_ATTR_ID, ESP_ZB_ZCL_ATTR_TYPE_OCTET_STRING, 
                          ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &agg_data);
  
  // Create basic cluster (coordinators need at least one)
  esp_zb_attribute_list_t *esp_zb_basic_cluster = esp_zb_basic_cluster_create(NULL);

  esp_zb_attribute_list_t *groups_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_GROUPS);
  
  // Create cluster list
  esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
  esp_zb_cluster_list_add_custom_cluster(cluster_list, custom_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
  esp_zb_cluster_list_add_basic_cluster(cluster_list, esp_zb_basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
  esp_zb_cluster_list_add_groups_cluster(cluster_list, groups_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

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
  Serial.println("2. Device registered");
  esp_zb_set_primary_network_channel_set(ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK);
  
  esp_err_t err = esp_zb_start(true);
  if (err != ESP_OK) {
  Serial.println("ZIGBEE START FAILED!");
  while(1) delay(1000);
  }

  Serial.println("3. Zigbee started, entering main loop");

  while(1) {
    esp_zb_main_loop_iteration();
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// Attribute report handler - receives data from endpoint devices
static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message){
  if (message->info.cluster == ASDL_CUSTOM_CLUSTER_ID && message->attribute.id == SENSOR_DATA_ATTR_ID) {
    data_struct *data = (data_struct*)message->attribute.data.value;
    uint16_t addr = data->device_addr;

    // Check if new device
    for (int i = 0; i < NUM_DEVICES_CONNECTED; i++) {
        if (devices[i] == addr) break;  // Already counted
    }
    
    // New device
    devices[NUM_DEVICES_CONNECTED++] = addr;
    Serial.printf("Total devices: %d\n", NUM_DEVICES_CONNECTED);
          
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
  
  Serial.printf(">>> Action handler called! Callback ID: %d\n", callback_id);
    // Print ALL possible callback IDs so we can see what we're actually getting
  Serial.printf("Checking callbacks - REPORT_ATTR=%d, CUSTOM_CMD=%d, SET_ATTR=%d\n",
                ESP_ZB_CORE_REPORT_ATTR_CB_ID,
                ESP_ZB_CORE_CMD_CUSTOM_CLUSTER_REQ_CB_ID,
                ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID);

  switch (callback_id) {
    case ESP_ZB_CORE_REPORT_ATTR_CB_ID:
      Serial.println("REPORT ATTR Recieved");
      ret = zb_attribute_handler((esp_zb_zcl_set_attr_value_message_t *)message);
    break;

    case ESP_ZB_CORE_CMD_CUSTOM_CLUSTER_REQ_CB_ID:
      Serial.println("CUSTOM COMMAND Received");
      esp_zb_zcl_custom_cluster_command_message_t *cmd = (esp_zb_zcl_custom_cluster_command_message_t *)message;
      data_struct *data = (data_struct*)cmd->data.value;
      uint16_t addr = data->device_addr;
      
      // Check if new device
      bool found = false;
      for (int i = 0; i < NUM_DEVICES_CONNECTED; i++) {
        if (devices[i] == addr) {
          found = true;
          break;
        }
      }
      
      if (!found) {
        devices[NUM_DEVICES_CONNECTED++] = addr;
        Serial.printf("New device! Total: %d\n", NUM_DEVICES_CONNECTED);
      }
      
      Serial.println("=== Received Data ===");
      Serial.printf("From: 0x%04x\n", addr);
      Serial.printf("Temperature: %.2f\n", data->temperature);
      Serial.printf("Pressure: %.2f\n", data->pressure);
      Serial.printf("CO2: %.2f\n", data->co2);
      Serial.println("====================");
    break;
  }
  return ret;
}