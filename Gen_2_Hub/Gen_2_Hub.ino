/***************************************************
  This Arduino Sketch is for the Hub device

  - Uses BMP388 sensor over I2C to validate incoming data
  - Communicates with Particle Boron 404x

****************************************************/
#include "Zigbee.h"

#define MAX_DEVICES 10

// Device data structure
struct DeviceData {
  uint16_t shortAddr;
  float temperature;      // EP 10
  float humidity;         // EP 11 (analog)
  float pm25;             // EP 12
  float pressure;         // EP 13
  float co2;              // EP 14 (analog)
  float altitude;         // EP 15 (analog)
  unsigned long lastSeen;
  bool active;
};

DeviceData devices[MAX_DEVICES];
int deviceCount = 0;

// Create light endpoint (makes this a coordinator)
ZigbeeLight zbLight = ZigbeeLight(1);

// Find or add device
int getDeviceIndex(uint16_t addr) {
  // Search for existing device
  for (int i = 0; i < deviceCount; i++) {
    if (devices[i].shortAddr == addr) {
      return i;
    }
  }
  
  // Add new device if space available
  if (deviceCount < MAX_DEVICES) {
    devices[deviceCount].shortAddr = addr;
    devices[deviceCount].active = true;
    devices[deviceCount].lastSeen = millis();
    Serial.printf("New device joined: 0x%04X (Device #%d)\n", addr, deviceCount + 1);
    return deviceCount++;
  }
  
  Serial.printf("Device limit reached! Cannot add 0x%04X\n", addr);
  return -1;
}

// Callback to receive sensor data
void onZigbeeData(esp_zb_zcl_addr_t *addr, esp_zb_zcl_attribute_t *attribute) {
  uint16_t shortAddr = addr->u.short_addr;
  uint16_t cluster_id = attribute->cluster_id;
  
  // Get or create device entry
  int idx = getDeviceIndex(shortAddr);
  if (idx < 0) return; // Device limit reached
  
  // Update last seen timestamp
  devices[idx].lastSeen = millis();
  
  // Parse sensor data based on cluster
  if (cluster_id == ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT) {
    int16_t temp_raw = *(int16_t*)attribute->data.value;
    devices[idx].temperature = temp_raw / 100.0;
    Serial.printf("[0x%04X] Temperature: %.2f°C\n", shortAddr, devices[idx].temperature);
  }
  else if (cluster_id == ESP_ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT) {
    int16_t pressure_raw = *(int16_t*)attribute->data.value;
    devices[idx].pressure = pressure_raw / 10.0; // kPa
    Serial.printf("[0x%04X] Pressure: %.1f kPa\n", shortAddr, devices[idx].pressure);
  }
  else if (cluster_id == ESP_ZB_ZCL_CLUSTER_ID_PM2_5_MEASUREMENT) {
    float pm25 = *(float*)attribute->data.value;
    devices[idx].pm25 = pm25;
    Serial.printf("[0x%04X] PM2.5: %.1f µg/m³\n", shortAddr, devices[idx].pm25);
  }
  else if (cluster_id == ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT) {
    float value = *(float*)attribute->data.value;
    Serial.printf("[0x%04X] Analog: %.2f\n", shortAddr, value);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  // Initialize device array
  for (int i = 0; i < MAX_DEVICES; i++) {
    devices[i].active = false;
    devices[i].shortAddr = 0xFFFF;
    devices[i].temperature = 0;
    devices[i].pressure = 0;
    devices[i].pm25 = 0;
    devices[i].lastSeen = 0;
  }
  
  // Start coordinator
  Zigbee.begin();
  
  Serial.println("Hub started and ready");
  Serial.println("Waiting for spoke devices to join...\n");
}

void loop() {
  static unsigned long lastStatusPrint = 0;
  unsigned long now = millis();
  
  // Print status every 30 seconds
  if (now - lastStatusPrint > 30000) {
    Serial.println("\n--- Device Status ---");
    Serial.printf("Connected devices: %d/%d\n", deviceCount, MAX_DEVICES);
    
    for (int i = 0; i < deviceCount; i++) {
      unsigned long timeSince = (now - devices[i].lastSeen) / 1000;
      Serial.printf("Device 0x%04X: ", devices[i].shortAddr);
      Serial.printf("Temp=%.1f°C, Press=%.1fkPa, PM2.5=%.1f µg/m³, ", 
                    devices[i].temperature, 
                    devices[i].pressure, 
                    devices[i].pm25);
      Serial.printf("Last seen: %lus ago\n", timeSince);
    }
    Serial.println("--------------------\n");
    
    lastStatusPrint = now;
  }
  
  delay(100);
}