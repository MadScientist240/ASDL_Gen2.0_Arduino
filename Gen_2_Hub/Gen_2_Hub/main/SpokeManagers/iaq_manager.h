#ifndef IAQ_MANAGER_H
#define IAQ_MANAGER_H
#include <stdint.h>
#include <esp_zigbee_type.h>
#include <zcl/esp_zigbee_zcl_command.h>

typedef struct {
  uint16_t short_addr;
  esp_zb_ieee_addr_t ieee_addr;
  uint32_t last_ping;
  bool active;

  float temperature;    // °C
  float humidity;       // %  
  float pm25;           // μg/m³
  float pressure;       // hPa
  float co2;            // ppm
  float altitude;       // meters
} device_info_t;

typedef struct {
  float temperature;    // °C
  float humidity;       // %  
  float pm25;           // μg/m³
  float pressure;       // hPa
  float co2;            // ppm
  float altitude;       // meters
  int num_devices_included;
} IAQ_Collected_Data;

#define MAX_DEVICES 10
#define TAG "IAQ_MANAGER"
#define DEVICE_TIMEOUT_MS (60 * 1000)



bool iaq_manager_add_device(uint16_t short_addr, esp_zb_ieee_addr_t ieee_addr);

bool iaq_manager_check_all_endpoint_status();

int iaq_manager_get_number_of_devices();

IAQ_Collected_Data iaq_manager_get_avg_data();

device_info_t* iaq_manager_get_all_devices();

void iaq_manager_handle_incoming_report(esp_zb_zcl_report_attr_message_t *report);

void iaq_manager_handle_ping_response(uint16_t short_addr, bool success);

void iaq_manager_send_keepalives(void);

bool iaq_manager_check_device_timeouts();

device_info_t* iaq_manager_get_device(uint16_t short_addr);

void iaq_mgr_print_data_snapshot();

#endif