#include "iaq_manager.h"
#include "esp_zigbee_core.h"
#include "esp_zigbee_cluster.h"
#include "esp_zigbee_endpoint.h"
#include <string.h>
#include <time.h>

static device_info_t devices[MAX_DEVICES];
int numDevices = 0;

bool iaq_manager_add_device(uint16_t short_addr, esp_zb_ieee_addr_t ieee_addr){
    if(numDevices < 10){
        if(iaq_manager_get_device(short_addr) != NULL){ // Reconnect
            devices[numDevices].active = true;
            devices[numDevices].last_ping = pdTICKS_TO_MS(xTaskGetTickCount());
            ESP_LOGI(TAG, "Device 0x%04x reconnected", short_addr);
        } else {
            devices[numDevices].short_addr = short_addr;
            memcpy(devices[numDevices].ieee_addr, ieee_addr, sizeof(esp_zb_ieee_addr_t));
            devices[numDevices].active = true;
            devices[numDevices].last_ping = pdTICKS_TO_MS(xTaskGetTickCount());
            numDevices++;
            ESP_LOGI(TAG, "Device 0x%04x added to list", short_addr);
        }
    } else {
        ESP_LOGI(TAG, "Device list is full. Cannot add more devices.");
        return false;
    }
    return true;
}

int iaq_manager_get_number_of_devices(){
    return numDevices;
}

bool iaq_manager_check_all_endpoint_status(){
    for (int i = 0; i < numDevices; i++) {
        if(devices[i].active == false){
            return false;
        }
    }
    return true;
}

IAQ_Collected_Data iaq_manager_get_avg_data(){
    IAQ_Collected_Data avg_data = {0};
    int accounted_devices = 0;
    
    for(int i = 0; i < numDevices; i++){
        device_info_t dev = devices[i];
        if(dev.active){
            avg_data.altitude += dev.altitude;
            avg_data.co2 += dev.co2;
            avg_data.humidity += dev.humidity;
            avg_data.pm25 += dev.pm25;
            avg_data.pressure += dev.pressure;
            avg_data.temperature += dev.temperature;
            accounted_devices++;
        }
    }

    if(accounted_devices > 0){
        avg_data.altitude /= accounted_devices;
        avg_data.co2 /= accounted_devices;
        avg_data.humidity /= accounted_devices;
        avg_data.pm25 /= accounted_devices; 
        avg_data.pressure /= accounted_devices;
        avg_data.temperature /= accounted_devices;
    }

    avg_data.num_devices_included = accounted_devices;
    return avg_data;
}

device_info_t* iaq_manager_get_all_devices(){
    return devices;
}

void iaq_manager_handle_incoming_report(esp_zb_zcl_report_attr_message_t *report){
    uint8_t endpoint = report->src_endpoint;
    uint16_t short_addr = report->src_address.u.short_addr;

    //ESP_LOGI(TAG, "Report from 0x%04x, attr 0x%04x", 
    //            short_addr, report->attribute.id);

    device_info_t *sender = iaq_manager_get_device(report->src_address.u.short_addr);
    
    if (sender == NULL) {
        ESP_LOGI(TAG, "Device 0x%04x not in device list, adding to list...", report->src_address.u.short_addr);
        
        esp_zb_ieee_addr_t ieee_addr;
        esp_zb_ieee_address_by_short(short_addr, ieee_addr);

        if(iaq_manager_add_device(short_addr, ieee_addr)){
            sender = iaq_manager_get_device(short_addr);
        } else {
            ESP_LOGW(TAG, "Failed to add device: max number of IAQ devices reached");
            return;
        }

        return;
    }

    sender->last_ping = xTaskGetTickCount();
    
    if (report->cluster == ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT) {
        int16_t temp_raw = *(int16_t*)report->attribute.data.value;
        float temp = temp_raw / 100.0f;
        //ESP_LOGI(TAG, "Temperature: %.2f°C", temp);
        sender->temperature = temp;
    }
    else if (report->cluster == ESP_ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT) {
        int16_t pressure_raw = *(int16_t*)report->attribute.data.value;
        float pressure = pressure_raw;
        //ESP_LOGI(TAG, "Pressure: %.2f hPa", pressure);
        sender->pressure = pressure;
    }
    else if (report->cluster == ESP_ZB_ZCL_CLUSTER_ID_PM2_5_MEASUREMENT) {
        int16_t pm25_raw = *(int16_t*)report->attribute.data.value;
        float pm25 = pm25_raw / 10.0;
        //ESP_LOGI(TAG, "PM2.5: %.1f μg/m³", pm25);
        sender->pm25 = pm25;
    }
    else if (report->cluster == ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT) {
        float value = *(float*)report->attribute.data.value;
        if (endpoint == 11) {
            //ESP_LOGI(TAG, "Humidity: %.2f%%", value);
            sender->humidity = value;
        } else if (endpoint == 14) {
            //ESP_LOGI(TAG, "CO2: %.2f ppm", value);
            sender->co2 = value;
        } else if (endpoint == 15) {
            //ESP_LOGI(TAG, "Altitude: %.2f m", value);
            sender->altitude = value;
        }
    }
}

device_info_t* iaq_manager_get_device(uint16_t short_addr){
    for(int i = 0; i < numDevices; i++){
        if((devices[i].short_addr) == (short_addr)){
            return &devices[i];
        }
    }
    return NULL;
}

void iaq_manager_handle_ping_response(uint16_t short_addr, bool success) {
    // Update the device in array
    for (int i = 0; i < numDevices; i++) {
        if (devices[i].short_addr == short_addr) {
            devices[i].active = success;
            devices[i].last_ping = time(NULL);
            if (success) {
                ESP_LOGI(TAG, "Device 0x%04x successful ping", short_addr);
            } else {
                ESP_LOGI(TAG, "Device 0x%04x failed ping", short_addr);
            }
            break;
        }
    }
}

void iaq_manager_send_keepalives(void) {
    for(int i = 0; i < numDevices; i++) {
        if(devices[i].active) {
            // Send keepalive write command
            esp_zb_zcl_write_attr_cmd_t write_req = {0};
            
            write_req.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
            write_req.zcl_basic_cmd.dst_addr_u.addr_short = devices[i].short_addr;
            write_req.zcl_basic_cmd.dst_endpoint = 16;
            write_req.zcl_basic_cmd.src_endpoint = 16;
            
            esp_zb_zcl_attribute_t attr;
            attr.id = ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_PRESENT_VALUE_ID;
            float keepalive_value = 1.0f;
            attr.data.type = ESP_ZB_ZCL_ATTR_TYPE_SINGLE;
            attr.data.value = &keepalive_value;
            attr.data.size = sizeof(keepalive_value);
            
            write_req.clusterID = ESP_ZB_ZCL_CLUSTER_ID_ANALOG_OUTPUT;
            write_req.attr_number = 1;
            write_req.attr_field = &attr;
            
            esp_zb_zcl_write_attr_cmd_req(&write_req);
            
            vTaskDelay(pdMS_TO_TICKS(500));  // Small delay between devices
        }
    }
}

bool iaq_manager_check_device_timeouts(){
    uint32_t now = xTaskGetTickCount();
    bool all_endpoint_status = iaq_manager_check_all_endpoint_status();

    if(!all_endpoint_status){
        for (int i = 0; i < numDevices; i++) {
            if (devices[i].active && devices[i].last_ping > 0) {
                uint32_t time_since = pdTICKS_TO_MS(now - devices[i].last_ping);
                
                if (time_since > DEVICE_TIMEOUT_MS) {
                    ESP_LOGW(TAG, "Device 0x%04x timed out - no data for %lu minutes", 
                            devices[i].short_addr, time_since / 60000);
                    devices[i].active = false;
                }
            }
        }
    }

    ESP_LOGI(TAG, "IAQ Network Status: %s", all_endpoint_status ? "Good" : "Bad");
    return all_endpoint_status;
}

void iaq_mgr_print_data_snapshot(){
    IAQ_Collected_Data snapshot = iaq_manager_get_avg_data();

    ESP_LOGI(TAG, "========= START OF SNAPSHOT =========");
    ESP_LOGI(TAG, "Temp: %.2f°C", snapshot.temperature);
    ESP_LOGI(TAG, "Humidity: %.2f%%", snapshot.humidity);
    ESP_LOGI(TAG, "PM25: %.2f μg/m³", snapshot.pm25);
    ESP_LOGI(TAG, "Pressure: %.2f hPa", snapshot.pressure);
    ESP_LOGI(TAG, "CO2: %.2f ppm", snapshot.co2);
    ESP_LOGI(TAG, "Altitude: %.2f meters", snapshot.altitude);
    ESP_LOGI(TAG, "# of Included Devices: %d", snapshot.num_devices_included);
    ESP_LOGI(TAG, "========== END OF SNAPSHOT ==========");
}