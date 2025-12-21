#include "data_bridge_manager.h"
#include <string.h>


// Take averaged struct, wrap for serial, and send
void bridge_mgr_ship_avg_iaq_data(IAQ_Collected_Data iaq) {

    char buffer[200];
    snprintf(buffer, sizeof(buffer),
    "{\"temp\":%.2f,\"hum\":%.2f,\"pm25\":%.2f,\"press\":%.2f,\"co2\":%.2f,\"alt\":%.2f,\"devices\":%d}\n",
    iaq.temperature, iaq.humidity, iaq.pm25,
    iaq.pressure, iaq.co2, iaq.altitude,
    iaq.num_devices_included);

    uart_write_bytes(UART_NUM_1, buffer, strlen(buffer));
    
    ESP_LOGI(TAG, "Sent IAQ data");
}

// Take collected structs, wrap for serial, and send
void bridge_mgr_ship_iaq_data(device_info_t* devices, int numDevices) {
    ESP_LOGI(TAG, "=== START bridge_mgr_ship_iaq_data ===");
    ESP_LOGI(TAG, "numDevices: %d", numDevices);
    ESP_LOGI(TAG, "devices pointer: %p", devices);
    
    // Check if devices is NULL
    if (devices == NULL) {
        ESP_LOGE(TAG, "ERROR: devices pointer is NULL!");
        return;
    }
    
    // Check if numDevices is valid
    if (numDevices <= 0) {
        ESP_LOGE(TAG, "ERROR: numDevices is %d (should be > 0)", numDevices);
        return;
    }
    
    char* buffer = malloc(2048);
    int len = 0;
    
    len += sprintf(buffer + len, "START|%d", numDevices);
    ESP_LOGI(TAG, "After START, len = %d", len);
    
    for (int i = 0; i < numDevices; i++) {
        ESP_LOGI(TAG, "Processing device %d: short_addr=0x%04x", i, devices[i].short_addr);
        
        int written = sprintf(buffer + len, "|ID:0x%04x,ACT:%s,T:%.2f,H:%.2f,PM:%.2f,P:%.2f,C:%.2f,A:%.2f",
            devices[i].short_addr,
            devices[i].active ? "Y" : "N",
            devices[i].temperature,
            devices[i].humidity,
            devices[i].pm25,
            devices[i].pressure,
            devices[i].co2,
            devices[i].altitude);
        
        len += written;
        ESP_LOGI(TAG, "After device %d, len = %d", i, len);
    }
    
    len += sprintf(buffer + len, "|END\n");
    ESP_LOGI(TAG, "Final len = %d", len);
    
    // Print the actual buffer
    ESP_LOGI(TAG, "Buffer contents: %s", buffer);
    
    int bytes_written = uart_write_bytes(UART_NUM_1, buffer, len);
    ESP_LOGI(TAG, "uart_write_bytes returned: %d", bytes_written);
    free(buffer);
    
    ESP_LOGI(TAG, "Sent IAQ Data");
}

void init_uart(){
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, 1024 * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, 4, 5, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}
