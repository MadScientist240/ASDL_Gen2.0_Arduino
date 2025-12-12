#include "data_bridge_manager.h"
#include <string.h>


// Take packaged struct, wrap for serial, and send
void bridge_mgr_ship_iaq_data(IAQ_Collected_Data iaq) {

    char buffer[200];
    snprintf(buffer, sizeof(buffer),
    "{\"temp\":%.2f,\"hum\":%.2f,\"pm25\":%.2f,\"press\":%.2f,\"co2\":%.2f,\"alt\":%.2f,\"devices\":%d}\n",
    iaq.temperature, iaq.humidity, iaq.pm25,
    iaq.pressure, iaq.co2, iaq.altitude,
    iaq.num_devices_included);

    uart_write_bytes(UART_NUM_1, buffer, strlen(buffer));
    
    ESP_LOGI(TAG, "Sent IAQ data");
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
    uart_set_pin(UART_NUM_1, 5, 4, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}
