#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_zigbee_core.h"
#include <string.h>
#include "led_driver.c"
#include "data_bridge_manager.c"
#include "SpokeManagers/iaq_manager.h"

#define TAG "ASDL_ZIGBEE_COORDINATOR"
#define MAX_DEVICES 25
#define TIMEOUT_CHECK_INTERVAL 1    // minutes
#define SHIPPING_INTERVAL 10         // minutes

static uint16_t connected_devices[MAX_DEVICES] = {0};
static int device_count = 0;

// TODO Fix Me
typedef enum {
    DEVICE_TYPE_UNKNOWN = 0,
    DEVICE_TYPE_IAQ_SPOKE,
    DEVICE_TYPE_POWER_SPOKE,
    DEVICE_TYPE_MOTION_SPOKE,
} device_type_t;

/*
 *  Utility Functions
 */
static void bind_device_clusters(uint16_t short_addr, esp_zb_ieee_addr_t ieee_addr)
{
    ESP_LOGI(TAG, "Binding device 0x%04x", short_addr);
    
    esp_zb_zdo_bind_req_param_t bind_req;
    esp_zb_ieee_addr_t coord_ieee;
    esp_zb_get_long_address(coord_ieee);
    
    memcpy(bind_req.src_address, ieee_addr, sizeof(esp_zb_ieee_addr_t));
    bind_req.dst_addr_mode = ESP_ZB_ZDO_BIND_DST_ADDR_MODE_64_BIT_EXTENDED;
    memcpy(bind_req.dst_address_u.addr_long, coord_ieee, sizeof(esp_zb_ieee_addr_t));
    bind_req.req_dst_addr = short_addr;
    
    // Bind Temperature (endpoint 10)
    bind_req.src_endp = 10;
    bind_req.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT;
    bind_req.dst_endp = 10;
    esp_zb_zdo_device_bind_req(&bind_req, NULL, NULL);
    vTaskDelay(pdMS_TO_TICKS(200));
    
    // Bind Humidity (endpoint 11)
    bind_req.src_endp = 11;
    bind_req.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT;
    bind_req.dst_endp = 11;
    esp_zb_zdo_device_bind_req(&bind_req, NULL, NULL);
    vTaskDelay(pdMS_TO_TICKS(200));
    
    // Bind PM2.5 (endpoint 12)
    bind_req.src_endp = 12;
    bind_req.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_PM2_5_MEASUREMENT;
    bind_req.dst_endp = 12;
    esp_zb_zdo_device_bind_req(&bind_req, NULL, NULL);
    vTaskDelay(pdMS_TO_TICKS(200));
    
    // Bind Pressure (endpoint 13)
    bind_req.src_endp = 13;
    bind_req.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT;
    bind_req.dst_endp = 13;
    esp_zb_zdo_device_bind_req(&bind_req, NULL, NULL);
    vTaskDelay(pdMS_TO_TICKS(200));
    
    // Bind CO2 (endpoint 14)
    bind_req.src_endp = 14;
    bind_req.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT;
    bind_req.dst_endp = 14;
    esp_zb_zdo_device_bind_req(&bind_req, NULL, NULL);
    vTaskDelay(pdMS_TO_TICKS(200));
    
    // Bind Altitude (endpoint 15)
    bind_req.src_endp = 15;
    bind_req.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT;
    bind_req.dst_endp = 15;
    esp_zb_zdo_device_bind_req(&bind_req, NULL, NULL);

    // Bind Keepalive (endpoint 16)
    bind_req.src_endp = 16;
    bind_req.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_ANALOG_OUTPUT;
    bind_req.dst_endp = 16;
    esp_zb_zdo_device_bind_req(&bind_req, NULL, NULL);
    
    ESP_LOGI(TAG, "Bindings created. Adding device to manager.");

    flash_color(0, 0, 255);

    iaq_manager_add_device(short_addr, ieee_addr);
    ESP_LOGI(TAG, "Number of IAQ devices %d", iaq_manager_get_number_of_devices());
}

static void poll_all_sensors(uint16_t short_addr)
{
    esp_zb_zcl_read_attr_cmd_t read_req = {0};
    read_req.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    read_req.zcl_basic_cmd.dst_addr_u.addr_short = short_addr;
    read_req.attr_number = 1;
    
    uint16_t temp_attr = ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID;
    uint16_t analog_attr = ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID;
    uint16_t pm25_attr = ESP_ZB_ZCL_ATTR_PM2_5_MEASUREMENT_MEASURED_VALUE_ID;
    uint16_t pressure_attr = ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_VALUE_ID;
    
    // Temperature
    read_req.zcl_basic_cmd.dst_endpoint = 10;
    read_req.zcl_basic_cmd.src_endpoint = 10;
    read_req.clusterID = ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT;
    read_req.attr_field = &temp_attr;
    esp_zb_zcl_read_attr_cmd_req(&read_req);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Humidity
    read_req.zcl_basic_cmd.dst_endpoint = 11;
    read_req.zcl_basic_cmd.src_endpoint = 11;
    read_req.clusterID = ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT;
    read_req.attr_field = &analog_attr;
    esp_zb_zcl_read_attr_cmd_req(&read_req);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // PM2.5
    read_req.zcl_basic_cmd.dst_endpoint = 12;
    read_req.zcl_basic_cmd.src_endpoint = 12;
    read_req.clusterID = ESP_ZB_ZCL_CLUSTER_ID_PM2_5_MEASUREMENT;
    read_req.attr_field = &pm25_attr;
    esp_zb_zcl_read_attr_cmd_req(&read_req);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Pressure
    read_req.zcl_basic_cmd.dst_endpoint = 13;
    read_req.zcl_basic_cmd.src_endpoint = 13;
    read_req.clusterID = ESP_ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT;
    read_req.attr_field = &pressure_attr;
    esp_zb_zcl_read_attr_cmd_req(&read_req);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // CO2
    read_req.zcl_basic_cmd.dst_endpoint = 14;
    read_req.zcl_basic_cmd.src_endpoint = 14;
    read_req.clusterID = ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT;
    read_req.attr_field = &analog_attr;
    esp_zb_zcl_read_attr_cmd_req(&read_req);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Altitude
    read_req.zcl_basic_cmd.dst_endpoint = 15;
    read_req.zcl_basic_cmd.src_endpoint = 15;
    read_req.clusterID = ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT;
    read_req.attr_field = &analog_attr;
    esp_zb_zcl_read_attr_cmd_req(&read_req);
}

static void monitoring_task(void *pvParameters){
    ESP_LOGI(TAG, "Starting Device Monitoring Task");

    while(1){
        iaq_manager_check_device_timeouts();
        vTaskDelay(pdMS_TO_TICKS(TIMEOUT_CHECK_INTERVAL * 60000));
    }
}

void keepalive_task(void *pvParameters) {
    ESP_LOGI(TAG, "Keepalive task started");
    
    // Wait for network to form before sending keepalives
    vTaskDelay(pdMS_TO_TICKS(10000));
    
    while(1) {
        iaq_manager_send_keepalives();
        vTaskDelay(pdMS_TO_TICKS(30000));  // Send every 30 seconds
    }
}

static void data_bridge_task(void *pvParameters){
    ESP_LOGI(TAG, "Starting Data Bridge Task");

    while(1){
        bridge_mgr_ship_iaq_data(iaq_manager_get_packaged_data());
        vTaskDelay(pdMS_TO_TICKS(SHIPPING_INTERVAL * 60000));
    }
}

/*
 *  Zigbee Functions
 */
static void zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{    
    if(callback_id == ESP_ZB_CORE_REPORT_ATTR_CB_ID) {
        esp_zb_zcl_report_attr_message_t *report = (esp_zb_zcl_report_attr_message_t *)message;

        // Ignore random stray Zigbee clusters in the area
        if(
            report->cluster != ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT ||
            report->cluster != ESP_ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT ||
            report->cluster != ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT ||
            report->cluster != ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT
        ) {
            iaq_manager_handle_incoming_report(report);
        } else {
            ESP_LOGW(TAG, "Ignoring stray Zigbee device.");
        }
    }
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = (esp_zb_app_signal_type_t)*p_sg_p;
    
    switch (sig_type) {
        case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
            break;
            
        case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
        case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
            if (err_status == ESP_OK) {
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_FORMATION);
            }
            break;
            
        case ESP_ZB_BDB_SIGNAL_FORMATION:
            if (err_status == ESP_OK) {
                ESP_LOGI(TAG, "Network formed - PAN ID: 0x%04X, Channel: %d", 
                         esp_zb_get_pan_id(), esp_zb_get_current_channel());
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            } else {
                esp_zb_scheduler_alarm((esp_zb_callback_t)esp_zb_bdb_start_top_level_commissioning, 
                                      ESP_ZB_BDB_MODE_NETWORK_FORMATION, 1000);
            }
            break;
            
        case ESP_ZB_ZDO_SIGNAL_DEVICE_ANNCE: {
            esp_zb_zdo_signal_device_annce_params_t *dev_annce_params = 
                (esp_zb_zdo_signal_device_annce_params_t *)esp_zb_app_signal_get_params(p_sg_p);
            ESP_LOGI(TAG, "Device joined: 0x%04x", dev_annce_params->device_short_addr);
            
            if (device_count < MAX_DEVICES) {
                connected_devices[device_count++] = dev_annce_params->device_short_addr;
            }
            
            vTaskDelay(pdMS_TO_TICKS(2000));
            
            esp_zb_ieee_addr_t ieee_addr;
            esp_zb_ieee_address_by_short(dev_annce_params->device_short_addr, ieee_addr);
            bind_device_clusters(dev_annce_params->device_short_addr, ieee_addr);
            
            vTaskDelay(pdMS_TO_TICKS(2000));
            poll_all_sensors(dev_annce_params->device_short_addr);
            break;
        }
        
        default:
            break;
    }
}

static void esp_zb_task(void *pvParameters)
{
    esp_zb_cfg_t zb_nwk_cfg = {
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_COORDINATOR,
        .install_code_policy = false,
        .nwk_cfg = {
            .zczr_cfg = {
                .max_children = 10,
            },
        },
    };
    esp_zb_init(&zb_nwk_cfg);
    esp_zb_core_action_handler_register(zb_action_handler);
    
    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    
    // Endpoint 10: Temperature
    esp_zb_cluster_list_t *ep10_clusters = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_basic_cluster(ep10_clusters, esp_zb_basic_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_attribute_list_t *temp_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT);
    int16_t temp_value = 0;
    esp_zb_temperature_meas_cluster_add_attr(temp_cluster, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, &temp_value);
    esp_zb_cluster_list_add_temperature_meas_cluster(ep10_clusters, temp_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
    esp_zb_ep_list_add_ep(ep_list, ep10_clusters, 10, ESP_ZB_AF_HA_PROFILE_ID, ESP_ZB_HA_TEMPERATURE_SENSOR_DEVICE_ID);
    
    // Endpoint 11: Humidity
    esp_zb_cluster_list_t *ep11_clusters = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_basic_cluster(ep11_clusters, esp_zb_basic_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_analog_input_cluster(ep11_clusters, esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT), ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
    esp_zb_ep_list_add_ep(ep_list, ep11_clusters, 11, ESP_ZB_AF_HA_PROFILE_ID, ESP_ZB_HA_CUSTOM_ATTR_DEVICE_ID);
    
    // Endpoint 12: PM2.5
    esp_zb_cluster_list_t *ep12_clusters = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_basic_cluster(ep12_clusters, esp_zb_basic_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_pm2_5_measurement_cluster(ep12_clusters, esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_PM2_5_MEASUREMENT), ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
    esp_zb_ep_list_add_ep(ep_list, ep12_clusters, 12, ESP_ZB_AF_HA_PROFILE_ID, ESP_ZB_HA_CUSTOM_ATTR_DEVICE_ID);
    
    // Endpoint 13: Pressure
    esp_zb_cluster_list_t *ep13_clusters = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_basic_cluster(ep13_clusters, esp_zb_basic_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_attribute_list_t *pressure_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT);
    int16_t pressure_value = 0;
    esp_zb_pressure_meas_cluster_add_attr(pressure_cluster, ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_VALUE_ID, &pressure_value);
    esp_zb_cluster_list_add_pressure_meas_cluster(ep13_clusters, pressure_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
    esp_zb_ep_list_add_ep(ep_list, ep13_clusters, 13, ESP_ZB_AF_HA_PROFILE_ID, ESP_ZB_HA_CUSTOM_ATTR_DEVICE_ID);
    
    // Endpoint 14: CO2
    esp_zb_cluster_list_t *ep14_clusters = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_basic_cluster(ep14_clusters, esp_zb_basic_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_analog_input_cluster(ep14_clusters, esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT), ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
    esp_zb_ep_list_add_ep(ep_list, ep14_clusters, 14, ESP_ZB_AF_HA_PROFILE_ID, ESP_ZB_HA_CUSTOM_ATTR_DEVICE_ID);
    
    // Endpoint 15: Altitude
    esp_zb_cluster_list_t *ep15_clusters = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_basic_cluster(ep15_clusters, esp_zb_basic_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_analog_input_cluster(ep15_clusters, esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT), ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
    esp_zb_ep_list_add_ep(ep_list, ep15_clusters, 15, ESP_ZB_AF_HA_PROFILE_ID, ESP_ZB_HA_CUSTOM_ATTR_DEVICE_ID);
    
    // Endpoint 16: IAQ Keepalive
    esp_zb_cluster_list_t *ep16_clusters = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_basic_cluster(ep16_clusters, esp_zb_basic_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_analog_output_cluster(ep16_clusters, esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_ANALOG_OUTPUT), ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
    esp_zb_ep_list_add_ep(ep_list, ep16_clusters, 16, ESP_ZB_AF_HA_PROFILE_ID, ESP_ZB_HA_CUSTOM_ATTR_DEVICE_ID);

    esp_zb_device_register(ep_list);
    esp_zb_set_primary_network_channel_set(ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK);
    
    ESP_LOGI(TAG, "Starting coordinator");
    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_main_loop_iteration();    
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    esp_zb_platform_config_t config = {
        .radio_config = {.radio_mode = RADIO_MODE_NATIVE},
        .host_config = {.host_connection_mode = HOST_CONNECTION_MODE_NONE},
    };
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    init_uart();
    
    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
    xTaskCreate(monitoring_task, "IAQ_Monitor", 4096, NULL, 4, NULL);
    xTaskCreate(keepalive_task, "Keepalive_Task", 4096, NULL, 4, NULL);
    xTaskCreate(data_bridge_task, "Data_Bridge", 4096, NULL, 4, NULL);
}