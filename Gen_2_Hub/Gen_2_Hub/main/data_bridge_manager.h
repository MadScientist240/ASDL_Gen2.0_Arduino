#ifndef DATA_BRIDGE_MANAGER
#define DATA_BRIDGE_MANAGER

#include "SpokeManagers/iaq_manager.h"

#define PACKET_START 0xAA
#define PACKET_END 0x55
#define MAX_PAYLOAD_SIZE 256
#define TAG "DATA_BRIDGE"

// Take avg struct, wrap for serial, and send
void bridge_mgr_ship_avg_iaq_data(IAQ_Collected_Data iaq);

// Take collected struct, wrap for serial, and send
void bridge_mgr_ship_iaq_data(device_info_t* devices, int numDevices);

#endif