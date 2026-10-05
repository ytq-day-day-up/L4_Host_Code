#ifndef __MQTT_TASK_H
#define __MQTT_TASK_H

#include "main.h"


// 检查并处理来自云端的 MQTT 下发指令
void Process_MQTT_Downlink(uint8_t *rx_buffer, uint16_t len);

#endif
