#ifndef __CONNECT_H
#define __CONNECT_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

// ================== 用户配置区域 ==================
#define MQTT_BROKER_IP    "47.108.128.105"   // IP
#define MQTT_BROKER_PORT  1883             // 端口
#define MQTT_CLIENT_ID    "bc28_mcu_dev01" 
#define MQTT_USER         "admin"          
#define MQTT_PASS         "public"         
#define MQTT_TOPIC        "test/topic"     // 发布主题

#define SUB_TOPIC   "cmd/led"      // 接收控制指令

/* ================== OTA 主题 ================== */
#define OTA_DEVICE_ID       "citrus_001"
#define OTA_DOWN_TOPIC      "device/citrus_001/ota/down"   /* 固件下发 */
#define OTA_UP_TOPIC        "device/citrus_001/ota/up"     /* 设备应答 */

#define RX_BUFFER_SIZE 256
extern uint8_t RxBuffer[RX_BUFFER_SIZE];
extern volatile uint16_t RxIndex;

void u3_printf(char* fmt, ...);
// 功能函数声明
void BC28_Init_Hardware(void);          // 初始化底层串口
bool BC28_Check_Status(void);           // 检查模块状态 (AT, CPIN, CSQ, CGATT)
bool BC28_Connect_Server(void);         // 连接 MQTT 服务器 (QMTOPEN, QMTCONN)
bool BC28_Publish_Data(int id); // 发送数据JSON
bool BC28_Publish_Anomaly(const char *json_payload); // 发布异常信息
bool BC28_Publish_OTA_Report(const char *json_payload); // 发布 OTA 应答
void BC28_SystemReset(void);         // 软件复位

#endif
