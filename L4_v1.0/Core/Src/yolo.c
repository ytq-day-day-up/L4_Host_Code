#include "yolo.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "usart.h"
#include "Connect.h"

// YOLO 环形缓冲区
#define RX3_BUFFER_SIZE 64
uint8_t rx3_buffer[RX3_BUFFER_SIZE];
volatile uint16_t rx3_head = 0;
volatile uint16_t rx3_tail = 0;

// 状态机
static uint8_t yolo_rx_state = 0;  // 0: 等待帧头 0x01, 1: 等待类别ID

// 疾病名称映射
static const char *disease_names[] = {
    "blackspot",  // 0
    "canker",     // 1
    "greening",   // 2
    "healthy",    // 3 
    "melanose"    // 4
};

// 初始化 UART3 接收中断
void YOLO_Init(void) {
    HAL_UART_Receive_IT(&huart3, &rx3_buffer[rx3_head], 1);
}

// 从环形缓冲区读取一个字节
static bool YOLO_ReadByte(uint8_t *byte) {
    if (rx3_head != rx3_tail) {
        *byte = rx3_buffer[rx3_tail];
        rx3_tail = (rx3_tail + 1) & (RX3_BUFFER_SIZE - 1);
        return true;
    }
    return false;
}

// 处理一次接收（在主循环中轮询调用）
void YOLO_Process(void) {
    uint8_t data;
    while (YOLO_ReadByte(&data)) {
        if (yolo_rx_state == 0) {
            // 等待帧头 0x01
            if (data == 0x01) {
                yolo_rx_state = 1;
            }
        } else {
            // 收到类别 ID
            yolo_rx_state = 0;

            if (data <= 4 && data != 3) {
                YOLO_Handle_Detection(data);
            }
        }
    }
}

// 处理检测结果，发布到 MQTT
void YOLO_Handle_Detection(uint8_t class_id) {
    if (class_id > 4) return;

    char json_buf[128];
    memset(json_buf, 0, sizeof(json_buf));
    sprintf(json_buf, "{\"disease\":\"%s\",\"id\":%d}",
            disease_names[class_id], class_id);

    BC28_Publish_Anomaly(json_buf);
}
