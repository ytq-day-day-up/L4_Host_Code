#ifndef __YOLO_H
#define __YOLO_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

// 初始化 UART3 接收（启动中断）
void YOLO_Init(void);

// 主循环中轮询调用，解析树莓派发送的 YOLO 检测数据
void YOLO_Process(void);

// 处理检测结果并上传 MQTT
void YOLO_Handle_Detection(uint8_t class_id);

#endif
