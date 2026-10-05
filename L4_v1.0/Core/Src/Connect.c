#include "Connect.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "usart.h"

// 外部辅助函数（替代 sprintf）
extern void my_itoa(int32_t val, char *buf);
extern void my_u16toa(uint16_t val, char *buf);
extern void my_ftoa(float val, char *buf, int decimals);

// 缓冲区定义
#define CONNECT_RX_BUFFER_SIZE 256
uint8_t ConnectRxBuffer[CONNECT_RX_BUFFER_SIZE] = {0};

// 外部变量声明
extern uint8_t rx_buffer[];

extern float wind_speed;

extern float air_temp;         
extern float air_humidity;     
extern uint16_t light_intensity;
extern float air_pressure;     

extern float soil_temp;        
extern float soil_humidity;    
extern float soil_ph;          

// 引入外部声明
extern uint8_t LPUART_ReadByte(uint8_t *byte);
extern void Process_MQTT_Downlink(uint8_t *rx_buffer, uint16_t len);
extern void LPUART_Flush(void);

extern uint8_t lpuart_buffer[];
extern volatile uint16_t lpuart_head;
extern volatile uint16_t lpuart_tail;

// ================= 底层串口发送函数 =================

// 发送一个字节
static void User_UART_SendByte(uint8_t data) {
    HAL_UART_Transmit(&hlpuart1, &data, 1, 100);
}

// 发送字符串
static void User_UART_SendString(char *str) {
    while (*str) {
        User_UART_SendByte(*str++);
    }
}

// 调试输出（已禁用，UART2用于数据采集）
static void BC28_Debug(const char *str) {
    (void)str;  // 空实现，保留函数签名避免调用处报错
}

// 调试输出换行（已禁用，UART2用于数据采集）
static void BC28_DebugLn(const char *str) {
    (void)str;  // 空实现，保留函数签名避免调用处报错
}

// 转发 BC28 的回复到调试串口 (UART2)
//void Forward_BC28_Response(void) {
//    char forward_buffer[300]; // 稍微加大一点防止溢出
//    // 简单格式化打印
//    int len = sprintf(forward_buffer, "[BC28]: %s\r\n", (char*)ConnectRxBuffer);
//    HAL_UART_Transmit(&huart2, (uint8_t*)forward_buffer, len, 1000);
//}

// 清空接收缓冲区
static void User_Clear_RxBuffer(void) {
    memset(ConnectRxBuffer, 0, sizeof(ConnectRxBuffer));
}

/* 已废弃：BC28_Rebuild_LPUART1_RX()
 * 原因：HAL_UART_AbortReceive() 会破坏 LPUART1 接收中断状态。
 * 现在统一使用 LPUART_Flush()（main.c 中实现）清空环形缓冲区。
 *
 * static void BC28_Rebuild_LPUART1_RX(void)
 * {
 *     HAL_UART_AbortReceive(&hlpuart1);
 *     ...
 * }
 */

// ================= 核心接收逻辑 =================

/**
 * @brief  等待特定响应
 * @param  expected_response: 期待收到的子字符串 (例如 "+QMTOPEN: 0,0" 或 ">")
 * @param  timeout_ms: 超时时间
 * @retval true: 找到了, false: 超时未找到
 */
/* 防止 Wait_For_Response 内部递归处理 MQTT 下行 */
static bool s_in_wait_response = false;

/**
 * @brief  等待特定响应 (已重构：从环形缓冲区读取)
 */
static bool Wait_For_Response(const char *expected_response, uint32_t timeout_ms)
{
    uint32_t start_time = HAL_GetTick();
    uint16_t index = 0;
    uint16_t exp_len = 0;

    if (expected_response == NULL)
        return false;

    exp_len = strlen(expected_response);
    if (exp_len == 0)
        return false;

    s_in_wait_response = true;
    User_Clear_RxBuffer();

    while ((HAL_GetTick() - start_time) < timeout_ms)
    {
        uint8_t data;

        if (LPUART_ReadByte(&data))
        {
            /*
             * 单字符响应，比如 ">"，直接判断当前字节。
             * 防止 ConnectRxBuffer 被旧数据塞满后错过 ">"。
             */
            if (exp_len == 1 && data == (uint8_t)expected_response[0])
            {
                if (index < CONNECT_RX_BUFFER_SIZE - 1)
                {
                    ConnectRxBuffer[index++] = data;
                    ConnectRxBuffer[index] = '\0';
                }

                s_in_wait_response = false;
                return true;
            }

            /*
             * 缓冲区快满时清空重来。
             * 不要让旧数据塞满后导致后续新响应永远无法进入缓冲。
             */
            if (index >= CONNECT_RX_BUFFER_SIZE - 1)
            {
                index = 0;
                memset(ConnectRxBuffer, 0, sizeof(ConnectRxBuffer));
            }

            ConnectRxBuffer[index++] = data;
            ConnectRxBuffer[index] = '\0';

            if (strstr((char *)ConnectRxBuffer, expected_response) != NULL)
            {
                s_in_wait_response = false;
                return true;
            }
        }
    }

    s_in_wait_response = false;
    return false;
}

// 发送简单的查询指令并打印结果 (用于 Check_Status)
static void BC28_Send_Cmd_Simple(char *cmd, uint32_t wait_time) {
    User_Clear_RxBuffer();
    User_UART_SendString(cmd);
    User_UART_SendString("\r\n");
    // 这里使用 Wait_For_Response 的变体，只要等待时间结束或收到 OK
    Wait_For_Response("OK", wait_time);
}


//检查模块基础状态
bool BC28_Check_Status(void) {
    // 增加一点间隔，防止指令堆积
    BC28_Send_Cmd_Simple("AT", 300);
    HAL_Delay(200);
    BC28_Send_Cmd_Simple("AT+CPIN?", 300);
    HAL_Delay(200);
    BC28_Send_Cmd_Simple("AT+CSQ", 300);
    HAL_Delay(200);
    
    // 循环等待网络附着
    uint32_t start_time = HAL_GetTick();
    while (HAL_GetTick() - start_time < 30000) { // 最多等 30 秒
        User_Clear_RxBuffer();
        User_UART_SendString("AT+CGATT?\r\n");
        if (Wait_For_Response("+CGATT:1", 2000)) {
            return true; // 附着成功
        }
        HAL_Delay(1000);
    }
    return false; // 超时未附着
}

// 连接 MQTT 服务器 
// 连接 MQTT 服务器并订阅
bool BC28_Connect_Server(void) {
    char cmd_buffer[128];
    
    User_UART_SendString("AT+QMTCLOSE=0\r\n");
    Wait_For_Response("OK", 2000); 
    HAL_Delay(500);
    
    strcpy(cmd_buffer, "AT+QMTOPEN=0,\"");
    strcat(cmd_buffer, MQTT_BROKER_IP);
    strcat(cmd_buffer, "\",");
    my_itoa((int32_t)MQTT_BROKER_PORT, cmd_buffer + strlen(cmd_buffer));
    strcat(cmd_buffer, "\r\n");
    User_Clear_RxBuffer();
    User_UART_SendString(cmd_buffer);
    
    if (!Wait_For_Response("+QMTOPEN: 0,0", 15000)) {
        return false;
    }

    strcpy(cmd_buffer, "AT+QMTCONN=0,\"");
    strcat(cmd_buffer, MQTT_CLIENT_ID);
    strcat(cmd_buffer, "\",\"");
    strcat(cmd_buffer, MQTT_USER);
    strcat(cmd_buffer, "\",\"");
    strcat(cmd_buffer, MQTT_PASS);
    strcat(cmd_buffer, "\"\r\n");
    User_Clear_RxBuffer();
    User_UART_SendString(cmd_buffer);
    
    if (!Wait_For_Response("+QMTCONN: 0,0,0", 10000)) {
        return false;
    }
    
    // 连接成功后，必须订阅 cmd/led 主题，否则收不到下发数据
    User_Clear_RxBuffer();
	User_UART_SendString("AT+QMTSUB=0,1,\"cmd/led\",0\r\n");
    Wait_For_Response("+QMTSUB:", 5000); // 等待订阅成功的反馈

    /* 订阅 OTA 固件下发主题 (QoS=1，减少 NB-IoT 下行丢帧) */
    User_Clear_RxBuffer();
    User_UART_SendString("AT+QMTSUB=0,1,\"device/citrus_001/ota/down\",1\r\n");
    if (!Wait_For_Response("+QMTSUB:", 5000)) {
        return false;
    }
    
    return true;
}

//发布数据
bool BC28_Publish_Data(int id) {
char cmd_buffer[128];
    char json_buffer[256]; // 加大缓冲区以容纳多数据的JSON

    // 根据 id 组装不同的 JSON 格式
    if (id == 1) {
        // ID=1: 风速数据
        strcpy(json_buffer, "{\"id\":1,\"wind\":");
        my_ftoa(wind_speed, json_buffer + strlen(json_buffer), 1);
        strcat(json_buffer, "}");
    } 
    else if (id == 2) {
        // ID=2: 空气传感器数据 (温、湿、光、压)
        strcpy(json_buffer, "{\"id\":2,\"temp\":");
        my_ftoa(air_temp, json_buffer + strlen(json_buffer), 1);
        strcat(json_buffer, ",\"hum\":");
        my_ftoa(air_humidity, json_buffer + strlen(json_buffer), 1);
        strcat(json_buffer, ",\"lux\":");
        my_u16toa(light_intensity, json_buffer + strlen(json_buffer));
        strcat(json_buffer, ",\"pa\":");
        my_ftoa(air_pressure, json_buffer + strlen(json_buffer), 1);
        strcat(json_buffer, "}");
    } 
    else if (id == 3) {
        // ID=3: 土壤传感器数据 (温、湿、pH)
        strcpy(json_buffer, "{\"id\":3,\"s_temp\":");
        my_ftoa(soil_temp, json_buffer + strlen(json_buffer), 1);
        strcat(json_buffer, ",\"s_hum\":");
        my_ftoa(soil_humidity, json_buffer + strlen(json_buffer), 1);
        strcat(json_buffer, ",\"ph\":");
        my_ftoa(soil_ph, json_buffer + strlen(json_buffer), 2);
        strcat(json_buffer, "}");
    } 
    else {
        return false; 
    }

    // 1. 发送 PUB 命令头
    strcpy(cmd_buffer, "AT+QMTPUB=0,0,0,0,\"");
    strcat(cmd_buffer, MQTT_TOPIC);
    strcat(cmd_buffer, "\"\r\n");    
    User_Clear_RxBuffer(); 
    HAL_UART_Transmit(&hlpuart1, (uint8_t*)cmd_buffer, strlen(cmd_buffer), 100); // 直接用HAL库发送避免依赖static函数问题

    // 2. 等待 ">"
    if (!Wait_For_Response(">", 5000)) {
        return false;
    }
    
    // 3. 发送 JSON 内容
    HAL_UART_Transmit(&hlpuart1, (uint8_t*)json_buffer, strlen(json_buffer), 100);
    
    // 4. 发送结束符 (Ctrl+Z -> 0x1A)
    uint8_t end_char = 0x1A;
    HAL_UART_Transmit(&hlpuart1, &end_char, 1, 100);
    
    // 5. 等待发布确认
    if (!Wait_For_Response("+QMTPUB: 0,0,0", 5000)) {
        return false;
    }
    
    return true;
}

// 发布异常信息 
bool BC28_Publish_Anomaly(const char *json_payload) {
    char cmd_buffer[128];
    
    // 1. 发送 PUB 命令头 (假设发布到 anomaly/report 这个具体主题，以符合 anomaly/#)
    strcpy(cmd_buffer, "AT+QMTPUB=0,0,0,0,\"anomaly/report\"\r\n");    
    User_Clear_RxBuffer(); 
    HAL_UART_Transmit(&hlpuart1, (uint8_t*)cmd_buffer, strlen(cmd_buffer), 100); 

    // 2. 等待 ">"
    if (!Wait_For_Response(">", 5000)) {
        return false;
    }
    
    // 3. 发送 JSON 内容
    HAL_UART_Transmit(&hlpuart1, (uint8_t*)json_payload, strlen(json_payload), 100);
    
    // 4. 发送结束符 (Ctrl+Z -> 0x1A)
    uint8_t end_char = 0x1A;
    HAL_UART_Transmit(&hlpuart1, &end_char, 1, 100);

    // QoS 0 无需等待 +QMTPUB: 确认，稍作延时即可返回
    HAL_Delay(50);

    return true;
}

/**
 * @brief  通用 MQTT 发布函数（稳定版）
 *         核心设计：不等 +QMTPUB: 确认，服务器收到 ACK 才是最终依据。
 *         使用 LPUART_Flush() 清环形缓冲，避免 AbortReceive 破坏中断状态。
 */
bool BC28_Publish_Raw(const char *topic, const char *payload)
{
    char cmd_buffer[180];

    if (topic == NULL || payload == NULL)
    {
        BC28_DebugLn("[BC28_PUB] topic or payload NULL");
        return false;
    }

    if (strlen(topic) > 100 || strlen(payload) > 220)
    {
        BC28_DebugLn("[BC28_PUB] topic/payload too long");
        return false;
    }

    snprintf(cmd_buffer, sizeof(cmd_buffer),
             "AT+QMTPUB=0,0,0,0,\"%s\"\r\n",
             topic);

    User_Clear_RxBuffer();

    /*
     * 这里必须清 LPUART1 环形缓冲。
     * 原因：发布 ACK 前要等待 BC28 返回 ">"，
     * 如果 LPUART1 里残留旧的 +QMTRECV / OK / +QMTPUB，
     * Wait_For_Response 的 256 字节缓冲会先被旧数据塞满，导致后面的 ">" 检测不到。
     *
     * 此处是 ACK 发布前，当前下行帧已经被 Process_MQTT_Downlink 复制到 line_buf 并处理完，
     * 清掉 LPUART1 旧残留是安全的。
     */
    LPUART_Flush();

    BC28_Debug("[BC28_PUB] CMD: ");
    BC28_Debug(cmd_buffer);

    User_UART_SendString(cmd_buffer);

    if (!Wait_For_Response(">", 5000))
    {
        BC28_Debug("[BC28_PUB] wait > FAILED, rx=");
        BC28_Debug((char *)ConnectRxBuffer);
        BC28_DebugLn("");
        return false;
    }

    BC28_DebugLn("[BC28_PUB] got >");

    HAL_Delay(200);

    /* 慢速逐字节发送，确保 BC28 能稳定接收 */
    for (uint16_t i = 0; i < strlen(payload); i++)
    {
        HAL_UART_Transmit(&hlpuart1, (uint8_t *)&payload[i], 1, 100);
        HAL_Delay(1);
    }

    HAL_Delay(100);

    uint8_t end_char = 0x1A;
    HAL_UART_Transmit(&hlpuart1, &end_char, 1, 100);

    /*
     * 给 BC28 一点时间把 +QMTPUB: 等内部返回吐完。
     * 否则服务器一收到 ACK 马上下发下一帧，URC 容易和发布返回挤在一起。
     */
    HAL_Delay(800);
    return true;
}

/**
 * @brief  OTA ACK 发布包装函数
 *         内部加 300ms 延时：从 QMTRECV 切回 AT 命令模式，BC28 需要恢复时间。
 */
bool BC28_Publish_OTA_Report(const char *json_payload)
{
    /*
     * 刚处理完 +QMTRECV，BC28 从 URC 输出状态切回 AT 命令输入需要一点时间。
     */
    HAL_Delay(500);
    return BC28_Publish_Raw("device/citrus_001/ota/up", json_payload);
}

// 软件复位
void BC28_SystemReset(void) {
    __disable_irq();
    NVIC_SystemReset();
}
