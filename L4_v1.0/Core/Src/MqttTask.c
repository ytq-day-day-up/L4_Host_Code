#include "MqttTask.h"
#include "Connect.h"
#include "ota_app.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "usart.h"
#include "gpio.h"
#include "Motor.h"
#include "encoder.h"

extern UART_HandleTypeDef huart2;

/**
 * @brief  调试输出（已禁用，UART2用于数据采集）
 */
static void MQTT_DBG(const char *s)
{
    (void)s;  // 空实现
}

/* Hex 解码输出 buffer，足够容纳 CHUNK_SIZE=200 的完整帧 (max ~250 bytes binary) */
#define OTA_BIN_BUF_SIZE  256
static uint8_t ota_bin_buf[OTA_BIN_BUF_SIZE];

/**
  * @brief  将大写 Hex 字符串解码为二进制字节。
  *        服务器使用 frame.hex().upper() 编码，因此输入为大写 Hex。
  * @param  hex  输入 Hex 字符串（仅含 0-9 A-F）
  * @param  len  hex 字符串长度（必须为偶数）
  * @param  out  输出二进制缓冲区
  * @param  out_size 输出缓冲区大小
  * @retval 解码后的字节数，0 表示失败。
  */
static uint16_t HexDecode(const char *hex, uint16_t len,
                          uint8_t *out, uint16_t out_size)
{
    if (len % 2 != 0)
        return 0;

    uint16_t bin_len = len / 2;
    if (bin_len > out_size)
        return 0;

    for (uint16_t i = 0; i < bin_len; i++)
    {
        char c_high = hex[i * 2];
        char c_low  = hex[i * 2 + 1];

        uint8_t high, low;

        if (c_high >= '0' && c_high <= '9')
            high = (uint8_t)(c_high - '0');
        else if (c_high >= 'A' && c_high <= 'F')
            high = (uint8_t)(c_high - 'A' + 10);
        else if (c_high >= 'a' && c_high <= 'f')
            high = (uint8_t)(c_high - 'a' + 10);
        else
            return 0;

        if (c_low >= '0' && c_low <= '9')
            low = (uint8_t)(c_low - '0');
        else if (c_low >= 'A' && c_low <= 'F')
            low = (uint8_t)(c_low - 'A' + 10);
        else if (c_low >= 'a' && c_low <= 'f')
            low = (uint8_t)(c_low - 'a' + 10);
        else
            return 0;

        out[i] = (uint8_t)((high << 4) | low);
    }

    return bin_len;
}

static int IsHexChar(char c)
{
    if (c >= '0' && c <= '9') return 1;
    if (c >= 'A' && c <= 'F') return 1;
    if (c >= 'a' && c <= 'f') return 1;
    return 0;
}

/**
 * @brief  将单个 Hex 字符转半字节（4bit），非法返回 0xFF
 */
static uint8_t HexNibble(char c)
{
    if (c >= '0' && c <= '9') return (uint8_t)(c - '0');
    if (c >= 'A' && c <= 'F') return (uint8_t)(c - 'A' + 10);
    if (c >= 'a' && c <= 'f') return (uint8_t)(c - 'a' + 10);
    return 0xFF;
}

/**
 * @brief  从 Hex 字符串指定位置读取一个字节，失败返回 0
 */
static int HexByteAt(const char *hex, uint16_t pos, uint8_t *out)
{
    uint8_t h = HexNibble(hex[pos]);
    uint8_t l = HexNibble(hex[pos + 1]);

    if (h == 0xFF || l == 0xFF)
        return 0;

    *out = (uint8_t)((h << 4) | l);
    return 1;
}

/**
 * @brief  按协议精确长度截帧解码（不再盲目吃所有 Hex）
 *
 *         帧格式：AA 55 CMD [Data...] CRC16_L CRC16_H DD
 *         最小帧：AA55 CMD CRC_L CRC_H DD = 6字节 = 12个Hex字符
 *
 *         START:  AA55 CMD + 14B body + CRC2 + DD = 20B = 40 Hex
 *         DATA:   AA55 CMD + 8B head + data_len + CRC2 + DD = 14+data_len B
 *         END:    AA55 CMD + 12B body + CRC2 + DD = 18B = 36 Hex
 */
static void OTA_HandleHexPayload(const char *hex_start, uint16_t max_len)
{
    /* 最小帧：6字节 = 12个Hex字符 */
    if (max_len < 12)
    {
        MQTT_DBG("[MQTT_RX] hex too short\r\n");
        return;
    }

    /* 帧头必须是 AA55 */
    if (hex_start[0] != 'A' || hex_start[1] != 'A' ||
        hex_start[2] != '5' || hex_start[3] != '5')
    {
        MQTT_DBG("[MQTT_RX] not AA55 start\r\n");
        return;
    }

    /* 解析 CMD */
    uint8_t cmd = 0;
    if (!HexByteAt(hex_start, 4, &cmd))
    {
        MQTT_DBG("[MQTT_RX] cmd parse fail\r\n");
        return;
    }

    uint16_t frame_bin_len = 0;

    if (cmd == 0x01)
    {
        /*
         * START:
         * AA55 CMD(1) + 14字节body + CRC2 + DD
         * 总长 = 3 + 14 + 2 + 1 = 20字节
         */
        frame_bin_len = 20;
    }
    else if (cmd == 0x02)
    {
        /*
         * DATA:
         * AA55 CMD(1) + 8字节头(task_no/seq/total/data_len) + data + CRC2 + DD
         * data_len 在二进制偏移 [9,10]，对应 Hex 偏移 [18,19]
         */
        if (max_len < 22)
        {
            MQTT_DBG("[MQTT_RX] data header hex too short\r\n");
            return;
        }

        uint8_t dl_h = 0, dl_l = 0;
        if (!HexByteAt(hex_start, 18, &dl_h) ||
            !HexByteAt(hex_start, 20, &dl_l))
        {
            MQTT_DBG("[MQTT_RX] data_len parse fail\r\n");
            return;
        }

        uint16_t data_len = ((uint16_t)dl_h << 8) | dl_l;

        if (data_len == 0 || data_len > 200)
        {
            MQTT_DBG("[MQTT_RX] invalid data_len in hex\r\n");
            return;
        }

        frame_bin_len = (uint16_t)(14 + data_len);
    }
    else if (cmd == 0x03)
    {
        /*
         * END:
         * AA55 CMD(1) + 12字节body + CRC2 + DD
         * 总长 = 3 + 12 + 2 + 1 = 18字节
         */
        frame_bin_len = 18;
    }
    else
    {
        MQTT_DBG("[MQTT_RX] unknown cmd\r\n");
        return;
    }

    /* 检查数据是否完整 */
    uint16_t frame_hex_len = frame_bin_len * 2;

    if (max_len < frame_hex_len)
    {
        MQTT_DBG("[MQTT_RX] incomplete hex frame\r\n");
        return;
    }

    /* 检查目标长度内必须全是合法 Hex 字符 */
    for (uint16_t i = 0; i < frame_hex_len; i++)
    {
        if (!IsHexChar(hex_start[i]))
        {
            MQTT_DBG("[MQTT_RX] non-hex in frame\r\n");
            return;
        }
    }

    /* 精确长度解码 */
    uint16_t bin_len = HexDecode(
        hex_start,
        frame_hex_len,
        ota_bin_buf,
        OTA_BIN_BUF_SIZE
    );

    if (bin_len != frame_bin_len)
    {
        MQTT_DBG("[MQTT_RX] bin_len mismatch\r\n");
        return;
    }

    MQTT_DBG("[MQTT_RX] CALL OTA_HandleBinaryFrame\r\n");
    OTA_HandleBinaryFrame(ota_bin_buf, bin_len);
}


/* 前置声明（定义在下方，但 Process_MQTT_Downlink 中已调用） */
static void Process_OTA_Payload(char *after_topic, char *line_end);
static void Process_CMD_Payload(char *after_topic, char *line_end);

void Process_MQTT_Downlink(uint8_t *rx_buffer, uint16_t len)
{
//  MQTT_DBG("[MQTT_RX] ENTER\r\n");

    if (len == 0) return;

    const char *prefix = "+QMTRECV:";
    char *line = (char *)rx_buffer;
    char *line_end = line + len;

    char *urc_start = strstr(line, prefix);
    if (urc_start == NULL)
    {
        return;
    }

    /*
     * 最高优先级：直接找 AA55。
     * 不要先卡死在 topic 和 length 解析上。
     * 真正合法性由 OTA_HandleBinaryFrame() 内部的帧头、帧尾、CRC16 再判断。
     */
    char *aa55 = strstr(line, "AA55");
    if (aa55 != NULL)
    {
//      MQTT_DBG("[MQTT_RX] AA55 found, direct decode\r\n");
        OTA_HandleHexPayload(aa55, (uint16_t)(line_end - aa55));
        return;
    }

    /*
     * 如果这一行没有 AA55，再走原来的标准 topic/length 解析。
     */
    char *ptr = urc_start + strlen(prefix);

    /* 提取 topic */
    char *t0 = strchr(ptr, '"');
    if (t0 == NULL) {
        MQTT_DBG("[MQTT_RX] no topic quote\r\n");
        return;
    }

    char *t1 = strchr(t0 + 1, '"');
    if (t1 == NULL) {
        MQTT_DBG("[MQTT_RX] topic end quote missing\r\n");
        return;
    }

    uint16_t topic_len = (uint16_t)(t1 - t0 - 1);
    if (topic_len == 0) {
        MQTT_DBG("[MQTT_RX] topic empty\r\n");
        return;
    }

    char topic[64];
    uint16_t copy_len = topic_len < sizeof(topic) - 1 ? topic_len : sizeof(topic) - 1;
    memcpy(topic, t0 + 1, copy_len);
    topic[copy_len] = '\0';

    /* topic 之后的起始位置（payload 解析起点） */
    char *after_topic = t1 + 1;

    if (strcmp(topic, OTA_DOWN_TOPIC) == 0)
    {
        /* OTA 主题处理 - 继续走原有的 hex payload 解析 */
        Process_OTA_Payload(after_topic, line_end);
        return;
    }

    /* cmd/led 主题处理 - JSON 命令解析 */
    if (strcmp(topic, SUB_TOPIC) == 0)
    {
        Process_CMD_Payload(after_topic, line_end);
        return;
    }

    MQTT_DBG("[MQTT_RX] unknown topic: ");
    MQTT_DBG(topic);
    MQTT_DBG("\r\n");
}

/**
 * @brief  处理 OTA 主题的 payload（从 topic 后面开始解析）
 */
static void Process_OTA_Payload(char *after_topic, char *line_end)
{
    /*
     * 兼容格式1：
     * +QMTRECV: 0,128,"device/citrus_001/ota/down","AA5501..."
     */
    char *q0 = strchr(after_topic, '"');
    if (q0 != NULL)
    {
        char *q1 = strchr(q0 + 1, '"');
        if (q1 != NULL && q1 > q0 + 1)
        {
            uint16_t hex_len = (uint16_t)(q1 - q0 - 1);
            OTA_HandleHexPayload(q0 + 1, hex_len);
            return;
        }
    }

    /*
     * 兼容格式2：BC28 常见格式
     * +QMTRECV: 0,128,"device/citrus_001/ota/down",40,AA5501...
     *
     * 注意：这里的 40 是 MQTT payload 的文本长度，
     * 也就是 AA5501... 这一串 Hex 字符串的长度。
     */
    char *p = after_topic;

    while (p < line_end && (*p == ' ' || *p == ','))
    {
        p++;
    }

    char *endptr = NULL;
    uint32_t payload_text_len = strtoul(p, &endptr, 10);

    if (endptr != p && payload_text_len > 0)
    {
        char *payload_start = endptr;

        while (payload_start < line_end &&
               (*payload_start == ' ' ||
                *payload_start == ',' ||
                *payload_start == '\r' ||
                *payload_start == '\n'))
        {
            payload_start++;
        }

        if (payload_start < line_end && *payload_start == '"')
        {
            payload_start++;
        }

        if (payload_start < line_end)
        {
            uint32_t available = (uint32_t)(line_end - payload_start);
            uint16_t max_hex_len;

            if (payload_text_len < available)
            {
                max_hex_len = (uint16_t)payload_text_len;
            }
            else
            {
                max_hex_len = (uint16_t)available;
            }

            OTA_HandleHexPayload(payload_start, max_hex_len);
            return;
        }
    }

    MQTT_DBG("[MQTT_RX] no hex payload found\r\n");
}

/* ==================== cmd/led JSON 命令处理 ==================== */

/** 电机移动固定速度 */
#define MOVE_SPEED   500

/**
 * @brief  从 JSON 字符串中提取键值（简单解析，不支持嵌套）
 * @param  json  JSON 字符串
 * @param  key   键名
 * @param  out_val  输出值缓冲区
 * @param  out_size  输出缓冲区大小
 * @retval 1=找到, 0=未找到
 */
static int Json_GetValue(const char *json, const char *key, char *out_val, uint16_t out_size)
{
    char search[32];
    int len = snprintf(search, sizeof(search), "\"%s\"", key);
    if (len <= 0) return 0;

    const char *pos = strstr(json, search);
    if (pos == NULL) return 0;

    pos += len;  // 跳过 "key"
    while (*pos == ' ' || *pos == ':') pos++;  // 跳过空格和冒号

    /* 提取值（到逗号、}或引号结束） */
    uint16_t i = 0;
    if (*pos == '"') pos++;  // 跳过字符串值的起始引号
    while (*pos && *pos != ',' && *pos != '}' && *pos != '"' && *pos != '\r' && *pos != '\n')
    {
        if (i < out_size - 1)
            out_val[i++] = *pos;
        pos++;
    }
    out_val[i] = '\0';
    return 1;
}

/**
 * @brief  电机按指定圈数移动（阻塞式，使用编码器反馈）
 * @param  motor_id  电机编号(1或2)
 * @param  circles  目标圈数（正数）
 */
static void Motor_MoveCircles(uint8_t motor_id, float circles)
{
    if (circles <= 0) return;

    Encoder_ClearPosition();

    /* 启动电机正向旋转 */
    Motor_SetSpeed(motor_id, MOVE_SPEED);

    /* 循环检测编码器圈数 */
    uint32_t timeout_tick = HAL_GetTick() + 30000;  // 30秒超时
    while (HAL_GetTick() < timeout_tick)
    {
        Encoder_UpdatePosition();

        float current_circles = (motor_id == 1) ? Encoder_GetCircle1() : Encoder_GetCircle2();

        if (current_circles >= circles)
        {
            break;
        }
        HAL_Delay(10);
    }

    /* 到达目标或超时，停止电机 */
    Motor_SetSpeed(motor_id, 0);
}

/**
 * @brief  处理 cmd/led 主题的 JSON 命令
 */
static void Process_CMD_Payload(char *after_topic, char *line_end)
{
    /* 提取 JSON payload（在双引号内） */
    char *q0 = strchr(after_topic, '"');
    if (q0 == NULL) return;
    char *q1 = strchr(q0 + 1, '"');
    if (q1 == NULL) return;

    uint16_t json_len = (uint16_t)(q1 - q0 - 1);
    if (json_len == 0 || json_len > 256) return;

    char json_buf[257];
    memcpy(json_buf, q0 + 1, json_len);
    json_buf[json_len] = '\0';

    /* 解析 cmd 字段 */
    char cmd_val[16];
    if (!Json_GetValue(json_buf, "cmd", cmd_val, sizeof(cmd_val))) return;

    if (strcmp(cmd_val, "pump") == 0)
    {
        /* 水泵控制 {"cmd": "pump", "val": 1/0} */
        char val_str[8];
        if (Json_GetValue(json_buf, "val", val_str, sizeof(val_str)))
        {
            int val = atoi(val_str);
            if (val == 1)
            {
                HAL_GPIO_WritePin(Water_beng_GPIO_Port, Water_beng_Pin, GPIO_PIN_SET);  // PH3 高电平 = 开泵
            }
            else if (val == 0)
            {
                HAL_GPIO_WritePin(Water_beng_GPIO_Port, Water_beng_Pin, GPIO_PIN_RESET);  // PH3 低电平 = 关泵
            }
        }
    }
    else if (strcmp(cmd_val, "move") == 0)
    {
        /* XY 平移控制 {"cmd": "move", "x": 2, "y": 3} */
        char x_str[8], y_str[8];
        int x = 0, y = 0;

        if (Json_GetValue(json_buf, "x", x_str, sizeof(x_str)))
            x = atoi(x_str);
        if (Json_GetValue(json_buf, "y", y_str, sizeof(y_str)))
            y = atoi(y_str);

        /* 限制范围 0-5 */
        if (x < 0) x = 0; if (x > 5) x = 5;
        if (y < 0) y = 0; if (y > 5) y = 5;

        /* 计算目标圈数：每单位 = 5 圈 */
        float target_circles_x = (float)(x * 5);
        float target_circles_y = (float)(y * 5);

        /* 依次移动两个电机（先X后Y） */
        if (target_circles_x > 0)
        {
            Motor_MoveCircles(1, target_circles_x);
            HAL_Delay(200);  // 间隔
        }

        if (target_circles_y > 0)
        {
            Motor_MoveCircles(2, target_circles_y);
        }
    }
}
