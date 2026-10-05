#include "stm32l4xx_hal.h"
#include <string.h>
#include "i2c.h"
#include "ST25DV.h"

extern I2C_HandleTypeDef hi2c2;

/* ========== 数字转字符串辅助函数 (与 Connect.c 中一致) ========== */
static void st25_itoa(int32_t val, char *buf) {
    char tmp[12];
    int i = 0, neg = 0;
    if (val < 0) { neg = 1; val = -val; }
    if (val == 0) { buf[0] = '0'; buf[1] = '\0'; return; }
    while (val > 0) { tmp[i++] = '0' + (val % 10); val /= 10; }
    int pos = 0;
    if (neg) buf[pos++] = '-';
    while (--i >= 0) buf[pos++] = tmp[i];
    buf[pos] = '\0';
}

static void st25_u16toa(uint16_t val, char *buf) {
    char tmp[6];
    int i = 0;
    if (val == 0) { buf[0] = '0'; buf[1] = '\0'; return; }
    while (val > 0) { tmp[i++] = '0' + (val % 10); val /= 10; }
    int pos = 0;
    while (--i >= 0) buf[pos++] = tmp[i];
    buf[pos] = '\0';
}

static void st25_ftoa(float val, char *buf, int decimals) {
    if (val < 0) { *buf++ = '-'; val = -val; }
    int32_t ipart = (int32_t)val;
    float fpart = val - (float)ipart;
    char tmp[12]; int i = 0;
    if (ipart == 0) tmp[i++] = '0';
    else { int32_t v = ipart; while (v > 0) { tmp[i++] = '0' + (v % 10); v /= 10; } }
    int pos = 0;
    while (--i >= 0) buf[pos++] = tmp[i];
    if (decimals > 0) {
        buf[pos++] = '.';
        for (int d = 0; d < decimals; d++) { fpart *= 10; int dig = (int)fpart; buf[pos++] = '0' + dig; fpart -= dig; }
    }
    buf[pos] = '\0';
}

/* ========== NDEF 内部缓冲区 ========== */
#define NDEF_MAX_SIZE  200
static uint8_t ST25DV_NDEF_BUF[NDEF_MAX_SIZE];

/* CC文件 (NFC Forum Type 2 Tag, 4字节) */
static const uint8_t ST25DV_CC_FILE[4] = {
    0xE1,   // Magic Number
    0x10,   // Version 1.0, Read/Write
    0x3F,   // Memory Size: (0x3F+1) x 8 = 512 bytes (ST25DV04KC满容量)
    0x00    // Read/Write Access
};

/* ============================================================ */
/*                    底层 I2C 操作                              */
/* ============================================================ */

/**
  * ST25DV初始化: 禁用FTM + 配置GPO
  */
void ST25DV_Init(void)
{
    ST25DV_DisableFTM();
    ST25DV_ConfigGPO();
}

/**
  * 禁用快速传输模式(FTM) - 写入EEPROM前必须调用
  */
void ST25DV_DisableFTM(void)
{
    uint8_t txData[3];
    uint16_t regAddr = ST25DV_MB_CTRL_DYN;

    txData[0] = (uint8_t)(regAddr >> 8);
    txData[1] = (uint8_t)(regAddr & 0xFF);
    txData[2] = 0x00;

    HAL_I2C_Master_Transmit(&hi2c2, ST25DV_ADDRESS, txData, 3, 100);
    HAL_Delay(5);
}

/**
  * 配置GPO引脚(RF场变化中断)
  */
void ST25DV_ConfigGPO(void)
{
    uint8_t txData[3];
    uint16_t regAddr = ST25DV_GPO_CTRL;

    txData[0] = (uint8_t)(regAddr >> 8);
    txData[1] = (uint8_t)(regAddr & 0xFF);
    txData[2] = ST25DV_GPO_CONFIG_RF;

    HAL_I2C_Master_Transmit(&hi2c2, ST25DV_SYS_ADDR, txData, 3, 100);
    HAL_Delay(1);
}

/**
  * 写入多字节数据到EEPROM
  */
void ST25DV_WriteBytes(uint16_t MemAddress, uint8_t *Data, uint16_t Length)
{
    HAL_I2C_Mem_Write(&hi2c2, ST25DV_ADDRESS, MemAddress,
                      I2C_MEMADD_SIZE_16BIT, Data, Length, 1000);
    HAL_Delay(10);
}

/**
  * 从EEPROM读取多字节数据
  */
void ST25DV_ReadBytes(uint16_t MemAddress, uint8_t *Data, uint16_t Length)
{
    HAL_I2C_Mem_Read(&hi2c2, ST25DV_ADDRESS, MemAddress,
                     I2C_MEMADD_SIZE_16BIT, Data, Length, 1000);
}

/* ============================================================ */
/*                    NDEF 格式操作                               */
/* ============================================================ */

/**
  * 写入CC文件到EEPROM地址0x0000
  */
void ST25DV_WriteCC(void)
{
    ST25DV_WriteBytes(ST25DV_CC_ADDR, (uint8_t*)ST25DV_CC_FILE, 4);
}

/**
  * 构建NDEF Text Record并写入EEPROM
  * @param utf8_text  UTF-8编码的文本数据
  * @param text_len   文本字节长度
  */
void ST25DV_WriteNDEF_Text(const char *utf8_text, uint16_t text_len)
{
    uint16_t idx = 0;

    /* --- 构建 NDEF Record --- */
    // Header: MB=1,ME=1,CF=0,SR=1,IL=0,TNF=0x01(Well Known)
    ST25DV_NDEF_BUF[idx++] = 0xD1;
    // Type Length = 1 ('T')
    ST25DV_NDEF_BUF[idx++] = 0x01;
    // Payload Length = Status(1) + Lang(2) + Text(N)
    uint8_t payload_len = 1 + 2 + (uint8_t)text_len;
    ST25DV_NDEF_BUF[idx++] = payload_len;
    // Type = 'T'
    ST25DV_NDEF_BUF[idx++] = 0x54;
    // Status byte: bit7=0(UTF-8), 语言代码长度=2
    ST25DV_NDEF_BUF[idx++] = 0x02;
    // 语言代码 "zh"
    ST25DV_NDEF_BUF[idx++] = 'z';
    ST25DV_NDEF_BUF[idx++] = 'h';
    // 文本数据
    memcpy(&ST25DV_NDEF_BUF[idx], utf8_text, text_len);
    idx += text_len;

    uint16_t ndef_len = idx;

    /* --- 用 TLV 封装 NDEF 消息 --- */
    uint8_t tlv_buf[NDEF_MAX_SIZE + 4];
    uint16_t tlv_idx = 0;

    tlv_buf[tlv_idx++] = 0x03;             // TLV Tag: NDEF Message
    tlv_buf[tlv_idx++] = (uint8_t)ndef_len; // TLV Length
    memcpy(&tlv_buf[tlv_idx], ST25DV_NDEF_BUF, ndef_len);
    tlv_idx += ndef_len;
    tlv_buf[tlv_idx++] = 0xFE;             // TLV Terminator

    /* --- 先写CC文件, 再写NDEF数据 --- */
    ST25DV_WriteCC();
    ST25DV_WriteBytes(ST25DV_NDEF_ADDR, tlv_buf, tlv_idx);
}



/**
  * 将全部8个传感器数据打包为一条NDEF Text Record写入EEPROM
  * 每行一条数据:
  *   风速:2.5
  *   气温:25.3
  *   湿度:65.2
  *   光照:12345
  *   气压:1013.2
  *   土温:22.1
  *   土湿:75.3
  *   酸碱度:6.50
  */
void ST25DV_StoreAllSensorData(float wind_speed, float air_temp, float air_humidity,
                                uint16_t light_intensity, float air_pressure,
                                float soil_temp, float soil_humidity, float soil_ph)
{
    char text_buf[160];
    uint16_t pos = 0;

    // 风速:xx.x\n
    memcpy(&text_buf[pos], "\xE9\xA3\x8E\xE9\x80\x9F", 6); pos += 6;  // "风速"
    text_buf[pos++] = ':';
    st25_ftoa(wind_speed, &text_buf[pos], 1);
    pos += strlen(&text_buf[pos]);
    text_buf[pos++] = ' ';

    // 气温:xx.x\n
    memcpy(&text_buf[pos], "\xE6\xB0\x94\xE6\xB8\xA9", 6); pos += 6;  // "气温"
    text_buf[pos++] = ':';
    st25_ftoa(air_temp, &text_buf[pos], 1);
    pos += strlen(&text_buf[pos]);
    text_buf[pos++] = ' ';

    // 湿度:xx.x\n
    memcpy(&text_buf[pos], "\xE6\xB9\xBF\xE5\xBA\xA6", 6); pos += 6;  // "湿度"
    text_buf[pos++] = ':';
    st25_ftoa(air_humidity, &text_buf[pos], 1);
    pos += strlen(&text_buf[pos]);
    text_buf[pos++] = ' ';

    // 光照:xxxxx\n
    memcpy(&text_buf[pos], "\xE5\x85\x89\xE7\x85\xA7", 6); pos += 6;  // "光照"
    text_buf[pos++] = ':';
    st25_u16toa(light_intensity, &text_buf[pos]);
    pos += strlen(&text_buf[pos]);
    text_buf[pos++] = ' ';

    // 气压:xxxx.x\n
    memcpy(&text_buf[pos], "\xE6\xB0\x94\xE5\x8E\x8B", 6); pos += 6;  // "气压"
    text_buf[pos++] = ':';
    st25_ftoa(air_pressure, &text_buf[pos], 1);
    pos += strlen(&text_buf[pos]);
    text_buf[pos++] = ' ';

    // 土温:xx.x\n
    memcpy(&text_buf[pos], "\xE5\x9C\x9F\xE6\xB8\xA9", 6); pos += 6;  // "土温"
    text_buf[pos++] = ':';
    st25_ftoa(soil_temp, &text_buf[pos], 1);
    pos += strlen(&text_buf[pos]);
    text_buf[pos++] = ' ';

    // 土湿:xx.x\n
    memcpy(&text_buf[pos], "\xE5\x9C\x9F\xE6\xB9\xBF", 6); pos += 6;  // "土湿"
    text_buf[pos++] = ':';
    st25_ftoa(soil_humidity, &text_buf[pos], 1);
    pos += strlen(&text_buf[pos]);
    text_buf[pos++] = ' ';

    // 酸碱度:xx.xx\n
    memcpy(&text_buf[pos], "\xE9\x85\xB8\xE7\xA2\xB1\xE5\xBA\xA6", 9); pos += 9;  // "酸碱度"
    text_buf[pos++] = ':';
    st25_ftoa(soil_ph, &text_buf[pos], 2);
    pos += strlen(&text_buf[pos]);

    text_buf[pos] = '\0';

    // 写入NDEF Text Record
    ST25DV_WriteNDEF_Text(text_buf, pos);
}

/**
  * 将单个浮点值以NDEF Text Record写入EEPROM
  * 格式: "Label:123.45"
  */
void ST25DV_StoreSingleFloat(const char *label, float value, uint8_t decimals)
{
    char text_buf[64];
    uint16_t pos = 0;
    uint16_t label_len = strlen(label);

    memcpy(text_buf, label, label_len);
    pos += label_len;
    text_buf[pos++] = ':';
    st25_ftoa(value, &text_buf[pos], decimals);
    pos += strlen(&text_buf[pos]);
    text_buf[pos] = '\0';

    ST25DV_WriteNDEF_Text(text_buf, pos);
}

/**
  * 将单个uint16值以NDEF Text Record写入EEPROM
  * 格式: "Label:12345"
  */
void ST25DV_StoreSingleUint16(const char *label, uint16_t value)
{
    char text_buf[64];
    uint16_t pos = 0;
    uint16_t label_len = strlen(label);

    memcpy(text_buf, label, label_len);
    pos += label_len;
    text_buf[pos++] = ':';
    st25_u16toa(value, &text_buf[pos]);
    pos += strlen(&text_buf[pos]);
    text_buf[pos] = '\0';

    ST25DV_WriteNDEF_Text(text_buf, pos);
}
