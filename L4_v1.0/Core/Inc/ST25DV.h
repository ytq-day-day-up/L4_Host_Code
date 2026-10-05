#ifndef __ST25DV_H
#define __ST25DV_H

#include "stm32l4xx_hal.h"
#include <stdint.h>

/* ========== ST25DV I2C 设备地址 (8位格式) ========== */
#define ST25DV_ADDRESS    0xA6        // 用户内存区域写地址
#define ST25DV_SYS_ADDR   0xAE        // 系统配置区域写地址

/* ========== ST25DV 寄存器地址 ========== */
#define ST25DV_MB_CTRL_DYN    0x2006  // Mailbox控制动态寄存器
#define ST25DV_MB_EN_BIT      0x01    // MB使能位
#define ST25DV_GPO_CTRL       0x0000  // GPO1静态寄存器
#define ST25DV_GPO_CONFIG_RF  0x98    // GPO: RF场变化中断使能

/* ========== EEPROM 地址布局 (ST25DV04KC = 512字节) ========== */
// 0x0000 ~ 0x0003 : CC文件 (Capability Container, 4字节)
// 0x0004 ~       : TLV + NDEF 数据区
#define ST25DV_EEPROM_SIZE    512
#define ST25DV_CC_ADDR        0x0000
#define ST25DV_NDEF_ADDR      0x0004  // NDEF数据起始地址(CC文件之后)

/* ========== 底层 I2C 操作 ========== */
void ST25DV_Init(void);
void ST25DV_DisableFTM(void);
void ST25DV_ConfigGPO(void);
void ST25DV_WriteBytes(uint16_t MemAddress, uint8_t *Data, uint16_t Length);
void ST25DV_ReadBytes(uint16_t MemAddress, uint8_t *Data, uint16_t Length);

/* ========== NDEF 格式操作 ========== */
void ST25DV_WriteCC(void);
void ST25DV_WriteNDEF_Text(const char *utf8_text, uint16_t text_len);

/* ========== 传感器数据存储 (采集完成后调用) ========== */

/**
 * @brief  将全部8个传感器数据打包为一条NDEF Text Record写入EEPROM
 * @note   数据格式: "w:风速,at:气温,ah:湿度,l:光照,ap:气压,st:土温,sh:土湿,pH:酸碱度"
 *         手机NFC碰一碰即可读取全部数据
 * @param  wind_speed    风速 (m/s)
 * @param  air_temp      空气温度 (°C)
 * @param  air_humidity  空气湿度 (%)
 * @param  light_intensity 光照强度 (Lux)
 * @param  air_pressure  大气压强 (hPa)
 * @param  soil_temp     土壤温度 (°C)
 * @param  soil_humidity 土壤湿度 (%)
 * @param  soil_ph       土壤pH值
 */
void ST25DV_StoreAllSensorData(float wind_speed, float air_temp, float air_humidity,
                                uint16_t light_intensity, float air_pressure,
                                float soil_temp, float soil_humidity, float soil_ph);

/**
 * @brief  将单个浮点值以NDEF Text Record写入EEPROM (通用工具函数)
 * @param  label   数据标签 (如 "Wind", "Temp" 等)
 * @param  value   浮点数值
 * @param  decimals 小数位数
 */
void ST25DV_StoreSingleFloat(const char *label, float value, uint8_t decimals);

/**
 * @brief  将单个uint16值以NDEF Text Record写入EEPROM
 * @param  label  数据标签
 * @param  value  整数值
 */
void ST25DV_StoreSingleUint16(const char *label, uint16_t value);

#endif
