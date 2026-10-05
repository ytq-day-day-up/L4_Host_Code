#ifndef __JUDGE_H
#define __JUDGE_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief 判断哪些传感器数据异常并返回JSON字符串
 * @param wind_speed 风速
 * @param air_temp 空气温度
 * @param air_humidity 空气湿度
 * @param light_intensity 光照强度
 * @param air_pressure 大气压
 * @param soil_temp 土壤温度
 * @param soil_humidity 土壤湿度
 * @param soil_ph 土壤pH
 * @return JSON字符串指针
 */
char* Judge_And_Pack_Anomaly(float wind_speed, float air_temp, float air_humidity, 
                             uint16_t light_intensity, float air_pressure, 
                             float soil_temp, float soil_humidity, float soil_ph);

#endif
