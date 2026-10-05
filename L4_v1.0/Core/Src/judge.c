#include "Connect.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "judge.h"

// 存储生成的JSON字符串
static char warning_json_buffer[256];

/**
 * @brief 用世界真实场景规则，判断哪个数据出现异常，并打包为JSON
 * 
 * 正常范围参考自world.py：
 * - 风速 wind_speed: < 5.0
 * - 空气温度 air_temp: [15.0, 35.0]
 * - 空气湿度 air_humidity: [45.0, 85.0]
 * - 光照强度 light_intensity: [5000, 15000]
 * - 大气压 air_pressure: [1000.0, 1030.0]
 * - 土壤湿度 soil_humidity: [50.0, 88.0]
 * - 土壤pH soil_ph: [5.0, 7.0]
 * - 土壤温度异常: 高出空气温度 10 度以上
 */
char* Judge_And_Pack_Anomaly(float wind_speed, float air_temp, float air_humidity, 
                             uint16_t light_intensity, float air_pressure, 
                             float soil_temp, float soil_humidity, float soil_ph) 
{
    // 清空缓冲区
    memset(warning_json_buffer, 0, sizeof(warning_json_buffer));
    
    // 我们用字符串拼接拼出异常信息
    char reasons[128] = "";
    bool has_anomaly = false;

    // 1. 风速
    if (wind_speed > 5.0) {
        strcat(reasons, "\"wind\":\"high\",");
        has_anomaly = true;
    }
    // 2. 空气温度
    if (air_temp > 35.0) {
        strcat(reasons, "\"air_temp\":\"high\",");
        has_anomaly = true;
    } else if (air_temp < 15.0) {
        strcat(reasons, "\"air_temp\":\"low\",");
        has_anomaly = true;
    }
    // 3. 空气湿度
    if (air_humidity > 85.0) {
        strcat(reasons, "\"air_hum\":\"high\",");
        has_anomaly = true;
    } else if (air_humidity < 45.0) {
        strcat(reasons, "\"air_hum\":\"low\",");
        has_anomaly = true;
    }
    // 4. 光照强度
    if (light_intensity > 15000) {
        strcat(reasons, "\"lux\":\"high\",");
        has_anomaly = true;
    } else if (light_intensity < 5000) {
        strcat(reasons, "\"lux\":\"low\",");
        has_anomaly = true;
    }
    // 5. 大气压强
    if (air_pressure < 1000.0) {
        strcat(reasons, "\"pa\":\"low\",");
        has_anomaly = true;
    } else if (air_pressure > 1030.0) {
        strcat(reasons, "\"pa\":\"high\",");
        has_anomaly = true;
    }
    // 6. 土壤温度 (依据world.py，土壤异常时会反常地大幅高于空气温度)
    if (soil_temp > air_temp + 10.0) {
        strcat(reasons, "\"s_temp\":\"high\",");
        has_anomaly = true;
    } else if (soil_temp < air_temp - 10.0) {
        strcat(reasons, "\"s_temp\":\"low\",");
        has_anomaly = true;
    }
    // 7. 土壤湿度
    if (soil_humidity > 88.0) {
        strcat(reasons, "\"s_hum\":\"high\",");
        has_anomaly = true;
    } else if (soil_humidity < 50.0) {
        strcat(reasons, "\"s_hum\":\"low\",");
        has_anomaly = true;
    }
    // 8. 土壤pH
    if (soil_ph > 7.0) {
        strcat(reasons, "\"ph\":\"high\",");
        has_anomaly = true;
    } else if (soil_ph < 5.0) {
        strcat(reasons, "\"ph\":\"low\",");
        has_anomaly = true;
    }

    // 去掉最后一个逗号
    if (has_anomaly) {
        int len = strlen(reasons);
        if (reasons[len - 1] == ',') {
            reasons[len - 1] = '\0';
        }
        
        // 打包为最终JSON
        snprintf(warning_json_buffer, sizeof(warning_json_buffer), 
                 "{\"anomaly\":true,\"reasons\":{%s}}", reasons);
    } else {
        snprintf(warning_json_buffer, sizeof(warning_json_buffer), 
                 "{\"anomaly\":false,\"reasons\":{}}");
    }

    return warning_json_buffer;
}

