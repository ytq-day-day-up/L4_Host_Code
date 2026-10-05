#include "ota_crc32.h"

/**
 * @brief 标准 CRC32 / IEEE 802.3 / zlib 同款
 *        初始值 0xFFFFFFFF，最终异或 0xFFFFFFFF
 *        多项式反射形式 0xEDB88320
 *
 *        使用纯位运算，不依赖查表，避免 CRC 表抄错导致校验失败。
 *        与 ota_app.c 中接收侧滚动 CRC32 算法完全一致。
 */
static uint32_t CRC32_Update(uint32_t crc, const uint8_t *data, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++)
    {
        crc ^= data[i];

        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 1U)
            {
                crc = (crc >> 1) ^ 0xEDB88320U;
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    return crc;
}

uint32_t OTA_CRC32(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFU;
    crc = CRC32_Update(crc, data, len);
    return crc ^ 0xFFFFFFFFU;
}

uint32_t OTA_CRC32_Flash(uint32_t address, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFU;
    const uint8_t *ptr = (const uint8_t *)address;

    crc = CRC32_Update(crc, ptr, len);

    return crc ^ 0xFFFFFFFFU;
}
