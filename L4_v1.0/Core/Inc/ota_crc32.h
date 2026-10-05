#ifndef OTA_CRC32_H
#define OTA_CRC32_H

#include <stdint.h>

uint32_t OTA_CRC32(const uint8_t *data, uint32_t len);
uint32_t OTA_CRC32_Flash(uint32_t address, uint32_t len);

#endif /* OTA_CRC32_H */
