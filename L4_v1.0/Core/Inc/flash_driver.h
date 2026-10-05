#ifndef FLASH_DRIVER_H
#define FLASH_DRIVER_H

#include <stdint.h>
#include "stm32l4xx_hal.h"

HAL_StatusTypeDef FlashDriver_Erase(uint32_t start_address, uint32_t length);
HAL_StatusTypeDef FlashDriver_Write(uint32_t address, const uint8_t *data, uint32_t length);
void FlashDriver_Read(uint32_t address, uint8_t *data, uint32_t length);

#endif // FLASH_DRIVER_H
