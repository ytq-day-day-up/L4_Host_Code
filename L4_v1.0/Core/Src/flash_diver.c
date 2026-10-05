#include "flash_driver.h"
#include "ota_global.h"
#include <string.h>

static uint32_t FlashDriver_GetPage(uint32_t address)
{
	/* 地址转换为页索引。 */
	return (address - FLASH_BASE) / FLASH_PAGE_SIZE;
}

static int FlashDriver_IsRangeValid(uint32_t address, uint32_t length)
{
	if (length == 0)
	{
		return 0;
	}

	uint32_t end_address = address + length - 1;
	uint32_t allowed_end = OTA_FLAG_START + OTA_FLAG_SIZE - 1;

	/* 避免溢出并限制在 App/New/Flag 区间内。 */
	if ((end_address < address) || (address < OTA_APP_START))
	{
		return 0;
	}

	if (end_address > allowed_end)
	{
		return 0;
	}

	return 1;
}

HAL_StatusTypeDef FlashDriver_Erase(uint32_t start_address, uint32_t length)
{
	/* 无需擦除时直接返回。 */
	if (length == 0)
	{
		return HAL_OK;
	}

	if (!FlashDriver_IsRangeValid(start_address, length))
	{
		return HAL_ERROR;
	}

	/* 计算需要擦除的页范围。 */
	uint32_t first_page = FlashDriver_GetPage(start_address);
	uint32_t last_address = start_address + length - 1;
	uint32_t last_page = FlashDriver_GetPage(last_address);
	uint32_t page_error = 0;

	FLASH_EraseInitTypeDef erase = {0};
	erase.TypeErase = FLASH_TYPEERASE_PAGES;
	erase.Banks = FLASH_BANK_1;
	erase.Page = first_page;
	erase.NbPages = (last_page - first_page) + 1;

	/* 擦除前解锁 Flash。 */
	if (HAL_FLASH_Unlock() != HAL_OK)
	{
		return HAL_ERROR;
	}

	HAL_StatusTypeDef status = HAL_OK;
	for (uint32_t page = first_page; page <= last_page; ++page)
	{
		erase.Page = page;
		erase.NbPages = 1;
		status = HAL_FLASHEx_Erase(&erase, &page_error);
		if (status != HAL_OK)
		{
			break;
		}
	}

	HAL_FLASH_Lock();
	return status;
}

HAL_StatusTypeDef FlashDriver_Write(uint32_t address, const uint8_t *data, uint32_t length)
{
	/* 双字编程要求 8 字节对齐。 */
	if ((address % 8) != 0)
	{
		return HAL_ERROR;
	}

	if (length == 0)
	{
		return HAL_OK;
	}

	if (!FlashDriver_IsRangeValid(address, length))
	{
		return HAL_ERROR;
	}

	/* 写入前解锁 Flash。 */
	if (HAL_FLASH_Unlock() != HAL_OK)
	{
		return HAL_ERROR;
	}

	HAL_StatusTypeDef status = HAL_OK;
	for (uint32_t offset = 0; offset < length; offset += 8)
	{
		uint8_t buffer[8];
		uint64_t value = 0;
		uint32_t copy_len = (length - offset) >= 8 ? 8 : (length - offset);

		/* 最后不足 8 字节用 0xFF 填充。 */
		memset(buffer, 0xFF, sizeof(buffer));
		memcpy(buffer, data + offset, copy_len);
		memcpy(&value, buffer, sizeof(value));

		status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address + offset, value);
		if (status != HAL_OK)
		{
			break;
		}
	}

	HAL_FLASH_Lock();
	return status;
}

void FlashDriver_Read(uint32_t address, uint8_t *data, uint32_t length)
{
	/*
	 * Flash 读取无需解锁 —— 内部 Flash 通过 I-Code/D-Code 总线直接
	 * 内存映射访问。解锁/加锁仅对编程和擦除操作是必需的。
	 * 移除不必要的 unlock/lock 可减少开销，并避免在 OTA 下载期间
	 * （大量调用 FlashDriver_Write 加锁）因读取操作意外解锁而引入竞态。
	 */
	memcpy(data, (const void *)address, length);
}
