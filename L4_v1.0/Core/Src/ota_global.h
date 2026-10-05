#ifndef OTA_GLOBAL_H
#define OTA_GLOBAL_H

#include "stm32l4xx_hal.h"

#ifndef FLASH_PAGE_SIZE
#define FLASH_PAGE_SIZE (0x800)
#endif

#define OTA_FLASH_BASE      (0x08000000)
#define OTA_FLASH_SIZE      (256 * 1024)
#define OTA_FLASH_END       (OTA_FLASH_BASE + OTA_FLASH_SIZE)

#define OTA_BOOT_SIZE       (32 * 1024)
#define OTA_APP_SIZE        (40 * 1024)
#define OTA_NEW_SIZE        (40 * 1024)
#define OTA_FLAG_SIZE       (4 * 1024)

#define OTA_BOOT_START      (OTA_FLASH_BASE)
#define OTA_APP_START       (OTA_BOOT_START + OTA_BOOT_SIZE)
#define OTA_NEW_START       (OTA_APP_START + OTA_APP_SIZE)
#define OTA_FLAG_START      (OTA_NEW_START + OTA_NEW_SIZE)

#define OTA_SRAM_START      (0x20000000)
#define OTA_SRAM_SIZE       (64 * 1024)
#define OTA_SRAM_END        (OTA_SRAM_START + OTA_SRAM_SIZE)

#if (OTA_FLAG_START + OTA_FLAG_SIZE) > OTA_FLASH_END
#error "OTA layout exceeds flash size"
#endif

#define OTA_FLAG_MAGIC      (0x4F544131)

typedef enum
{
	OTA_SLOT_NONE = 0,
	OTA_SLOT_APP = 1,
	OTA_SLOT_NEW = 2
} ota_slot_t;

typedef struct
{
	uint32_t magic;			// 标志区魔数，必须为 OTA_FLAG_MAGIC
	uint32_t active_slot;	// 当前运行的镜像槽位，OTA_SLOT_APP 或 OTA_SLOT_NEW
	uint32_t pending_slot;	// 待升级的镜像槽位，OTA_SLOT_APP、OTA_SLOT_NEW 或 OTA_SLOT_NONE
	uint32_t image_size;	// 待升级镜像的大小（字节），仅当 pending_slot 不为 OTA_SLOT_NONE 时有效
	uint32_t confirmed;		// NEW 镜像确认标志，1 表示已确认
	uint32_t boot_attempts; // NEW 镜像试运行次数
	uint32_t reserved[2];	// 预留字段，保持 32 字节对齐
} ota_flag_t;

#define OTA_FLAG_PTR ((const ota_flag_t *)(OTA_FLAG_START))			// 以只读方式访问标志区

#endif // OTA_GLOBAL_H
