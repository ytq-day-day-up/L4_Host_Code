#ifndef OTA_APP_H
#define OTA_APP_H

#include <stdint.h>
#include <stdbool.h>
#include "ota_global.h"

/* ============================== OTA 帧协议常量 ============================== */

#define OTA_FRAME_HEADER1   0xAA
#define OTA_FRAME_HEADER2   0x55
#define OTA_FRAME_FOOTER    0xDD

#define OTA_CMD_START       0x01
#define OTA_CMD_DATA        0x02
#define OTA_CMD_END         0x03

/* 帧最小长度：header(2) + CMD(1) + CRC16(2) + footer(1) = 6 */
#define OTA_FRAME_MIN_LEN   6

/* ============================== 帧接收位图 ============================== */

/*
 * 最大数据帧数：1024 帧。
 * 若 total_frames > OTA_MAX_FRAMES，OTA_HandleStartFrame 将拒绝该任务。
 * 以 OTA_NEW_SIZE=40KB 计，1024 帧对应每帧 ~40 字节，
 * 远小于帧头开销，实际上 chunk_size 不会如此小。
 */
#define OTA_MAX_FRAMES      1024
#define OTA_BITMAP_WORDS    ((OTA_MAX_FRAMES + 31) / 32)  /* 32 个 uint32_t = 128 字节 */

/* 建议的最小 chunk_size：避免帧数爆炸导致位图溢出 */
#define OTA_MIN_CHUNK_SIZE  128

/* ============================== 网络字节序转主机字节序 ============================== */

/*
 * 服务器使用大端（网络字节序）编码所有多字节字段。
 * STM32L4 是小端，因此读取帧中所有 uint16_t/uint32_t 字段后必须字节交换。
 */
static inline uint16_t ota_ntohs(uint16_t net) {
    return (uint16_t)((net >> 8) & 0xFF) | ((net & 0xFF) << 8);
}

static inline uint32_t ota_ntohl(uint32_t net) {
    return ((net >> 24) & 0xFF)
         | ((net >> 8)  & 0xFF00)
         | ((net << 8)  & 0xFF0000)
         | ((net << 24) & 0xFF000000);
}

/* ============================== OTA 帧结构体 ============================== */

/* CMD=0x01 启动帧 Data 部分（大端传输，读取后用 ota_ntohs/ota_ntohl 转换） */
#pragma pack(push, 1)
typedef struct {
    uint16_t task_no;           /* 任务编号（大端） */
    uint32_t total_size;        /* 固件文件总大小（大端） */
    uint16_t total_frames;      /* 数据帧总数（大端） */
    uint16_t chunk_size;        /* 每数据帧载荷大小（大端） */
    uint32_t crc32_expected;    /* 整体固件 CRC32 期望值（大端） */
} ota_start_data_t;

/* CMD=0x02 数据帧 Data 部分（固定头 8 字节 + 可变长 data，大端传输） */
typedef struct {
    uint16_t task_no;           /* 任务编号（大端） */
    uint16_t seq;               /* 帧序号，从0开始（大端） */
    uint16_t total_frames;      /* 数据帧总数（大端） */
    uint16_t data_len;          /* 本帧固件数据长度（大端） */
    /* uint8_t data[data_len];  ---- 可变长，不在此结构体中 */
} ota_data_head_t;

/* CMD=0x03 结束帧 Data 部分（大端传输） */
typedef struct {
    uint16_t task_no;           /* 任务编号（大端） */
    uint32_t file_size;         /* 文件总大小（大端） */
    uint16_t total_frames;      /* 数据帧总数（大端） */
    uint32_t crc32_expected;    /* 整体 CRC32（大端） */
} ota_end_data_t;
#pragma pack(pop)

/* ============================== OTA 状态枚举 ============================== */

typedef enum {
    OTA_STATE_IDLE = 0,
    OTA_STATE_DOWNLOADING,
    OTA_STATE_VERIFYING,
    OTA_STATE_ACTIVATING,
    OTA_STATE_SUCCESS,
    OTA_STATE_FAILED
} ota_state_t;

/* ============================== API 函数 ============================== */

/**
  * @brief 启动时检查标志区，处理试运行启动确认/回滚逻辑。
  *         应在系统初始化、外设配置完成之后调用。
  */
void OTA_Init(void);

/**
  * @brief 主循环中调用，处理下载超时等后台任务。
  */
void OTA_Process(void);

/**
  * @brief 处理收到的 OTA 帧（二进制格式，已由 MqttTask 完成 Hex→Bin 解码）。
  * @param data 指向二进制帧数据
  * @param len  帧字节长度
  */
void OTA_HandleBinaryFrame(const uint8_t *data, uint16_t len);

/**
  * @brief 获取当前 OTA 下载状态。
  */
ota_state_t OTA_GetState(void);

/**
  * @brief 确认 NEW 镜像启动成功，将 confirmed=1 写入标志区。
  */
void OTA_ConfirmBoot(void);

/**
  * @brief 强制重置 OTA 状态机，清理所有中间状态。
  *         用于外部手动取消下载或从 FAILED 状态恢复。
  */
void OTA_ForceReset(void);

#endif /* OTA_APP_H */
