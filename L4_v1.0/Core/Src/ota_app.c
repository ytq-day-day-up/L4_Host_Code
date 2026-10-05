#include "ota_app.h"
#include "ota_crc32.h"
#include "flash_driver.h"
#include <string.h>
#include <stdio.h>

/* ========== 外部函数声明（定义在 Connect.c / main.c） ========== */

extern bool BC28_Publish_OTA_Report(const char *json);
extern uint16_t Modbus_CRC16(uint8_t *ptr, uint16_t len);
extern void BC28_SystemReset(void);
extern UART_HandleTypeDef huart3;

/* ========== 调试日志 ========== */

#define OTA_DEBUG_ENABLE  0

extern UART_HandleTypeDef huart2;

#if OTA_DEBUG_ENABLE
static void OTA_DebugLog(const char *msg)
{
    if (msg == NULL) return;

    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 1000);

}

static void OTA_DebugLogHex(const char *prefix, uint32_t val)
{
    char buf[64];
    snprintf(buf, sizeof(buf), "%s: 0x%08X\r\n", prefix, (unsigned int)val);
    OTA_DebugLog(buf);
}
#else
#define OTA_DebugLog(msg)       ((void)0)
#define OTA_DebugLogHex(p, v)   ((void)0)
#endif

/* ========== 模块内部静态变量 ========== */

static ota_state_t ota_state = OTA_STATE_IDLE;

static uint16_t ota_task_no       = 0;
static uint32_t ota_total_size    = 0;
static uint16_t ota_total_frames  = 0;
static uint16_t ota_chunk_size    = 0;
static uint32_t ota_crc32_exp     = 0;

static uint16_t ota_frames_received = 0;
static uint32_t ota_bytes_written   = 0;

static uint32_t ota_frame_bitmap[OTA_BITMAP_WORDS];

static bool boot_is_trial = false;

static uint32_t ota_last_frame_tick = 0;
#define OTA_DOWNLOAD_TIMEOUT_MS  120000U

/* 接收侧滚动 CRC32：用于区分"数据接收错误"和"Flash写入错误" */
static uint32_t ota_rx_crc_state = 0xFFFFFFFFU;

/* DATA 帧协议头固定长度（不依赖 sizeof(结构体)，避免对齐问题） */
#define OTA_DATA_HEAD_LEN  8

/**
 * @brief  从大端字节流读取 uint16（固定偏移解析）
 */
static uint16_t OTA_ReadU16BE(const uint8_t *p)
{
    return ((uint16_t)p[0] << 8) | p[1];
}

/**
 * @brief  CRC32 逐步更新（标准多项式 0xEDB88320）
 */
static uint32_t OTA_CRC32_UpdateState(uint32_t crc, const uint8_t *data, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xEDB88320U;
            else
                crc >>= 1;
        }
    }
    return crc;
}

/* ========== 位图操作 ========== */

static void Bitmap_ClearAll(void)
{
    memset(ota_frame_bitmap, 0, sizeof(ota_frame_bitmap));
}

static int Bitmap_IsSet(uint16_t seq)
{
    if (seq >= OTA_MAX_FRAMES) return 0;
    return (ota_frame_bitmap[seq / 32] & (1U << (seq % 32))) != 0;
}

static void Bitmap_Set(uint16_t seq)
{
    if (seq < OTA_MAX_FRAMES)
        ota_frame_bitmap[seq / 32] |= (1U << (seq % 32));
}

static int Bitmap_CountSet(uint16_t total)
{
    int count = 0;
    uint16_t limit = total < OTA_MAX_FRAMES ? total : OTA_MAX_FRAMES;
    for (uint16_t i = 0; i < limit; i++)
    {
        if (Bitmap_IsSet(i)) count++;
    }
    return count;
}

/* ========== 标志区读写 ========== */

static ota_flag_t OTA_ReadFlag(void)
{
    ota_flag_t flag;
    FlashDriver_Read(OTA_FLAG_START, (uint8_t *)&flag, sizeof(flag));
    return flag;
}

static int OTA_WriteFlag(const ota_flag_t *flag)
{
    if (FlashDriver_Erase(OTA_FLAG_START, FLASH_PAGE_SIZE) != HAL_OK)
        return 0;
    if (FlashDriver_Write(OTA_FLAG_START, (const uint8_t *)flag, sizeof(*flag)) != HAL_OK)
        return 0;
    return 1;
}

/* ========== 资源清理 ========== */

static void OTA_Cleanup(void)
{
    Bitmap_ClearAll();
}

/* ========== 应答发布 ========== */
/*
 *   ota_start_ack → {"cmd":"ota_start_ack","task_no":X,"status":"ready"|"fail"}
 *   ota_data_ack  → {"cmd":"ota_data_ack","task_no":X,"seq":Y,"status":"ok"|"fail"}
 *   ota_end_ack   → {"cmd":"ota_end_ack","task_no":X,"status":"ok"|"fail"}
 */
static void OTA_SendAck(const char *cmd, uint16_t task_no,
                        int16_t seq, const char *status)
{
    char json[160];
    if (seq < 0)
    {
        snprintf(json, sizeof(json),
                 "{\"cmd\":\"%s\",\"task_no\":%u,\"status\":\"%s\"}",
                 cmd, (unsigned int)task_no, status);
    }
    else
    {
        snprintf(json, sizeof(json),
                 "{\"cmd\":\"%s\",\"task_no\":%u,\"seq\":%d,\"status\":\"%s\"}",
                 cmd, (unsigned int)task_no, (int)seq, status);
    }
    if (!BC28_Publish_OTA_Report(json))
    {
        OTA_DebugLog("[OTA] ACK publish failed!\r\n");
    }
    else
    {
        OTA_DebugLog("[OTA] ACK sent: ");
        OTA_DebugLog(json);
        OTA_DebugLog("\r\n");
    }
}

/* ========== 帧解析与 CRC16 校验 ========== */

/**
  * @brief  校验并提取 OTA 二进制帧（已由 MqttTask 完成 Hex→Bin 解码）。
  *
  *         帧格式（与服务器 build_ota_*_frame + append_crc_and_tail 一致）：
  *           [0xAA] [0x55] [CMD] [Data...] [CRC16_low] [CRC16_high] [0xDD]
  *
  *         服务器 CRC16 计算范围：crct16_modbus(frame_without_crc)
  *         frame_without_crc = bytes([0xAA, 0x55, CMD]) + Data
  *         → 即整帧去掉末尾 3 字节（CRC16(2) + tail(1)）。
  */
static int OTA_ParseFrame(const uint8_t *data, uint16_t len,
                          uint8_t *cmd, const uint8_t **body,
                          uint16_t *body_len)
{
    if (len < OTA_FRAME_MIN_LEN)
    {
        OTA_DebugLogHex("[OTA_PARSE] len too short", len);
        return 0;
    }

    if (data[0] != OTA_FRAME_HEADER1 || data[1] != OTA_FRAME_HEADER2)
    {
        OTA_DebugLogHex("[OTA_PARSE] bad head0", data[0]);
        OTA_DebugLogHex("[OTA_PARSE] bad head1", data[1]);
        return 0;
    }

    if (data[len - 1] != OTA_FRAME_FOOTER)
    {
        OTA_DebugLogHex("[OTA_PARSE] bad tail", data[len - 1]);
        OTA_DebugLogHex("[OTA_PARSE] len", len);
        return 0;
    }

    *cmd = data[2];
    *body_len = len - OTA_FRAME_MIN_LEN;  /* Data 部分长度 */

    if (*cmd != OTA_CMD_START && *cmd != OTA_CMD_DATA && *cmd != OTA_CMD_END)
    {
        OTA_DebugLogHex("[OTA_PARSE] unknown cmd", *cmd);
        return 0;
    }

    if (*cmd == OTA_CMD_START && *body_len < sizeof(ota_start_data_t))
    {
        OTA_DebugLogHex("[OTA_PARSE] start body too small", *body_len);
        return 0;
    }
    if (*cmd == OTA_CMD_DATA && *body_len < OTA_DATA_HEAD_LEN)
    {
        OTA_DebugLogHex("[OTA_PARSE] data body too small", *body_len);
        return 0;
    }
    if (*cmd == OTA_CMD_END && *body_len < sizeof(ota_end_data_t))
    {
        OTA_DebugLogHex("[OTA_PARSE] end body too small", *body_len);
        return 0;
    }

    /* CRC16 校验：覆盖整帧去掉末尾 3 字节（CRC16(2B) + tail(1B)） */
    {
        uint16_t crc_expected = data[len - 3] | ((uint16_t)data[len - 2] << 8);
        uint16_t crc_calc = Modbus_CRC16((uint8_t *)data, len - 3);

        if (crc_calc != crc_expected)
        {
            OTA_DebugLogHex("[OTA_PARSE] crc_expected", crc_expected);
            OTA_DebugLogHex("[OTA_PARSE] crc_calc", crc_calc);
            OTA_DebugLogHex("[OTA_PARSE] frame_len", len);
            return 0;
        }
    }

    *body = data + 3;  /* 指向 Data（跳过 0xAA 0x55 CMD） */
    return 1;
}

/* ========== 各帧处理 ========== */
/**
  * @brief 处理 CMD=0x01 启动帧。
  *        校验参数 → 擦除 NEW 槽位 → 应答服务器。
  *        参数为大端传输，需要字节序转换。
  */
static void OTA_HandleStartFrame(const ota_start_data_t *start_raw)
{
    /* ---- 字节序转换（服务器大端 → MCU 小端） ---- */
    uint16_t task_no       = ota_ntohs(start_raw->task_no);
    uint32_t total_size    = ota_ntohl(start_raw->total_size);
    uint16_t total_frames  = ota_ntohs(start_raw->total_frames);
    uint16_t chunk_size    = ota_ntohs(start_raw->chunk_size);
    uint32_t crc32_exp     = ota_ntohl(start_raw->crc32_expected);

    /* 正在下载且同 task → 服务器重发，直接 ACK */
    if (ota_state == OTA_STATE_DOWNLOADING && task_no == ota_task_no)
    {
        OTA_SendAck("ota_start_ack", task_no, -1, "ready");
        return;
    }

    OTA_Cleanup();

    /* ---- 参数校验 ---- */
    if (total_size == 0 || total_size > OTA_NEW_SIZE)
    {
        OTA_DebugLog("[OTA] invalid total_size\r\n");
        OTA_SendAck("ota_start_ack", task_no, -1, "fail");
        ota_state = OTA_STATE_FAILED;
        return;
    }

    if (total_frames == 0 || chunk_size == 0)
    {
        OTA_DebugLog("[OTA] invalid frame params\r\n");
        OTA_SendAck("ota_start_ack", task_no, -1, "fail");
        ota_state = OTA_STATE_FAILED;
        return;
    }

    if ((chunk_size % 8) != 0)
    {
        OTA_DebugLog("[OTA] chunk_size not 8-byte aligned\r\n");
        OTA_SendAck("ota_start_ack", task_no, -1, "fail");
        ota_state = OTA_STATE_FAILED;
        return;
    }

    if (chunk_size < OTA_MIN_CHUNK_SIZE)
    {
        OTA_DebugLog("[OTA] chunk_size too small\r\n");
        OTA_SendAck("ota_start_ack", task_no, -1, "fail");
        ota_state = OTA_STATE_FAILED;
        return;
    }

    /* ---- 交叉一致性校验 ---- */
    {
        uint32_t max_coverage = (uint32_t)total_frames * chunk_size;
        uint32_t min_coverage = (uint32_t)(total_frames - 1) * chunk_size;

        if (total_size > max_coverage)
        {
            OTA_DebugLog("[OTA] total_size > max_coverage\r\n");
            OTA_SendAck("ota_start_ack", task_no, -1, "fail");
            ota_state = OTA_STATE_FAILED;
            return;
        }

        if (total_size <= min_coverage && total_frames > 1)
        {
            OTA_DebugLog("[OTA] total_size too small for total_frames\r\n");
            OTA_SendAck("ota_start_ack", task_no, -1, "fail");
            ota_state = OTA_STATE_FAILED;
            return;
        }
    }

    if (total_frames > OTA_MAX_FRAMES)
    {
        OTA_DebugLog("[OTA] too many frames\r\n");
        OTA_SendAck("ota_start_ack", task_no, -1, "fail");
        ota_state = OTA_STATE_FAILED;
        return;
    }

    /* ---- 保存元数据 ---- */
    ota_task_no        = task_no;
    ota_total_size     = total_size;
    ota_total_frames   = total_frames;
    ota_chunk_size     = chunk_size;
    ota_crc32_exp      = crc32_exp;
    ota_frames_received = 0;
    ota_bytes_written   = 0;

    /* 初始化接收侧滚动 CRC32 */
    ota_rx_crc_state = 0xFFFFFFFFU;

    Bitmap_ClearAll();

    /* ---- 擦除 NEW 槽位 ---- */
    if (FlashDriver_Erase(OTA_NEW_START, OTA_NEW_SIZE) != HAL_OK)
    {
        OTA_DebugLog("[OTA] erase NEW slot failed!\r\n");
        OTA_SendAck("ota_start_ack", task_no, -1, "fail");
        ota_state = OTA_STATE_FAILED;
        return;
    }

    ota_state = OTA_STATE_DOWNLOADING;
    ota_last_frame_tick = HAL_GetTick();

    OTA_DebugLog("[OTA] NEW slot erased, ready\r\n");
    OTA_DebugLogHex("[OTA] total_size", ota_total_size);
    OTA_DebugLogHex("[OTA] total_frames", ota_total_frames);

    OTA_SendAck("ota_start_ack", ota_task_no, -1, "ready");
}

static int OTA_VerifyFlash(uint32_t address, const uint8_t *src, uint32_t len)
{
    const uint8_t *flash_ptr = (const uint8_t *)address;

    for (uint32_t i = 0; i < len; i++)
    {
        if (flash_ptr[i] != src[i])
        {
            OTA_DebugLogHex("[OTA] verify fail offset", i);
            OTA_DebugLogHex("[OTA] flash byte", flash_ptr[i]);
            OTA_DebugLogHex("[OTA] src byte", src[i]);
            return 0;
        }
    }

    return 1;
}

/**
  * @brief 处理 CMD=0x02 数据帧。
  *        使用固定字节偏移解析 DATA 帧头（8 字节），不依赖 sizeof(结构体)。
  */
static void OTA_HandleDataFrame(const ota_data_head_t *head_raw,
                                const uint8_t *body, uint16_t total_body_len)
{
    (void)head_raw;

    if (total_body_len < OTA_DATA_HEAD_LEN)
    {
        OTA_DebugLog("[OTA] data body too short\r\n");
        return;
    }

    /*
     * DATA帧协议头固定 8 字节：
     * task_no(2) + seq(2) + total_frames(2) + data_len(2)
     * 不再依赖 sizeof(ota_data_head_t)，避免结构体对齐导致偏移错误。
     */
    uint16_t task_no      = OTA_ReadU16BE(body + 0);
    uint16_t seq          = OTA_ReadU16BE(body + 2);
    uint16_t total_frames = OTA_ReadU16BE(body + 4);
    uint16_t data_len     = OTA_ReadU16BE(body + 6);

    const uint8_t *fw_data = body + OTA_DATA_HEAD_LEN;

    if (ota_state != OTA_STATE_DOWNLOADING)
    {
        OTA_DebugLog("[OTA] data ignored: not downloading\r\n");
        return;
    }

    if (task_no != ota_task_no)
    {
        OTA_DebugLogHex("[OTA] task_no mismatch", task_no);
        return;
    }

    if (total_frames != ota_total_frames)
    {
        OTA_DebugLogHex("[OTA] total_frames mismatch", total_frames);
        return;
    }

    if (seq >= ota_total_frames)
    {
        OTA_DebugLogHex("[OTA] seq out of bounds", seq);
        return;
    }

    if (data_len == 0 || data_len > ota_chunk_size)
    {
        OTA_DebugLogHex("[OTA] invalid data_len", data_len);
        return;
    }

    if (total_body_len != OTA_DATA_HEAD_LEN + data_len)
    {
        OTA_DebugLogHex("[OTA] total_body_len", total_body_len);
        OTA_DebugLogHex("[OTA] expect_body_len", OTA_DATA_HEAD_LEN + data_len);
        OTA_DebugLog("[OTA] body_len mismatch\r\n");
        return;
    }

    /* 重复帧：只应答，不重复写 Flash */
    if (Bitmap_IsSet(seq))
    {
        OTA_SendAck("ota_data_ack", ota_task_no, (int16_t)seq, "ok");
        ota_last_frame_tick = HAL_GetTick();
        return;
    }

    uint32_t write_addr = OTA_NEW_START + (uint32_t)seq * ota_chunk_size;

    if (write_addr + data_len > OTA_NEW_START + OTA_NEW_SIZE)
    {
        OTA_DebugLog("[OTA] write overflow\r\n");
        return;
    }

    if ((write_addr % 8) != 0)
    {
        OTA_DebugLog("[OTA] unaligned write addr\r\n");
        return;
    }

    OTA_DebugLogHex("[OTA] write seq", seq);
    OTA_DebugLogHex("[OTA] write addr", write_addr);
    OTA_DebugLogHex("[OTA] data_len", data_len);

    /*
     * 写 Flash 时先关闭中断，避免 Flash 编程期间被 UART/SysTick/其他中断打断。
     * 因为服务器是等 ACK 才发下一帧，所以这里短时间关中断是可以接受的。
     */
    __disable_irq();
    HAL_StatusTypeDef flash_status = FlashDriver_Write(write_addr, fw_data, data_len);
    __enable_irq();

    if (flash_status != HAL_OK)
    {
        OTA_DebugLog("[OTA] flash write failed\r\n");
        OTA_SendAck("ota_data_ack", ota_task_no, (int16_t)seq, "fail");
        ota_state = OTA_STATE_FAILED;
        OTA_Cleanup();
        return;
    }

    /* 写完立刻读回校验，不能等最后才发现 CRC32 错 */
    if (!OTA_VerifyFlash(write_addr, fw_data, data_len))
    {
        OTA_DebugLog("[OTA] flash verify failed\r\n");
        OTA_SendAck("ota_data_ack", ota_task_no, (int16_t)seq, "fail");
        ota_state = OTA_STATE_FAILED;
        OTA_Cleanup();
        return;
    }

    /* 接收侧滚动 CRC32 更新（在 Bitmap_Set 之前） */
    ota_rx_crc_state = OTA_CRC32_UpdateState(ota_rx_crc_state, fw_data, data_len);

    Bitmap_Set(seq);
    ota_frames_received++;
    ota_bytes_written += data_len;
    ota_last_frame_tick = HAL_GetTick();

    OTA_SendAck("ota_data_ack", ota_task_no, (int16_t)seq, "ok");
}

/**
  * @brief 处理 CMD=0x03 结束帧。
  *        帧完整性检查 → CRC32 校验 → 写 Flag 区 → 软件复位。
  */
static void OTA_HandleEndFrame(const ota_end_data_t *end_raw)
{
    uint16_t task_no      = ota_ntohs(end_raw->task_no);
    uint32_t file_size    = ota_ntohl(end_raw->file_size);
    uint16_t total_frames = ota_ntohs(end_raw->total_frames);
    uint32_t crc32_exp    = ota_ntohl(end_raw->crc32_expected);

    if (ota_state != OTA_STATE_DOWNLOADING) return;
    if (task_no != ota_task_no)             return;

    if (file_size != ota_total_size ||
        total_frames != ota_total_frames ||
        crc32_exp != ota_crc32_exp)
    {
        OTA_DebugLog("[OTA] end frame params mismatch\r\n");
        OTA_SendAck("ota_end_ack", ota_task_no, -1, "fail");
        ota_state = OTA_STATE_FAILED;
        OTA_Cleanup();
        return;
    }

    /* ---- 帧完整性检查 ---- */
    if (Bitmap_CountSet(ota_total_frames) < ota_total_frames)
    {
        OTA_DebugLog("[OTA] missing frames\r\n");
        OTA_SendAck("ota_end_ack", ota_task_no, -1, "fail");
        ota_state = OTA_STATE_FAILED;
        OTA_Cleanup();
        return;
    }

    /*
     * 字节数交叉校验（在 Flash CRC32 之前，快速排除统计异常）
     */
    OTA_DebugLogHex("[OTA] frames_received", ota_frames_received);
    OTA_DebugLogHex("[OTA] bytes_written", ota_bytes_written);
    OTA_DebugLogHex("[OTA] total_size", ota_total_size);

    if (ota_frames_received != ota_total_frames)
    {
        OTA_DebugLog("[OTA] frames_received mismatch\r\n");
        OTA_SendAck("ota_end_ack", ota_task_no, -1, "fail");
        ota_state = OTA_STATE_FAILED;
        OTA_Cleanup();
        return;
    }

    if (ota_bytes_written != ota_total_size)
    {
        OTA_DebugLog("[OTA] bytes_written mismatch\r\n");
        OTA_SendAck("ota_end_ack", ota_task_no, -1, "fail");
        ota_state = OTA_STATE_FAILED;
        OTA_Cleanup();
        return;
    }

    /*
     * 接收侧滚动 CRC32 校验：
     * 区分"接收数据本身错误" vs "Flash写入错误"
     * 如果 RX CRC ≠ 服务器 CRC，说明问题在 MqttTask/帧提取/fw_data偏移层；
     * 如果 RX CRC = 服务器 CRC 但 Flash CRC ≠，说明问题在 Flash 层。
     */
    {
        uint32_t rx_crc = ota_rx_crc_state ^ 0xFFFFFFFFU;
        OTA_DebugLogHex("[OTA] RX CRC32 computed", rx_crc);

        if (rx_crc != ota_crc32_exp)
        {
            OTA_DebugLog("[OTA] RX CRC32 MISMATCH, data before flash already wrong\r\n");
            OTA_SendAck("ota_end_ack", ota_task_no, -1, "fail");
            ota_state = OTA_STATE_FAILED;
            OTA_Cleanup();
            return;
        }
    }

    /* ---- CRC32 整体校验（Flash 层） ---- */
    ota_state = OTA_STATE_VERIFYING;
    uint32_t calc_crc = OTA_CRC32_Flash(OTA_NEW_START, ota_total_size);

    OTA_DebugLogHex("[OTA] CRC32 expected", ota_crc32_exp);
    OTA_DebugLogHex("[OTA] CRC32 computed", calc_crc);

    if (calc_crc != ota_crc32_exp)
    {
        OTA_DebugLog("[OTA] CRC32 MISMATCH\r\n");
        OTA_SendAck("ota_end_ack", ota_task_no, -1, "fail");
        ota_state = OTA_STATE_FAILED;
        OTA_Cleanup();
        return;
    }

    /* ---- 写标志区 ---- */
    ota_state = OTA_STATE_ACTIVATING;

    ota_flag_t flag = OTA_ReadFlag();
    if (flag.magic != OTA_FLAG_MAGIC)
    {
        memset(&flag, 0, sizeof(flag));
        flag.magic       = OTA_FLAG_MAGIC;
        flag.active_slot = OTA_SLOT_APP;
    }
    flag.pending_slot = OTA_SLOT_NEW;
    flag.image_size   = ota_total_size;

    if (!OTA_WriteFlag(&flag))
    {
        OTA_DebugLog("[OTA] FATAL: flag write failed\r\n");
        OTA_SendAck("ota_end_ack", ota_task_no, -1, "fail");
        ota_state = OTA_STATE_FAILED;
        OTA_Cleanup();
        return;
    }

    ota_state = OTA_STATE_SUCCESS;
    OTA_DebugLog("[OTA] SUCCESS, resetting...\r\n");

    OTA_SendAck("ota_end_ack", ota_task_no, -1, "ok");

    HAL_Delay(500);
    BC28_SystemReset();
}

/* ========== 公开 API ========== */
/**
  * @brief  处理二进制 OTA 帧（已由 MqttTask 完成 Hex→Bin 解码）。
  */
void OTA_HandleBinaryFrame(const uint8_t *data, uint16_t len)
{
    uint8_t cmd;
    const uint8_t *body;
    uint16_t body_len;

    if (!OTA_ParseFrame(data, len, &cmd, &body, &body_len))
        return;

    switch (cmd)
    {
    case OTA_CMD_START:
        if (body_len >= sizeof(ota_start_data_t))
            OTA_HandleStartFrame((const ota_start_data_t *)body);
        break;

    case OTA_CMD_DATA:
        if (body_len >= sizeof(ota_data_head_t))
            OTA_HandleDataFrame((const ota_data_head_t *)body, body, body_len);
        break;

    case OTA_CMD_END:
        if (body_len >= sizeof(ota_end_data_t))
            OTA_HandleEndFrame((const ota_end_data_t *)body);
        break;

    default:
        break;
    }
}

/**
  * @brief 启动时检查标志区，确定启动模式。
  *        active_slot=NEW && confirmed=0 → 试运行启动。
  */
void OTA_Init(void)
{
    ota_flag_t flag = OTA_ReadFlag();

    if (flag.magic != OTA_FLAG_MAGIC)
    {
        OTA_DebugLog("[OTA] Flag uninitialized, normal boot\r\n");
        boot_is_trial = false;
        ota_state = OTA_STATE_IDLE;
        return;
    }

    if (flag.active_slot == OTA_SLOT_NEW && flag.confirmed == 0)
    {
        OTA_DebugLog("[OTA] Trial boot: NEW slot, unconfirmed\r\n");
        boot_is_trial = true;
        ota_state = OTA_STATE_IDLE;
        return;
    }

    OTA_DebugLog("[OTA] Normal boot\r\n");
    boot_is_trial = false;
    ota_state = OTA_STATE_IDLE;
}

/**
  * @brief 确认 NEW 镜像启动成功。
  */
void OTA_ConfirmBoot(void)
{
    if (!boot_is_trial) return;

    ota_flag_t flag = OTA_ReadFlag();
    if (flag.magic != OTA_FLAG_MAGIC) return;

    flag.confirmed = 1;
    flag.boot_attempts = 0;

    if (OTA_WriteFlag(&flag))
    {
        boot_is_trial = false;
        OTA_DebugLog("[OTA] Boot confirmed\r\n");
    }
    else
    {
        OTA_DebugLog("[OTA] Failed to write confirmed\r\n");
    }
}

/**
  * @brief 主循环中调用，处理下载超时。
  */
void OTA_Process(void)
{
    if (ota_state == OTA_STATE_DOWNLOADING)
    {
        if ((HAL_GetTick() - ota_last_frame_tick) > OTA_DOWNLOAD_TIMEOUT_MS)
        {
            OTA_DebugLog("[OTA] Download timeout\r\n");
            ota_state = OTA_STATE_FAILED;
            OTA_Cleanup();
        }
    }
}

/**
  * @brief 获取当前 OTA 状态。
  */
ota_state_t OTA_GetState(void)
{
    return ota_state;
}

/**
  * @brief 强制重置 OTA 状态机。
  */
void OTA_ForceReset(void)
{
    OTA_DebugLog("[OTA] Force reset\r\n");
    ota_state = OTA_STATE_IDLE;
    OTA_Cleanup();
}
