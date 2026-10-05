/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "rtc.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include "Connect.h"
#include "MqttTask.h"
#include "yolo.h"
#include "NanoEdgeAI.h"
#include "judge.h"
#include "ST25DV.h"
//#include "stop.h"
#include "encoder.h"
#include "Motor.h"
#include "ota_app.h"
// YOLO 外部变量
extern uint8_t rx3_buffer[];
extern volatile uint16_t rx3_head;
extern volatile uint16_t rx3_tail;
#define RX3_BUFFER_SIZE 64
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t lpuart_buf = 0;   // 定义LPUART接收缓冲
// NanoEdgeAI 变量
enum neai_state neai_state;        // NEAI 函数返回状
bool use_pretrained = true;        // true=使用预训练模形, false=从头学习
uint8_t similarity;                // 相似度百分比 (0-100)
float input_signal[NEAI_INPUT_SIGNAL_LENGTH * NEAI_INPUT_AXIS_NUMBER];  // 输入信号缓冲 (8个float)

// 接收完整数据帧的缓冲
uint8_t tx_buffer[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x84, 0x0A}; // 风速
uint8_t tx_buffer2[] = {0x02, 0x03, 0x00, 0x00, 0x00, 0x04, 0x44, 0x3A}; // 空气温湿度
uint8_t tx_buffer3[] = {0x03, 0x03, 0x00, 0x00, 0x00, 0x03, 0x04, 0x29}; // 土壤传感度

float wind_speed = 0.0f;           // 风速

float air_temp = 0.0f;             // 空气温度
float air_humidity = 0.0f;         // 空气湿度
uint16_t light_intensity = 0;      // 光照强度
float air_pressure = 0.0f;		   //大气压强

float soil_temp = 0.0f;            // 土壤温度
float soil_humidity = 0.0f;        // 土壤湿度
float soil_ph = 0.0f;              // 土壤pH

uint32_t last_send_time = 0; // 上次发送数据时间记

#define LPUART_BUFFER_SIZE 4096
uint8_t lpuart_buffer[LPUART_BUFFER_SIZE];   // LPUART1 环形缓冲区 (BC28, 需容纳 Hex OTA 帧 ~500B, 加大防溢出)
volatile uint16_t lpuart_head = 0;        // 读指针
volatile uint16_t lpuart_tail = 0;        // 写指针

// LPUART1 错误统计（中断回调里只计数，不打印/不重操作）
volatile uint32_t lpuart_error_count = 0;
volatile uint32_t lpuart_last_error = 0;
volatile uint8_t  lpuart_error_flag = 0;

// 改进的消息队列实�?
#define RX_BUFFER_SIZE 256
uint8_t rx_buffer[RX_BUFFER_SIZE];     // 环形缓冲
volatile uint16_t rx_head = 0;         // 缓冲区读指针
volatile uint16_t rx_tail = 0;         // 缓冲区写指针

// MQTT连接状态
volatile uint8_t mqtt_connected = 0;   // MQTT连接状态

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint8_t LPUART_ReadByte(uint8_t *byte);
void Process_LPUART_Idle(void);
void ProcessReceivedData(void);
int FindModbusFrame(uint8_t* frame_len);
void LPUART_Flush(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// ===== 轻量级数字转字符串辅助函数（替代 sprintf，避免栈溢出�? =====

void my_itoa(int32_t val, char *buf) {
    char tmp[12];
    int i = 0, neg = 0;
    if (val < 0) { neg = 1; val = -val; }
    if (val == 0) { buf[0] = '0'; buf[1] = '\0'; return; }
    while (val > 0) { tmp[i++] = '0' + (val % 10); val /= 10; }
    int pos = 0;
    if (neg) buf[pos++] = '-';
    while (--i >= 0) buf[pos++] = tmp[i];
    buf[pos] = '\0';
}

void my_u16toa(uint16_t val, char *buf) {
    char tmp[6];
    int i = 0;
    if (val == 0) { buf[0] = '0'; buf[1] = '\0'; return; }
    while (val > 0) { tmp[i++] = '0' + (val % 10); val /= 10; }
    int pos = 0;
    while (--i >= 0) buf[pos++] = tmp[i];
    buf[pos] = '\0';
}

void my_ftoa(float val, char *buf, int decimals) {
    if (val < 0) { *buf++ = '-'; val = -val; }
    int32_t ipart = (int32_t)val;
    float fpart = val - (float)ipart;
    // 整数部分
    char tmp[12]; int i = 0;
    if (ipart == 0) tmp[i++] = '0';
    else { int32_t v = ipart; while (v > 0) { tmp[i++] = '0' + (v % 10); v /= 10; } }
    int pos = 0;
    while (--i >= 0) buf[pos++] = tmp[i];
    // 小数部分
    if (decimals > 0) {
        buf[pos++] = '.';
        for (int d = 0; d < decimals; d++) { fpart *= 10; int dig = (int)fpart; buf[pos++] = '0' + dig; fpart -= dig; }
    }
    buf[pos] = '\0';
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_I2C2_Init();
  MX_TIM16_Init();
  MX_LPUART1_UART_Init();
  MX_RTC_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM15_Init();
  /* USER CODE BEGIN 2 */

  // 清除 BootLoader 跳转遗留的 PRIMASK，恢复全局中断（否则 SysTick/USART/DMA 全部不工作）
  __enable_irq();

  // 增加初始延时让外设模块稳定
  HAL_Delay(1000);

  // OTA 启动检查
  OTA_Init();

  // 启动串口接收中断（UART2用于数据采集，LPUART1用于BC28）
  HAL_UART_Receive_IT(&huart2, &rx_buffer[rx_head], 1);
  HAL_UART_Receive_IT(&hlpuart1, &lpuart_buffer[lpuart_head], 1);
  
//串口三打印处（1）

  // 等待BC28模块就绪并连接服务器
  while(!BC28_Check_Status()) {
      HAL_Delay(1000);
  }
  HAL_Delay(1000);
  
//串口三打印处（2）
  
  mqtt_connected = BC28_Connect_Server() ? 1 : 0;
  HAL_Delay(1000);

  // 确认OTA启动（如果需要）
  OTA_ConfirmBoot();

//  串口三打印处（3）
  
  // 初始化各外设模块
  YOLO_Init();
  Motor_Init();
  Encoder_Init();

  last_send_time = HAL_GetTick(); // 初始化时间记录

  // 拉高PC6，导通高侧开关
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
  // 确认PH3继电器关闭
  HAL_GPIO_WritePin(Water_beng_GPIO_Port, Water_beng_Pin, GPIO_PIN_RESET);

  // 初始化NanoEdgeAI
  neai_state = neai_anomalydetection_init(use_pretrained);

  // 初始化ST25DV NFC标签
  ST25DV_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // 持续处理UART2接收的Modbus传感器数据
    ProcessReceivedData();

    // 处理LPUART1接收（MQTT下行指令）
    Process_LPUART_Idle();

    // 处理UART3接收（YOLO视觉数据）
    YOLO_Process();

    // OTA 后台处理
    OTA_Process();

    /* LPUART1 错误处理（每秒最多一次，仅清除标志） */
    static uint32_t lpuart_err_print_tick = 0;
    if (lpuart_error_flag && HAL_GetTick() - lpuart_err_print_tick >= 1000)
    {
        lpuart_err_print_tick = HAL_GetTick();
        lpuart_error_flag = 0;
    }

    // ===== 每30秒执行一次：传感器采集 + MQTT上报 + AI异常检测 =====
    if (HAL_GetTick() - last_send_time > 10000)
    {
        // OTA 进行中（下载/校验/激活）时暂停数据上报，避免 Flash 冲突和 UART 争用
        ota_state_t ota_state = OTA_GetState();
        if (ota_state != OTA_STATE_DOWNLOADING &&
            ota_state != OTA_STATE_VERIFYING &&
            ota_state != OTA_STATE_ACTIVATING)
        {
        if (mqtt_connected)
        {
            // --- 步骤A: 询问风速传感器 (Modbus地址0x01) ---
            HAL_UART_Transmit(&huart2, tx_buffer, sizeof(tx_buffer), HAL_MAX_DELAY);
            HAL_Delay(200);
            ProcessReceivedData();           // 解析风速回复
            BC28_Publish_Data(1);            // MQTT上报风速
            HAL_Delay(500);

            // --- 步骤B: 询问空气传感器 (Modbus地址0x02) ---
            HAL_UART_Transmit(&huart2, tx_buffer2, sizeof(tx_buffer2), HAL_MAX_DELAY);
            HAL_Delay(200);
            ProcessReceivedData();           // 解析空气传感器回复
            BC28_Publish_Data(2);            // MQTT上报空气数据
            HAL_Delay(500);

            // --- 步骤C: 询问土壤传感器 (Modbus地址0x03) ---
            HAL_UART_Transmit(&huart2, tx_buffer3, sizeof(tx_buffer3), HAL_MAX_DELAY);
            HAL_Delay(200);
            ProcessReceivedData();           // 解析土壤传感器回复
            BC28_Publish_Data(3);            // MQTT上报土壤数据

            // 采集完毕，将最新数据写入NFC供用户随时读取
            ST25DV_StoreAllSensorData(wind_speed, air_temp, air_humidity,
                                      light_intensity, air_pressure,
                                      soil_temp, soil_humidity, soil_ph);

            // === NanoEdge AI 异常检测 ===
            input_signal[0] = wind_speed;
            input_signal[1] = air_temp;
            input_signal[2] = air_humidity;
            input_signal[3] = (float)light_intensity;
            input_signal[4] = air_pressure;
            input_signal[5] = soil_temp;
            input_signal[6] = soil_humidity;
            input_signal[7] = soil_ph;

            neai_state = neai_anomalydetection_detect(input_signal, &similarity);

            if (neai_state == NEAI_OK)
            {
                if (similarity < 80)
                {
                    char* warning_json = Judge_And_Pack_Anomaly(wind_speed, air_temp, air_humidity,
                                                                light_intensity, air_pressure,
                                                                soil_temp, soil_humidity, soil_ph);
                    BC28_Publish_Anomaly(warning_json);
                }
            }
        }

        } // OTA 状态检查结束

        last_send_time = HAL_GetTick();    // 重置计时器
    }

    /* 主循环保持高频轮询 UART */
    HAL_Delay(10);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/* USER CODE BEGIN 4 */
/**
 * @brief LPUART环形缓冲区读取一个字节
 */
uint8_t LPUART_ReadByte(uint8_t *byte) {
    if (lpuart_head != lpuart_tail) {
        *byte = lpuart_buffer[lpuart_tail];
        lpuart_tail = (lpuart_tail + 1) & (LPUART_BUFFER_SIZE - 1);
        return 1;
    }
    return 0;
}

/**
 * @brief  清空 LPUART1 环形缓冲区旧数据
 *         用于 AT 命令发送前，避免 Wait_For_Response 被
 *         旧的 +QMTRECV / OK / +QMTPUB 干扰。
 *         注意：不用 HAL_UART_AbortReceive()，只做指针归位。
 */
void LPUART_Flush(void)
{
    __disable_irq();
    lpuart_tail = lpuart_head;
    __enable_irq();
}

/**
 * @brief 主循环中处理 LPUART1 接收到的异步数据 (如下发指令)
 */
void Process_LPUART_Idle(void) {
    static char line_buf[1536];
    static int line_idx = 0;
    uint8_t data;

    while (LPUART_ReadByte(&data)) {
        if (line_idx < sizeof(line_buf) - 1) {
            line_buf[line_idx++] = data;
            line_buf[line_idx] = '\0';
        } else {
            /* 溢出保护：清空缓冲区，丢弃过长行 */
            line_idx = 0;
            memset(line_buf, 0, sizeof(line_buf));
            continue;
        }
        // 遇到换行符，说明接收到完整的数据
        if (data == '\n') {
            // 如果这一行包含 MQTT 下发前缀，则交给 MqttTask 处理
            if (strstr(line_buf, "+QMTRECV:")) {
                Process_MQTT_Downlink((uint8_t*)line_buf, line_idx);
            }
            /* 处理完毕，清空行缓存 */
            line_idx = 0;
            memset(line_buf, 0, sizeof(line_buf));
        }
    }
}


uint16_t Modbus_CRC16(uint8_t *ptr, uint16_t len) {
    uint16_t crc = 0xFFFF;
    while (len--) {
        crc ^= *ptr++;
        for (int i = 0; i < 8; i++) {
            if (crc & 0x01) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/**
 * @brief  查找MODBUS RTU�?
 * @retval -1:未找到完整帧, 其他:帧起始位�?
 */
int FindModbusFrame(uint8_t* frame_len) //frame_len 是指向帧长度的指�?
{
  uint16_t len = (rx_head - rx_tail) & (RX_BUFFER_SIZE - 1);
  if(len < 7) return -1; // 至少7字节
  
  // 查找帧头
  uint16_t pos = rx_tail;
  uint16_t checked = 0;
  
  while(checked < (len - 6)) {
	uint8_t addr = rx_buffer[pos];
    uint8_t func = rx_buffer[(pos + 1) & (RX_BUFFER_SIZE - 1)];
    // 地址码，功能�?
    if((addr >= 0x01 && addr <= 0x03) && func == 0x03){
      // 获取数据长度
      uint8_t data_len = rx_buffer[(pos + 2) & (RX_BUFFER_SIZE - 1)];
      uint16_t frame_length = 3 + data_len + 2; // 地址(1) + 功能�?(1) + 数据长度(1) + 数据(N) + CRC(2)
      
      if(frame_length <= len) {
        *frame_len = frame_length;
        return pos;
      }
    }
    pos = (pos + 1) & (RX_BUFFER_SIZE - 1);
    checked++;
  }
  
  return -1;
}

/**
 * @brief  处理接收到的传感器数�?
 * @retval None
 */
void ProcessReceivedData(void)
{
    static uint8_t processing_buffer[32]; // 临时处理缓冲
    uint8_t frame_len;
    int frame_start;

    // 持续查找并处理完整帧
    while ((frame_start = FindModbusFrame(&frame_len)) >= 0)
    {
        // 提取完整帧到临时缓冲
        for (int i = 0; i < frame_len; i++)
        {
            processing_buffer[i] = rx_buffer[(frame_start + i) & (RX_BUFFER_SIZE - 1)];
        }

        //CRC校验:对帧数据部分(去掉�?�?2字节CRC本身)计算CRC，与帧尾CRC对比
        uint16_t calc_crc = Modbus_CRC16(processing_buffer, frame_len - 2);
        uint16_t recv_crc = (processing_buffer[frame_len - 1] << 8) | processing_buffer[frame_len - 2]; // Modbus CRC低字节在�?
        if (calc_crc != recv_crc)
        {
            // CRC校验失败，丢弃当前字节继续查�????
            rx_tail = (frame_start + 1) & (RX_BUFFER_SIZE - 1);
            continue;
        }

        // 根据地址判断是哪个传感器
        uint8_t slave_addr = processing_buffer[0];
        uint8_t func_code = processing_buffer[1];

        if (func_code == 0x03)
        {
            uint8_t data_len = processing_buffer[2]; // 数据字节

            if (data_len == 0x02 && slave_addr == 0x01)
            {
                // 解析风力数据(地址0x01)
                uint16_t wind_raw = (processing_buffer[3] << 8) | processing_buffer[4];
                wind_speed = wind_raw / 10.0f;

			}
			else if (data_len == 0x08 && slave_addr == 0x02)
            {
                // 解析空气温湿度光照大气压(地址0x02)
                
                // 1. 温度 (字节 3,4)
                int16_t temp_raw = (processing_buffer[3] << 8) | processing_buffer[4];
                air_temp = temp_raw / 10.0f;

                // 2. 湿度 (字节 5,6)
                uint16_t hum_raw = (processing_buffer[5] << 8) | processing_buffer[6];
                air_humidity = hum_raw / 10.0f;

                // 3. 光照 (字节 7,8)
                // 根据手册 1234 -> 12340 Lux，所以要乘以 10
                uint32_t light_raw = (processing_buffer[7] << 8) | processing_buffer[8];
                light_intensity = light_raw * 10; 

                // 4. 大气压强 (字节 9,10)
                uint16_t press_raw = (processing_buffer[9] << 8) | processing_buffer[10];
                air_pressure = press_raw / 10.0f; // 9462 -> 946.2 hPa

			}
            else if (data_len == 0x06 && slave_addr == 0x03)
            {
                // 解析土壤传感器数�?(地址0x03)
                // 土壤温度（补码）
                int16_t soil_temp_raw = (processing_buffer[3] << 8) | processing_buffer[4];
                soil_temp = soil_temp_raw / 10.0f;

                // 土壤湿度
                uint16_t soil_hum_raw = (processing_buffer[5] << 8) | processing_buffer[6];
                soil_humidity = soil_hum_raw / 10.0f;

                //  pH
                uint16_t ph_raw = (processing_buffer[7] << 8) | processing_buffer[8];
                soil_ph = ph_raw / 100.0f; // 因为例如 6.86 对应 686 / 100

            }
        }

        // 移动缓冲区指针，跳过已处理的�?
        rx_tail = (frame_start + frame_len) & (RX_BUFFER_SIZE - 1);
    }
}

/**
 * @brief  Rx Transfer completed callback
 * @param  huart: UART handle
 * @retval None
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2) 
  {
    uint16_t next_head = (rx_head + 1) & (RX_BUFFER_SIZE - 1);
    if(next_head != rx_tail) rx_head = next_head;
    HAL_UART_Receive_IT(&huart2, &rx_buffer[rx_head], 1);
  }
  else if (huart->Instance == LPUART1) //  LPUART1中断处理(BC28)
  {
    uint16_t next_head = (lpuart_head + 1) & (LPUART_BUFFER_SIZE - 1);
    if(next_head != lpuart_tail) lpuart_head = next_head;
    HAL_UART_Receive_IT(&hlpuart1, &lpuart_buffer[lpuart_head], 1);
  }
  else if (huart->Instance == USART3) // UART3中断处理 (树莓派YOLO数据)
  {
    uint16_t next_head = (rx3_head + 1) & (RX3_BUFFER_SIZE - 1);
    if(next_head != rx3_tail) rx3_head = next_head;
    HAL_UART_Receive_IT(&huart3, &rx3_buffer[rx3_head], 1);
  }
}

/**
 * @brief  UART 错误回调（仅计数+恢复，禁止打印/重操作）
 *         Flash 编程期间 CPU/中断时序易受影响，LPUART1 可能触发 ORE。
 *         此回调只清错误标志、重置状态机、重启接收中断。
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == LPUART1)
    {
        lpuart_error_count++;
        lpuart_last_error = huart->ErrorCode;
        lpuart_error_flag = 1;

        /*
         * 清 UART 错误标志。
         * 重点是 ORE，过载时最常见。
         */
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_PEFLAG(huart);

        /*
         * 不要在这里打印，不要做耗时操作。
         * 只尝试重新打开 1 字节中断接收。
         */
        huart->ErrorCode = HAL_UART_ERROR_NONE;
        huart->RxState = HAL_UART_STATE_READY;

        HAL_UART_Receive_IT(&hlpuart1, &lpuart_buffer[lpuart_head], 1);
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
