#include "stop.h"

extern uint8_t lpuart_buf;

extern void SystemClock_Config(void);
extern void MX_LPUART1_UART_Init(void);

void Enter_Stop2(void)
{
    __HAL_RCC_LPUART1_CLK_DISABLE();  // 如果你不用 LPUART 唤醒，就关掉
    // 如果你还有其他外设（ADC/TIM/SPI），也在这里关掉

    // 1. 挂起系统滴答
    HAL_SuspendTick();

    // 2. 进入 STOP2 模式
    HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);

    // 3. 唤醒后重配系统时钟
    SystemClock_Config();

    MX_LPUART1_UART_Init();              // 重新初始化 LPUART
    HAL_UART_Receive_IT(&hlpuart1, &lpuart_buf, 1);  // 重新开启接收

    // 4. 恢复系统滴答
    HAL_ResumeTick();
}

HAL_StatusTypeDef RTC_Set_Alarm(uint32_t sec)
{
    RTC_AlarmTypeDef sAlarm = {0};
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};

    if (sec == 0)
    {
        sec = 1;
    }

    if (HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
    {
        return HAL_ERROR;
    }

    uint32_t totalSeconds = sTime.Hours * 3600
                          + sTime.Minutes * 60
                          + sTime.Seconds
                          + sec;

    totalSeconds %= 24 * 3600;

    sAlarm.AlarmTime.Hours = totalSeconds / 3600;
    sAlarm.AlarmTime.Minutes = (totalSeconds % 3600) / 60;
    sAlarm.AlarmTime.Seconds = totalSeconds % 60;
    sAlarm.AlarmTime.SubSeconds = 0;

    sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;

    sAlarm.Alarm = RTC_ALARM_A;
    sAlarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY;
    sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;

    sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
    sAlarm.AlarmDateWeekDay = 1;

    if (HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}
