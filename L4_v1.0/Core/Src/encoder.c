#include "encoder.h"
#include "tim.h"

static int32_t encoder1_total_pulse = 0;
static int32_t encoder2_total_pulse = 0;

void Encoder_Init(void)
{
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(&htim1, 0);

    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(&htim2, 0);

    Encoder_ClearPosition();
}

int16_t Encoder_Get1(void)
{
    int16_t pul = (int16_t)__HAL_TIM_GET_COUNTER(&htim1);
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    return pul;
}

int16_t Encoder_Get2(void)
{
    int16_t pul = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    return pul;
}

void Encoder_ClearPosition(void)
{
    encoder1_total_pulse = 0;
    encoder2_total_pulse = 0;

    __HAL_TIM_SET_COUNTER(&htim1, 0);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
}

void Encoder_UpdatePosition(void)
{
    encoder1_total_pulse += Encoder_Get1();
    encoder2_total_pulse += Encoder_Get2();
}

float Encoder_GetCircle1(void)
{
    return encoder1_total_pulse / ENCODER_ONE_CIRCLE_PULSE;
}

float Encoder_GetCircle2(void)
{
    return encoder2_total_pulse / ENCODER_ONE_CIRCLE_PULSE;
}

float Encoder_GetAverageCircle(void)
{
    float c1 = Encoder_GetCircle1();
    float c2 = Encoder_GetCircle2();

    return (c1 + c2) / 2.0f;
}

