#ifndef __MOTOR_H
#define __MOTOR_H
#include "main.h"

void Motor_Init(void);
void Motor_SetSpeed(uint8_t n,int16_t speed);

/* 梯度下降缓启缓停 */
void Motor_SetTarget(uint8_t n, int16_t target);
void Motor_SmoothUpdate(void);
void Motor_Stop(uint8_t n);
void Motor_StopAll(void);
int16_t Motor_GetCurrent(uint8_t n);

#endif

