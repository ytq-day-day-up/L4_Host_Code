#ifndef __ENCODER_H
#define __ENCODER_H

#include "main.h"

#define ENCODER_PPR             11.0f
#define ENCODER_GEAR_RATIO      35.5f


#define ENCODER_ONE_CIRCLE_PULSE   (ENCODER_PPR * ENCODER_GEAR_RATIO)

void Encoder_Init(void);

int16_t Encoder_Get1(void);
int16_t Encoder_Get2(void);

void Encoder_ClearPosition(void);
void Encoder_UpdatePosition(void);

float Encoder_GetCircle1(void);
float Encoder_GetCircle2(void);
float Encoder_GetAverageCircle(void);

#endif

