#include "tim.h"
#include "motor.h"
#include "gpio.h"

//梯度下降缓启缓停参赛 
#define MOTOR_PWM_MAX 999
#define MOTOR_ALPHA   0.03f
#define MOTOR_DEADBAND 1.0f

static float motor1_target  = 0.0f;
static float motor1_current = 0.0f;

static float motor2_target  = 0.0f;
static float motor2_current = 0.0f;


static float Motor_LimitFloat(float value)
{
    if(value > MOTOR_PWM_MAX) value = MOTOR_PWM_MAX;
    if(value < -MOTOR_PWM_MAX) value = -MOTOR_PWM_MAX;
    return value;
}

static int16_t Motor_LimitInt(int16_t value)
{
    if(value > MOTOR_PWM_MAX) value = MOTOR_PWM_MAX;
    if(value < -MOTOR_PWM_MAX) value = -MOTOR_PWM_MAX;
    return value;
}

//梯度下降式更新
static float Motor_GradientUpdate(float current, float target)
{
    float error = target - current;// 计算误差：还差多少到目标
		// 误差很小，直接等于目标，不再微动
    if(error > -MOTOR_DEADBAND && error < MOTOR_DEADBAND)
    {	
				//梯度下降核心公式
        current = target;
    }
    else
    {
        current = current + MOTOR_ALPHA * error;
    }

    return Motor_LimitFloat(current);
}



void Motor_Init(void){
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_SET);
	//启动PWM
	HAL_TIM_PWM_Start(&htim15,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim15,TIM_CHANNEL_2);
	
}

void Motor_SetSpeed(uint8_t n,int16_t speed){
	if(n==1){
		if(speed>0){
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);
			__HAL_TIM_SET_COMPARE(&htim15,TIM_CHANNEL_1,speed);
		}
		else if(speed<0){
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);
			__HAL_TIM_SET_COMPARE(&htim15,TIM_CHANNEL_1,-speed);
		}
		else{
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);
			__HAL_TIM_SET_COMPARE(&htim15,TIM_CHANNEL_1,0);
		}
	}else if(n==2){
		if(speed>0){
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
			__HAL_TIM_SET_COMPARE(&htim15,TIM_CHANNEL_2,speed);
		}
		else if(speed<0){
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);
			__HAL_TIM_SET_COMPARE(&htim15,TIM_CHANNEL_2,-speed);
		}
		else{
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
			__HAL_TIM_SET_COMPARE(&htim15,TIM_CHANNEL_2,0);
		}
	}
}



void Motor_SetTarget(uint8_t n, int16_t target)
{
    target = Motor_LimitInt(target);

    if(n == 1)
    {
        motor1_target = target;
    }
    else if(n == 2)
    {
        motor2_target = target;
    }
}

void Motor_SmoothUpdate(void)
{
    motor1_current = Motor_GradientUpdate(motor1_current, motor1_target);
    motor2_current = Motor_GradientUpdate(motor2_current, motor2_target);

    Motor_SetSpeed(1, (int16_t)motor1_current);
    Motor_SetSpeed(2, (int16_t)motor2_current);
}

void Motor_Stop(uint8_t n)
{
    if(n == 1)
    {
        motor1_target = 0;
    }
    else if(n == 2)
    {
        motor2_target = 0;
    }
}

void Motor_StopAll(void)
{
    motor1_target = 0;
    motor2_target = 0;
}

int16_t Motor_GetCurrent(uint8_t n)
{
    if(n == 1)
    {
        return (int16_t)motor1_current;
    }
    else if(n == 2)
    {
        return (int16_t)motor2_current;
    }

    return 0;
}

